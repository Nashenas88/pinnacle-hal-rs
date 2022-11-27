#![no_std]
#![no_main]

use core::cell::RefCell;

use bsp::hal::gpio::bank0::{Gpio21, Gpio9};
use bsp::hal::gpio::{Interrupt, Pin, PullDownInput, PushPullOutput};
use bsp::hal::pac::interrupt;
use bsp::hal::spi::Enabled;
use bsp::hal::usb::UsbBus;
use bsp::hal::{gpio, spi, Spi};
use bsp::pac::SPI0;
use bsp::{entry, pac};
use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::InputPin;
use fugit::RateExtU32;
use panic_probe as _;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceState;
use usb_device::prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_hid::descriptor::{MouseReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;

use pinnacle_touchpad::{PinnacleTouchpad, PinnacleTouchpadBuilder, RelativeData};
use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::clocks::{init_clocks_and_plls, Clock};
use bsp::hal::sio::Sio;
use bsp::hal::watchdog::Watchdog;

type TouchpadDrPin = Pin<Gpio9, PullDownInput>;
type TouchpadSpi = Spi<Enabled, SPI0, 8>;
type SpiCs = Pin<Gpio21, PushPullOutput>;

static mut PINNACLE: Option<(
    Delay,
    TouchpadDrPin,
    PinnacleTouchpad<TouchpadSpi, SpiCs, RelativeData>,
)> = None;
static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;
static mut USB_DEV: Option<Mutex<RefCell<UsbDevice<'static, UsbBus>>>> = None;
static mut MOUSE_CLASS: Option<Mutex<RefCell<HIDClass<'static, UsbBus>>>> = None;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Only used by debug connector.
    let _pin0 = pins.tx0;
    let _pin1 = pins.tx1;

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The bus that is used to manage the device and class below.
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    unsafe { USB_BUS = Some(usb_bus) };

    // The class which specifies this device supports HID Mouse reports.
    let mouse_class = HIDClass::new(
        unsafe { USB_BUS.as_ref().unwrap() },
        MouseReport::desc(),
        60,
    );

    let usb_dev = UsbDeviceBuilder::new(
        unsafe { USB_BUS.as_ref().unwrap() },
        // vid and pid for generic mouse from
        // https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
        UsbVidPid(0x16c0, 0x27da),
    )
    .manufacturer("Cirque")
    .product("Pinnacle")
    .serial_number(env!("CARGO_PKG_VERSION"))
    .build();
    unsafe {
        USB_DEV = Some(Mutex::new(RefCell::new(usb_dev)));
        MOUSE_CLASS = Some(Mutex::new(RefCell::new(mouse_class)));
    }

    let touchpad_irq = pins.rx1.into_mode::<PullDownInput>();
    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.sck.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.copi.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.cipo.into_mode::<gpio::FunctionSpi>();
    let spi_cs = pins.ncs.into_push_pull_output();

    // // Create an SPI driver instance for the SPI0 device
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI0);

    // // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        &embedded_hal::spi::MODE_1,
    );

    touchpad_irq.set_interrupt_enabled(Interrupt::EdgeHigh, true);
    let mut pinnacle = PinnacleTouchpadBuilder::new(spi, spi_cs)
        .relative_mode()
        .calibrate()
        .build(&mut delay)
        .unwrap();

    let firmware_id_bytes = pinnacle.firmware_id().unwrap();
    delay.delay_us(1);
    let firmware_ver_bytes = pinnacle.firmware_version().unwrap();
    delay.delay_us(1);
    let product_id = pinnacle.product_id().unwrap();
    delay.delay_us(1);
    let status_bytes = pinnacle.status().unwrap();
    delay.delay_us(1);
    defmt::info!(
        "Firmware ID: {}, Version: {}, Product ID: {}, status: {}",
        firmware_id_bytes,
        firmware_ver_bytes,
        product_id,
        status_bytes
    );

    pinnacle.set_z_idle(0x0A).unwrap(); // Set z-idle packet count to 5 (default is 30)

    unsafe {
        PINNACLE = Some((delay, touchpad_irq, pinnacle));
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    }

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn USBCTRL_IRQ() {
    cortex_m::interrupt::free(|cs| {
        let mut usb_dev = unsafe { USB_DEV.as_ref().unwrap().borrow(cs).borrow_mut() };
        let mut mouse_class = unsafe { MOUSE_CLASS.as_ref().unwrap().borrow(cs).borrow_mut() };
        let _ = usb_dev.poll(&mut [&mut *mouse_class]);
    });
}

#[interrupt]
fn IO_IRQ_BANK0() {
    let Some((delay, touchpad_irq, pinnacle)) = (unsafe { PINNACLE.as_mut() }) else {
        return;
    };

    if !touchpad_irq.interrupt_status(Interrupt::EdgeHigh) {
        return;
    }

    let data = pinnacle.read_relative().unwrap();
    touchpad_irq.clear_interrupt(Interrupt::EdgeHigh);
    pinnacle.clear_flags(delay).unwrap();

    cortex_m::interrupt::free(|cs| {
        let usb_dev = unsafe { USB_DEV.as_ref().unwrap().borrow(cs).borrow_mut() };
        let mouse_class = unsafe { MOUSE_CLASS.as_ref().unwrap().borrow(cs).borrow() };

        // If the device is not configured yet, we need to bail out.
        if usb_dev.state() != UsbDeviceState::Configured {
            return;
        }

        // Watchdog will prevent the keyboard from getting stuck in this loop.
        let mouse_report = MouseReport {
            buttons: 0,
            x: (data.y * -5) as i8,
            y: (data.x * -5) as i8,
            wheel: 0,
            pan: 0,
        };
        while let Ok(0) = mouse_class.push_input(&mouse_report) {}
    });
}

// End of file
