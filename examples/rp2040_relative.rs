#![no_std]
#![no_main]

use bsp::hal::gpio::bank0::{Gpio21, Gpio9};
use bsp::hal::gpio::{Interrupt, Pin, PullDownInput, PushPullOutput};
use bsp::hal::pac::interrupt;
use bsp::hal::spi::Enabled;
use bsp::hal::{gpio, spi, Spi, Timer};
use bsp::pac::SPI0;
use bsp::{entry, pac};
use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::InputPin;
use fugit::RateExtU32;
use panic_probe as _;

use pinnacle_touchpad::{PinnacleTouchpad, PinnacleTouchpadBuilder, RelativeData};
use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::clocks::{init_clocks_and_plls, Clock};
use bsp::hal::sio::Sio;
use bsp::hal::watchdog::Watchdog;

type TouchpadDrPin = Pin<Gpio9, PullDownInput>;
type TouchpadSpi = Spi<Enabled, SPI0, 8>;
type SpiCs = Pin<Gpio21, PushPullOutput>;

static mut PINNACLE: Option<(
    Timer,
    TouchpadDrPin,
    PinnacleTouchpad<TouchpadSpi, SpiCs, RelativeData>,
)> = None;

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
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

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
        .build(&mut timer.count_down())
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
        PINNACLE = Some((timer, touchpad_irq, pinnacle));
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    let Some((timer, touchpad_irq, pinnacle)) = (unsafe { PINNACLE.as_mut() }) else {
        return;
    };

    if !touchpad_irq.interrupt_status(Interrupt::EdgeHigh) {
        return;
    }

    let data = pinnacle.read_relative().unwrap();
    touchpad_irq.clear_interrupt(Interrupt::EdgeHigh);
    pinnacle.clear_flags(&mut timer.count_down()).unwrap();
    defmt::info!("{:?}", data);
}

// End of file
