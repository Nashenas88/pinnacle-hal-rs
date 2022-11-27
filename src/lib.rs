//! TODO: Add documentation
#![no_std]

use core::marker::PhantomData;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::CountDown;
use fugit::{ExtU32, MicrosDurationU32};

pub struct PinnacleTouchpad<S, P, D> {
    spi: S,
    cs: P,
    phantom_: PhantomData<D>,
}

mod private {
    pub trait Sealed {}
}

pub struct PinnacleTouchpadBuilder<S, P, D> {
    spi: S,
    cs: P,
    x: bool,
    y: bool,
    filter: bool,
    swap_x_y: bool,
    glide_extend: bool,
    scroll: bool,
    secondary_tap: bool,
    all_taps: bool,
    intellimouse: bool,
    calibrate: bool,
    data: D,
}

impl<S, P> PinnacleTouchpadBuilder<S, P, ()> {
    pub fn new(spi: S, cs: P) -> Self {
        Self {
            spi,
            cs,
            x: true,
            y: true,
            filter: true,
            swap_x_y: true,
            glide_extend: true,
            scroll: true,
            secondary_tap: true,
            all_taps: true,
            intellimouse: false,
            calibrate: false,
            data: (),
        }
    }

    pub fn relative_mode(self) -> PinnacleTouchpadBuilder<S, P, Relative> {
        PinnacleTouchpadBuilder {
            spi: self.spi,
            cs: self.cs,
            x: self.x,
            y: self.y,
            filter: self.filter,
            swap_x_y: self.swap_x_y,
            glide_extend: self.glide_extend,
            scroll: self.scroll,
            secondary_tap: self.secondary_tap,
            all_taps: self.all_taps,
            intellimouse: self.intellimouse,
            calibrate: self.calibrate,
            data: Relative,
        }
    }

    pub fn absolute_mode(self) -> PinnacleTouchpadBuilder<S, P, Absolute> {
        PinnacleTouchpadBuilder {
            spi: self.spi,
            cs: self.cs,
            x: self.x,
            y: self.y,
            filter: self.filter,
            swap_x_y: self.swap_x_y,
            glide_extend: self.glide_extend,
            scroll: self.scroll,
            secondary_tap: self.secondary_tap,
            all_taps: self.all_taps,
            intellimouse: self.intellimouse,
            calibrate: self.calibrate,
            data: Absolute {
                invert_x: false,
                invert_y: false,
            },
        }
    }
}

impl<S, P, D> PinnacleTouchpadBuilder<S, P, D> {
    pub fn enable_x(mut self) -> Self {
        self.x = true;
        self
    }
    pub fn disable_x(mut self) -> Self {
        self.x = false;
        self
    }
    pub fn enable_y(mut self) -> Self {
        self.y = true;
        self
    }
    pub fn disable_y(mut self) -> Self {
        self.y = false;
        self
    }
    pub fn enable_filter(mut self) -> Self {
        self.filter = true;
        self
    }
    pub fn disable_filter(mut self) -> Self {
        self.filter = false;
        self
    }
    pub fn swap_x_y(mut self, swap: bool) -> Self {
        self.swap_x_y = swap;
        self
    }
    pub fn enable_glide_extend(mut self) -> Self {
        self.glide_extend = true;
        self
    }
    pub fn disable_glide_extend(mut self) -> Self {
        self.glide_extend = false;
        self
    }
    pub fn enable_scroll(mut self) -> Self {
        self.scroll = true;
        self
    }
    pub fn disable_scroll(mut self) -> Self {
        self.scroll = false;
        self
    }
    pub fn enable_secondary_tap(mut self) -> Self {
        self.secondary_tap = true;
        self
    }
    pub fn disable_secondary_tap(mut self) -> Self {
        self.secondary_tap = false;
        self
    }
    pub fn enable_all_taps(mut self) -> Self {
        self.all_taps = true;
        self
    }
    pub fn disable_all_taps(mut self) -> Self {
        self.all_taps = false;
        self
    }
    pub fn enable_intellimouse(mut self) -> Self {
        self.intellimouse = true;
        self
    }
    pub fn disable_intellimouse(mut self) -> Self {
        self.intellimouse = false;
        self
    }
    pub fn calibrate(mut self) -> Self {
        self.calibrate = true;
        self
    }
}

impl<S, P> PinnacleTouchpadBuilder<S, P, Relative> {}

impl<S, P> PinnacleTouchpadBuilder<S, P, Absolute> {
    pub fn invert_x(mut self, invert: bool) -> Self {
        self.data.invert_x = invert;
        self
    }
}

impl<S, P> PinnacleTouchpadBuilder<S, P, Absolute> {
    pub fn invert_y(mut self, invert: bool) -> Self {
        self.data.invert_y = invert;
        self
    }
}

pub trait Build: private::Sealed {
    type Data: private::Sealed;
    fn build(&self, feed_config1: &mut u8);
}

impl private::Sealed for Absolute {}
impl Build for Absolute {
    type Data = AbsoluteData;
    fn build(&self, feed_config1: &mut u8) {
        *feed_config1 |= (self.invert_y as u8) << 7 | (self.invert_x as u8) << 6 | 1 << 1;
    }
}

impl private::Sealed for Relative {}
impl Build for Relative {
    type Data = RelativeData;
    fn build(&self, feed_config1: &mut u8) {
        *feed_config1 &= !(1 << 1);
    }
}

impl<S, P, D> PinnacleTouchpadBuilder<S, P, D>
where
    S: Transfer<u8>,
    P: OutputPin,
    D: Build,
    <D as Build>::Data: TouchpadData,
{
    pub fn build<C>(
        self,
        delay: &mut C,
    ) -> Result<PinnacleTouchpad<S, P, <D as Build>::Data>, S::Error>
    where
        C: CountDown,
        MicrosDurationU32: Into<<C as CountDown>::Time>,
    {
        let mut pinnacle = PinnacleTouchpad::new(self.spi, self.cs);
        pinnacle.write(STATUS1_ADDR, 0x00)?;
        delay.start(50.micros());
        let _ = delay.wait();
        let feed_config2 = (self.swap_x_y as u8) << 7
            | (!self.glide_extend as u8) << 4
            | (!self.scroll as u8) << 4
            | (!self.secondary_tap as u8) << 2
            | (!self.all_taps as u8) << 1
            | (self.intellimouse as u8);
        pinnacle.write(FEED_CONFIG2_ADDR, feed_config2)?;
        if self.calibrate {
            let calibrate_config = 1 << 4 | 1 << 3 | 1 << 2 | 1 << 1 | 1;
            pinnacle.write(CAL_CONFIG1_ADDR, calibrate_config)?;
        }

        let mut feed_config1 =
            1 | (!self.y as u8) << 4 | (!self.x as u8) << 3 | (!self.filter as u8) << 2;
        self.data.build(&mut feed_config1);
        pinnacle.write(FEED_CONFIG1_ADDR, feed_config1)?;
        Ok(pinnacle)
    }
}

impl<S, P, D> PinnacleTouchpad<S, P, D>
where
    D: TouchpadData,
{
    fn new(spi: S, cs: P) -> Self {
        Self {
            spi,
            cs,
            phantom_: PhantomData,
        }
    }
}

pub enum SampleRate {
    OneHundred,
    Eighty,
    Sixty,
    Forty,
    Twenty,
    Ten,
}

impl<S, P, D> PinnacleTouchpad<S, P, D>
where
    S: Transfer<u8>,
    P: OutputPin,
    D: TouchpadData,
{
    pub fn clear_flags<C>(&mut self, delay: &mut C) -> Result<(), S::Error>
    where
        C: CountDown,
        MicrosDurationU32: Into<<C as CountDown>::Time>,
    {
        let res = self.write(STATUS1_ADDR, 0x00);
        delay.start(50.micros());
        let _ = delay.wait();
        res
    }

    pub fn product_id(&mut self) -> Result<u8, S::Error> {
        self.read(PRODUCT_ID_ADDR)
    }

    pub fn firmware_id(&mut self) -> Result<u8, S::Error> {
        self.read(FIRMWARE_ID_ADDR)
    }

    pub fn firmware_version(&mut self) -> Result<u8, S::Error> {
        self.read(FIRMWARE_VERSION_ADDR)
    }

    pub fn status(&mut self) -> Result<u8, S::Error> {
        self.read(STATUS1_ADDR)
    }

    /*
    100 Sample/Second 64H
    80  Sample/Second 50H
    60  Sample/Second 3CH
    40  Sample/Second 28H
    20  Sample/Second 14H
    10  Sample/Second 0Ah
     */

    pub fn sample_rate(&mut self) -> Result<SampleRate, S::Error> {
        todo!()
    }

    pub fn set_sample_rate(&mut self, _sample_rate: SampleRate) -> Result<(), S::Error> {
        todo!()
    }

    pub fn z_idle(&mut self) -> Result<u8, S::Error> {
        self.read(Z_IDLE_ADDR)
    }

    pub fn set_z_idle(&mut self, z_idle: u8) -> Result<(), S::Error> {
        self.write(Z_IDLE_ADDR, z_idle)
    }

    pub fn z_scaler(&mut self) -> Result<u8, S::Error> {
        self.read(Z_SCALER_ADDR)
    }

    pub fn set_z_scaler(&mut self, z_scaler: u8) -> Result<(), S::Error> {
        self.write(Z_SCALER_ADDR, z_scaler)
    }

    // Read a byte from `addr`.
    fn read(&mut self, addr: u8) -> Result<u8, S::Error> {
        let addr = READ_BITS | (addr & ADDR_MASK);
        let mut buf = [addr, READ_FILL, READ_FILL, READ_FILL];
        let _ = self.cs.set_low();
        let _ = self.spi.transfer(&mut buf);
        let _ = self.cs.set_high();
        Ok(buf[3])
    }

    fn read_multi<const N: usize>(&mut self, addr: u8) -> Result<[u8; N], S::Error> {
        let addr = READ_BITS | (addr & ADDR_MASK);
        let mut buf = [READ_CONTINUE; N];
        buf[N - 1] = READ_FILL;
        let _ = self.cs.set_low();
        let mut addr_buf = [addr, READ_CONTINUE, READ_CONTINUE];
        let _ = self.spi.transfer(&mut addr_buf);
        let _ = self.spi.transfer(&mut buf);
        let _ = self.cs.set_high();
        Ok(buf)
    }

    fn write(&mut self, addr: u8, data: u8) -> Result<(), S::Error> {
        let addr = WRITE_BITS | (addr & ADDR_MASK);
        let mut buf = [addr, data];

        let _ = self.cs.set_low();
        let _ = self.spi.transfer(&mut buf);
        let _ = self.cs.set_high();

        Ok(())
    }
}
pub struct Relative;
pub struct Absolute {
    invert_x: bool,
    invert_y: bool,
}

impl private::Sealed for RelativeData {}
impl private::Sealed for AbsoluteData {}

pub trait TouchpadData: private::Sealed {}
impl TouchpadData for AbsoluteData {}
impl TouchpadData for RelativeData {}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AbsoluteData {
    pub x: u16,
    pub y: u16,
    pub z: u8,
    pub button_flags: u8,
}

impl<S, P> PinnacleTouchpad<S, P, AbsoluteData>
where
    S: Transfer<u8>,
    P: OutputPin,
{
    pub fn read_absolute(&mut self) -> Result<AbsoluteData, S::Error> {
        let data = self.read_multi::<6>(PACKET_BYTE_0_ADDR)?;
        Ok(AbsoluteData {
            x: data[2] as u16 | (((data[4] & 0x0F) as u16) << 8),
            y: data[3] as u16 | (((data[4] & 0xF0) as u16) << 4),
            z: data[5] & 0x3F,
            button_flags: data[0] & 0x3F,
        })
    }
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RelativeData {
    pub x: i16,
    pub y: i16,
    pub button_flags: u8,
    pub wheel: i8,
}

impl<S, P> PinnacleTouchpad<S, P, RelativeData>
where
    S: Transfer<u8>,
    P: OutputPin,
{
    pub fn read_relative(&mut self) -> Result<RelativeData, S::Error> {
        let data = self.read_multi::<4>(PACKET_BYTE_0_ADDR)?;
        let mut x = data[1] as i16;
        let mut y = data[2] as i16;
        if (data[0] & 0x10) > 0 {
            x -= 256;
        }
        if (data[0] & 0x20) > 0 {
            y -= 256;
        }
        Ok(RelativeData {
            x,
            y,
            button_flags: data[0] & 0x07,
            wheel: data[3] as i8,
        })
    }
}

const WRITE_BITS: u8 = 0b_1000_0000;
const READ_BITS: u8 = 0b_1010_0000;
const ADDR_MASK: u8 = 0b_0001_1111;
const READ_FILL: u8 = 0xFB;
const READ_CONTINUE: u8 = 0xFC;

const FIRMWARE_ID_ADDR: u8 = 0x00;
const FIRMWARE_VERSION_ADDR: u8 = 0x01;
const STATUS1_ADDR: u8 = 0x02;
const SYS_CONFIG1_ADDR: u8 = 0x03;
const FEED_CONFIG1_ADDR: u8 = 0x04;
const FEED_CONFIG2_ADDR: u8 = 0x05;
const CAL_CONFIG1_ADDR: u8 = 0x07;
const PS2_AUX_CTRL_ADDR: u8 = 0x08;
const SAMPLE_RATE_ADDR: u8 = 0x09;
const Z_IDLE_ADDR: u8 = 0x0A;
const Z_SCALER_ADDR: u8 = 0x0B;
const SLEEP_INTERVAL_ADDR: u8 = 0x0C;
const SLEEP_TIMER_ADDR: u8 = 0x0D;
const PACKET_BYTE_0_ADDR: u8 = 0x12;
const PACKET_BYTE_1_ADDR: u8 = 0x13;
const PACKET_BYTE_2_ADDR: u8 = 0x14;
const PACKET_BYTE_3_ADDR: u8 = 0x15;
const PACKET_BYTE_4_ADDR: u8 = 0x16;
const PACKET_BYTE_5_ADDR: u8 = 0x17;
const PRODUCT_ID_ADDR: u8 = 0x1F;
