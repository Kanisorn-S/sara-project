#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use stm32h7xx_hal::{
  pac,
  prelude::*,
  spi,
  delay::Delay,
};
use embedded_hal_compat::ForwardCompat;
use embedded_hal_bus::spi::ExclusiveDevice;
use mcp4x::{Channel, Mcp4x};
use panic_halt as _;
use embedded_hal::delay::DelayNs;

struct NoopDelay;
impl DelayNs for NoopDelay {
    fn delay_ns(&mut self, _ns: u32) {
        // Do nothing
    }
}

#[entry]
fn main() -> ! {
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // === Power + Clocks ===
  let pwr = dp.PWR.constrain();
  let pwrcfg = pwr.freeze();
  let rcc = dp.RCC.constrain();
  let ccdr = rcc
    .sys_ck(96.MHz())
    .pll1_q_ck(48.MHz())
    .freeze(pwrcfg, &dp.SYSCFG);

  // === GPIO ===
  let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
  let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

  // === LED ===
  let mut led = gpioe.pe3.into_push_pull_output();

  // SPI pins
  let sck = gpiob.pb13.into_alternate();
  let miso = gpiob.pb14.into_alternate();
  let mosi = gpiob.pb15.into_alternate();
  let cs = gpiob.pb12.into_push_pull_output();

  // === SPI peripheral ===
  let spi = dp.SPI2.spi(
    (sck, miso, mosi),
    spi::MODE_0,
    3.MHz(),
    ccdr.peripheral.SPI2,
    &ccdr.clocks,
  );

  // === Delay ===
  let _delay = Delay::new(cp.SYST, ccdr.clocks);

  // === Convert all to embedded-hal 1.0 traits ===
  let spi_v1 = spi.forward();
  let cs_v1 = cs.forward();
  // let delay_v1 = delay.forward();

  // === Wrap in ExclusiveDevice ===
  let dev = ExclusiveDevice::new(spi_v1, cs_v1, NoopDelay).unwrap();

  // === Construct the MCP41010 ===
  let mut mcp41x = Mcp4x::new_mcp41x(dev);

  // === Control the potentiometer ===
  mcp41x.set_position(Channel::Ch0, 0).unwrap();

  loop {
    led.toggle();
    cortex_m::asm::delay(10_000_000);
  }
}

