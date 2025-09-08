#![no_std]
#![no_main]

mod gpio;
mod timer;

use cortex_m_rt::entry;
use stm32h7xx_hal as _;
use panic_halt as _;
use crate::gpio::{gpio_set_pin_mode, gpio_toggle_pin, rcc_enable_gpio_clock, GpioPort, PinMode};
use crate::timer::delay;

#[entry]
fn main() -> ! {
    const LED_PORT: GpioPort = GpioPort::E;
    const LED_PIN: u8 = 3;
    
    unsafe {
        rcc_enable_gpio_clock(LED_PORT);
        gpio_set_pin_mode(LED_PORT, LED_PIN, PinMode::OUTPUT);
    }
    
    loop {
        unsafe {
            gpio_toggle_pin(LED_PORT, LED_PIN);
        }
        delay(400_000);
    }
}