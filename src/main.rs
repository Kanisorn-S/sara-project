#![no_std]
#![no_main]

mod gpio;
mod timer;
mod adc;

use cortex_m::asm::nop;
use cortex_m_rt::entry;
use stm32h7xx_hal as _;
use panic_halt as _;
use crate::adc::{adc_calibrate, adc_configure_single_conversion, adc_enable, adc_power_up, adc_read_value, adc_start_conversion, adc_wait_for_conversion, rcc_enable_adc_clock};
use crate::gpio::{gpio_set_pin_mode, gpio_toggle_pin, rcc_enable_gpio_clock, GpioPort, PinMode};
use crate::timer::delay;

#[entry]
fn main() -> ! {
    const ADC_PIN_PORT: GpioPort = GpioPort::A;
    const ADC_PIN: u8 = 0;
    const ADC_CHANNEL: u32 = 0;
    
    unsafe {
        rcc_enable_gpio_clock(ADC_PIN_PORT);
        rcc_enable_adc_clock();
        
        gpio_set_pin_mode(ADC_PIN_PORT, ADC_PIN, PinMode::ANALOG);
        
        adc_power_up();
        adc_calibrate();
        adc_enable();
        adc_configure_single_conversion(ADC_CHANNEL);
    }
    
    loop {
        let adc_value: u32;
        unsafe {
            adc_start_conversion();
            adc_wait_for_conversion();
            adc_value = adc_read_value();
        }
        
        nop();
        delay(100_000);
    }
}