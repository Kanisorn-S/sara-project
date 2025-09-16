#![no_std] #![no_main]
mod gpio;
mod timer;
mod adc;
mod uart;

use cortex_m::asm::nop;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32h7xx_hal as _;
use crate::adc::{ADC1};
use crate::gpio::{gpio_set_alternate_function, gpio_set_pin_mode, gpio_toggle_pin, rcc_enable_gpio_clock, GpioPort, PinMode};
use crate::gpio::AlternateFunction::{AF7};
use crate::timer::delay;
use crate::uart::{rcc_enable_uart1_clock, Uart};


#[entry]
fn main() -> ! {
    const LED_PIN_PORT: GpioPort = GpioPort::E;
    const LED_PIN: u8 = 3;

    const UART1_BASE_ADDR: u32 = 0x4001_1000;
    const UART1_GPIO_PORT: GpioPort = GpioPort::A;
    const UART1_TX_PIN: u8 = 9;
    const UART1_RX_PIN: u8 = 10;

    let uart1 = Uart::new(UART1_BASE_ADDR);
    
    let adc1 = ADC1::new();


    unsafe {
        rcc_enable_gpio_clock(LED_PIN_PORT);
        gpio_set_pin_mode(LED_PIN_PORT, LED_PIN, PinMode::OUTPUT);


        rcc_enable_gpio_clock(UART1_GPIO_PORT);
        gpio_set_pin_mode(UART1_GPIO_PORT, UART1_RX_PIN, PinMode::ALTERNATE);
        gpio_set_pin_mode(UART1_GPIO_PORT, UART1_TX_PIN, PinMode::ALTERNATE);

        gpio_set_alternate_function(UART1_GPIO_PORT, UART1_RX_PIN, AF7);
        gpio_set_alternate_function(UART1_GPIO_PORT, UART1_TX_PIN, AF7);

        rcc_enable_uart1_clock();

        uart1.init(115200);
        
        adc1.init();
    }


    loop {
        let adc_value: u32;
        unsafe {
            gpio_toggle_pin(LED_PIN_PORT, LED_PIN);
            delay(1_000_000);
            adc1.start_conversion();
            adc1.wait_for_conversion();
            adc_value = adc1.read_value();
            let bytes = adc_value.to_be_bytes();
            for b in &bytes {
                uart1.send_byte(*b);
            }
        }

        nop();
        delay(1_000_000);
    }
}