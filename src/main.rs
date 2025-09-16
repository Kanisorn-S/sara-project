#![no_std] #![no_main]
mod gpio;
mod timer;
mod adc;
mod uart;

use cortex_m::asm::nop;
use cortex_m_rt::entry;
use panic_halt as _;
use crate::adc::{adc_calibrate, adc_configure_single_conversion, adc_enable, adc_power_up, adc_preselect_channel, adc_read_calfact, adc_read_ccr, adc_read_diff, adc_read_pcsel, adc_read_sqr, adc_read_value, adc_set_diff, adc_start_conversion, adc_wait_for_conversion, configure_pwr, rcc_enable_adc_clock, rcc_read_d3ccipr};
use stm32h7xx_hal as _;
use crate::gpio::{gpio_read_pin_mode, gpio_set_alternate_function, gpio_set_pin_mode, gpio_toggle_pin, rcc_enable_gpio_clock, GpioPort, PinMode};
use crate::gpio::AlternateFunction::{AF6, AF7};
use crate::timer::delay;
use crate::uart::{rcc_enable_uart1_clock, rcc_enable_uart4_clock, Uart};


#[entry]
fn main() -> ! {
    const LED_PIN_PORT: GpioPort = GpioPort::E;
    const LED_PIN: u8 = 3;
    const ADC_PIN_PORT: GpioPort = GpioPort::A;
    const ADC_PIN_P: u8 = 0;
    const ADC_PIN_N: u8 = 1;
    const ADC_CHANNEL: u32 = 16;

    const UART1_BASE_ADDR: u32 = 0x4001_1000;
    const UART1_GPIO_PORT: GpioPort = GpioPort::A;
    const UART1_TX_PIN: u8 = 9;
    const UART1_RX_PIN: u8 = 10;

    let uart1 = Uart::new(UART1_BASE_ADDR);


    unsafe {
        rcc_enable_gpio_clock(LED_PIN_PORT);
        gpio_set_pin_mode(LED_PIN_PORT, LED_PIN, PinMode::OUTPUT);

        rcc_enable_gpio_clock(ADC_PIN_PORT);
        rcc_enable_adc_clock();
        
        gpio_set_pin_mode(ADC_PIN_PORT, ADC_PIN_P, PinMode::ANALOG);
        // gpio_set_pin_mode(ADC_PIN_PORT, ADC_PIN_N, PinMode::ANALOG);

        configure_pwr();
        adc_power_up();
        adc_configure_single_conversion(ADC_CHANNEL);
        adc_preselect_channel(ADC_CHANNEL);
        adc_set_diff();
        adc_calibrate();
        adc_enable();

        gpio_set_pin_mode(UART1_GPIO_PORT, UART1_RX_PIN, PinMode::ALTERNATE);
        gpio_set_pin_mode(UART1_GPIO_PORT, UART1_TX_PIN, PinMode::ALTERNATE);

        gpio_set_alternate_function(UART1_GPIO_PORT, UART1_RX_PIN, AF7);
        gpio_set_alternate_function(UART1_GPIO_PORT, UART1_TX_PIN, AF7);

        rcc_enable_uart1_clock();

        uart1.init(115200);
    }

    let mut counter: u32 = 0;

    loop {
        let adc_value: u32;
        let calfact: u32;
        let diffsel: u32;
        let sqr: u32;
        let pcsel: u32;
        let moder: u32;
        let d3ccipr: u32;
        let ccr: u32;
        unsafe {
            gpio_toggle_pin(LED_PIN_PORT, LED_PIN);
            delay(1_000_000);
            adc_start_conversion();
            adc_wait_for_conversion();
            gpio_toggle_pin(LED_PIN_PORT, LED_PIN);
            adc_value = adc_read_value();
            calfact = adc_read_calfact();
            diffsel = adc_read_diff();
            sqr = adc_read_sqr();
            pcsel = adc_read_pcsel();
            moder = gpio_read_pin_mode(GpioPort::A);
            d3ccipr = rcc_read_d3ccipr();
            ccr = adc_read_ccr();
            let bytes = adc_value.to_be_bytes();
            for b in &bytes {
                uart1.send_byte(*b);
            }
        }

        nop();
        delay(1_000_000);
    }
}