#![no_std] #![no_main]
mod gpio;
mod timer;
mod uart;

use cortex_m_rt::entry;
use panic_halt as _;
use stm32h7xx_hal as _;
use crate::gpio::{gpio_set_alternate_function, gpio_set_pin_mode, gpio_toggle_pin, rcc_enable_gpio_clock, GpioPort, PinMode};
use crate::gpio::AlternateFunction::{AF6, AF7};
use crate::timer::delay;
use crate::uart::{rcc_enable_uart1_clock, rcc_enable_uart4_clock, Uart};


#[entry]
fn main() -> ! {
    const LED_PORT: GpioPort = GpioPort::E;
    const LED_PIN: u8 = 3;

    const UART1_BASE_ADDR: u32 = 0x4001_1000;
    const UART1_GPIO_PORT: GpioPort = GpioPort::A;
    const UART1_TX_PIN: u8 = 9;
    const UART1_RX_PIN: u8 = 10;

    // const UART4_BASE_ADDR: u32 = 0x4000_4C00;
    // const UART4_GPIO_PORT: GpioPort = GpioPort::A;
    // const UART4_RX_PIN: u8 = 11;
    // const UART4_TX_PIN: u8 = 12;

    let uart1 = Uart::new(UART1_BASE_ADDR);
    // let uart4 = Uart::new(UART4_BASE_ADDR);


    unsafe {
        rcc_enable_gpio_clock(LED_PORT);
        gpio_set_pin_mode(LED_PORT, LED_PIN, PinMode::OUTPUT);

        rcc_enable_gpio_clock(UART1_GPIO_PORT);
        gpio_set_pin_mode(UART1_GPIO_PORT, UART1_RX_PIN, PinMode::ALTERNATE);
        gpio_set_pin_mode(UART1_GPIO_PORT, UART1_TX_PIN, PinMode::ALTERNATE);
        // rcc_enable_gpio_clock(UART4_GPIO_PORT);
        // gpio_set_pin_mode(UART4_GPIO_PORT, UART4_RX_PIN, PinMode::ALTERNATE);
        // gpio_set_pin_mode(UART4_GPIO_PORT, UART4_TX_PIN, PinMode::ALTERNATE);

        gpio_set_alternate_function(UART1_GPIO_PORT, UART1_RX_PIN, AF7);
        gpio_set_alternate_function(UART1_GPIO_PORT, UART1_TX_PIN, AF7);
        // gpio_set_alternate_function(UART4_GPIO_PORT, UART4_RX_PIN, AF6);
        // gpio_set_alternate_function(UART4_GPIO_PORT, UART4_TX_PIN, AF6);

        rcc_enable_uart1_clock();
        // rcc_enable_uart4_clock();

        uart1.init(115200);
        // uart4.init(115200);

    }

    let mut counter: u32 = 0;

    loop {

        unsafe {
            gpio_toggle_pin(LED_PORT, LED_PIN);
            // uart1.send_byte(9);
            uart1.send_string("Testing");
            // uart4.send_byte(9);
            // uart4.send_string("Hello");
        }

        counter += 1;
        delay(400_000);
    }
}