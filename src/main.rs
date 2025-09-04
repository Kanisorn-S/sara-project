#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};
use cortex_m::asm::nop;
use cortex_m_rt::entry;
use stm32h7xx_hal as _;
use panic_halt as _;

#[entry]
fn main() -> ! {
    const RCC_AHB4ENR_ADDR: *mut u32 = 0x5802_44E0 as *mut u32;
    const GPIOE_MODER_ADDR: *mut u32 = 0x5802_1000 as *mut u32;
    const GPIOE_ODR_ADDR: *mut u32 = 0x5802_1014 as *mut u32;

    unsafe {
        const RCC_AHB4ENR_GPIOEEN: u32 = 1 << 4;
        let rcc_val = read_volatile(RCC_AHB4ENR_ADDR);
        write_volatile(RCC_AHB4ENR_ADDR, rcc_val | RCC_AHB4ENR_GPIOEEN);

        let current_moder = read_volatile(GPIOE_MODER_ADDR);

        let cleared_moder = current_moder & !(0b11 << 6);

        let output_moder = cleared_moder | (0b01 << 6);

        write_volatile(GPIOE_MODER_ADDR, output_moder);
    }

    const TOGGLE_GPIOE_PIN_3: u32 = 1 << 3;

    loop {
        unsafe {
            let current_gpioe = read_volatile(GPIOE_ODR_ADDR);
            let toggle_pin_3 = current_gpioe ^ TOGGLE_GPIOE_PIN_3;
            write_volatile(GPIOE_ODR_ADDR, toggle_pin_3);
        }

        for _ in 0..400_000 {
            nop();
        }
    }
}