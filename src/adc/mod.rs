use core::ptr::{read_volatile, write_volatile};
use crate::timer::delay;

const RCC_BASE: u32 = 0x5802_4400;
const RCC_AHB1ENR_ADDR: *mut u32 = (RCC_BASE + 0x0D8) as *mut u32;

const ADC1_BASE: u32 = 0x4002_2000;
const ADC1_CR_ADDR: *mut u32 = (ADC1_BASE + 0x008) as *mut u32;
const ADC1_ISR_ADDR: *mut u32 = (ADC1_BASE + 0x000) as *mut u32;
const ADC1_SQR1_ADDR: *mut u32 = (ADC1_BASE + 0x030) as *mut u32;
const ADC1_DR_ADDR: *const u32 = (ADC1_BASE + 0x040) as *const u32;

pub unsafe fn rcc_enable_adc_clock() {
    const RCC_AHB1ENR_ADC12EN: u32 = 1 << 5;
    let rcc_val = read_volatile(RCC_AHB1ENR_ADDR);
    write_volatile(RCC_AHB1ENR_ADDR, rcc_val | RCC_AHB1ENR_ADC12EN);
}

pub unsafe fn adc_power_up() {
    write_volatile(ADC1_CR_ADDR, 1 << 28); // ADVREGEN = 1
    delay(20_000); // Wait for regulator startup
}

pub unsafe fn adc_calibrate() {
    // Start calibration with ADVREGEN still enabled
    write_volatile(ADC1_CR_ADDR, (1 << 31) | (1 << 28)); // ADCAL = 1
    // Wait until calibration is finished (ADCAL bit goes to 0)
    while (read_volatile(ADC1_CR_ADDR) & (1 << 31)) != 0 {}
}

pub unsafe fn adc_enable() {
    // Enable the ADC with ADVREGEN still enabled
    write_volatile(ADC1_CR_ADDR, (1 << 0) | (1 << 28)); // ADEN = 1
    // Wait for ADC to be ready (ADRDY flag in ISR)
    while (read_volatile(ADC1_ISR_ADDR) & 1) == 0 {}
    // Clear the ADRDY flag by writing 1 to it
    write_volatile(ADC1_ISR_ADDR, 1);
}

pub unsafe fn adc_configure_single_conversion(channel: u32) {
    // Set sequence length to 1 (L=0) and first conversion to the desired channel
    write_volatile(ADC1_SQR1_ADDR, channel << 6);
}

pub unsafe fn adc_start_conversion() {
    let cr_val = read_volatile(ADC1_CR_ADDR);
    write_volatile(ADC1_CR_ADDR, cr_val | (1 << 2)); // ADSTART = 1
}

pub unsafe fn adc_wait_for_conversion() {
    // Wait for End of Conversion (EOC flag in ISR)
    while (read_volatile(ADC1_ISR_ADDR) & (1 << 2)) == 0 {}
}

pub unsafe fn adc_read_value() -> u32 {
    let result = read_volatile(ADC1_DR_ADDR);
    // Clear the EOC flag by writing 1 to it
    write_volatile(ADC1_ISR_ADDR, 1 << 2);
    result
}