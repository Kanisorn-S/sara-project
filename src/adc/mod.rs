use core::ptr::{read_volatile, write_volatile};
use crate::gpio::{gpio_set_pin, GpioPort};
use crate::timer::delay;

const LED_PIN_PORT: GpioPort = GpioPort::E;
const LED_PIN: u8 = 3;

const RCC_BASE: u32 = 0x5802_4400;
const RCC_AHB1ENR_ADDR: *mut u32 = (RCC_BASE + 0x0D8) as *mut u32;
const RCC_D3CCIPR_ADDR: *mut u32 = (RCC_BASE + 0x058) as *mut u32;

const ADC1_BASE: u32 = 0x4002_2000;
const ADC1_CR_ADDR: *mut u32 = (ADC1_BASE + 0x008) as *mut u32;
const ADC1_ISR_ADDR: *mut u32 = (ADC1_BASE + 0x000) as *mut u32;
const ADC1_SQR1_ADDR: *mut u32 = (ADC1_BASE + 0x030) as *mut u32;
const ADC1_DR_ADDR: *const u32 = (ADC1_BASE + 0x040) as *const u32;
const ADC1_PCSEL: *mut u32 = (ADC1_BASE + 0x01C) as *mut u32;


pub unsafe fn rcc_enable_adc_clock() {
    const RCC_AHB1ENR_ADC12EN: u32 = 1 << 5;
    let rcc_ahb1enr = read_volatile(RCC_AHB1ENR_ADDR);
    write_volatile(RCC_AHB1ENR_ADDR, rcc_ahb1enr | RCC_AHB1ENR_ADC12EN);

    let mut rcc_d3ccipr = read_volatile(RCC_D3CCIPR_ADDR);
    rcc_d3ccipr &= !(0b11 << 16);
    rcc_d3ccipr |= 1 << 17;
    write_volatile(RCC_D3CCIPR_ADDR, rcc_d3ccipr);
}

pub unsafe fn adc_power_up() {
    // Exit Deep Power Down mode
    const ADC1_CR_DEEPPWD: u32 = 1 << 29;
    let adc1_cr = read_volatile(ADC1_CR_ADDR);
    write_volatile(ADC1_CR_ADDR, adc1_cr & !(ADC1_CR_DEEPPWD));

    // Enable ADC Voltage Regulator
    const ADC1_CR_ADVREGEN: u32 = 1 << 28;
    let adc1_cr = read_volatile(ADC1_CR_ADDR);
    write_volatile(ADC1_CR_ADDR,  adc1_cr | ADC1_CR_ADVREGEN);
    delay(20_000); // Wait for regulator startup
}

pub unsafe fn adc_calibrate() {
    const ADC1_CR_ADCALDIF: u32 = 1 << 30;
    let adc1_cr = read_volatile(ADC1_CR_ADDR);
    write_volatile(ADC1_CR_ADDR, adc1_cr & !(ADC1_CR_ADCALDIF));
    
    const ADC1_CR_ADCAL: u32 = 1 << 31;
    let adc1_cr = read_volatile(ADC1_CR_ADDR);
    write_volatile(ADC1_CR_ADDR, adc1_cr | ADC1_CR_ADCAL);

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
    let ADC1_SQR1_CHANNEL: u32 = channel << 6;
    let adc1_sqr1 = read_volatile(ADC1_SQR1_ADDR);
    write_volatile(ADC1_SQR1_ADDR, adc1_sqr1 | ADC1_SQR1_CHANNEL);
}

pub unsafe fn adc_preselect_channel(channel: u32) {
    const ADC1_PCSEL_16: u32 = 1 << 16;
    let adc1_pcsel = read_volatile(ADC1_PCSEL);
    write_volatile(ADC1_PCSEL, adc1_pcsel | ADC1_PCSEL_16);
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