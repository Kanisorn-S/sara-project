use core::ptr::{read_volatile, write_volatile};
use crate::timer::delay;

const RCC_BASE: u32 = 0x5802_4400;
const RCC_AHB1ENR_ADDR: *mut u32 = (RCC_BASE + 0x0D8) as *mut u32;
const RCC_D3CCIPR_ADDR: *mut u32 = (RCC_BASE + 0x058) as *mut u32;

const ADC1_BASE: u32 = 0x4002_2000;
const ADC1_CR_ADDR: *mut u32 = (ADC1_BASE + 0x008) as *mut u32;
const ADC1_ISR_ADDR: *mut u32 = (ADC1_BASE + 0x000) as *mut u32;
const ADC1_SQR1_ADDR: *mut u32 = (ADC1_BASE + 0x030) as *mut u32;
const ADC1_DR_ADDR: *const u32 = (ADC1_BASE + 0x040) as *const u32;
const ADC1_PCSEL: *mut u32 = (ADC1_BASE + 0x01C) as *mut u32;
const ADC1_DIFSEL: *mut u32 = (ADC1_BASE + 0x0C0) as *mut u32;
const ADC1_CALFACT: *const u32 = (ADC1_BASE + 0x0C4) as *const u32;

const ADC_COMMON_BASE_ADDR: u32 = ADC1_BASE + 0x300;
const ADC_CCR: *mut u32 = (ADC_COMMON_BASE_ADDR + 0x08) as *mut u32;

const VREFBUF_ADDR: u32 = 0x5800_3C00;
const VREFBUF_CSR_ADDR: *mut u32 = (VREFBUF_ADDR + 0x00) as *mut u32;

pub unsafe fn configure_pwr() {
    const VREFBUF_CSR_VRS: u32 = 0b111 << 4;
    let mut vrefbuf_csr = read_volatile(VREFBUF_CSR_ADDR);
    vrefbuf_csr &= !(VREFBUF_CSR_VRS);
    write_volatile(VREFBUF_CSR_ADDR, vrefbuf_csr);

    // while (read_volatile(VREFBUF_CSR_ADDR) & (1 << 3)) == 0 {}
    const VREFBUF_CSR_HIZ_ENVR: u32 = 1 << 0;
    let mut vrefbuf_csr = read_volatile(VREFBUF_CSR_ADDR);
    vrefbuf_csr &= !(0b11 << 0);
    vrefbuf_csr |= VREFBUF_CSR_HIZ_ENVR;
    write_volatile(VREFBUF_CSR_ADDR, vrefbuf_csr);
}


pub unsafe fn rcc_enable_adc_clock() {
    const RCC_AHB1ENR_ADC12EN: u32 = 1 << 5;
    let rcc_ahb1enr = read_volatile(RCC_AHB1ENR_ADDR);
    write_volatile(RCC_AHB1ENR_ADDR, rcc_ahb1enr | RCC_AHB1ENR_ADC12EN);

    let mut rcc_d3ccipr = read_volatile(RCC_D3CCIPR_ADDR);
    rcc_d3ccipr &= !(0b11 << 16);
    rcc_d3ccipr |= 1 << 17;
    write_volatile(RCC_D3CCIPR_ADDR, rcc_d3ccipr);

    let mut adc_ccr = read_volatile(ADC_CCR);
    adc_ccr &= !(0b11 << 16);
    adc_ccr |= 1 << 17;
    write_volatile(ADC_CCR, adc_ccr);
}

pub unsafe fn rcc_read_d3ccipr() -> u32 {
    read_volatile(RCC_D3CCIPR_ADDR)
}

pub unsafe fn adc_read_ccr() -> u32 {
    read_volatile(ADC_CCR)
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
    delay(1_000_000);
    while (read_volatile(ADC1_ISR_ADDR) & (1 << 12) == 0) {}
}

pub unsafe fn adc_read_calfact() -> u32 {
    read_volatile(ADC1_CALFACT)
}

pub unsafe fn adc_set_diff() {
    const ADC1_DIFSEL_16: u32 = 1 << 16;
    let adc1_difsel = read_volatile(ADC1_DIFSEL);
    write_volatile(ADC1_DIFSEL, adc1_difsel & !(ADC1_DIFSEL_16));
}

pub unsafe fn adc_read_diff() -> u32 {
    read_volatile(ADC1_DIFSEL)
}

pub unsafe fn adc_calibrate() {
    const ADC1_CR_ADDIS: u32 = 1 << 1;
    let adc1_cr = read_volatile(ADC1_CR_ADDR);
    write_volatile(ADC1_CR_ADDR, adc1_cr | ADC1_CR_ADDIS);

    while (read_volatile(ADC1_CR_ADDR) & (1 << 1)) != 0 {}
    while (read_volatile(ADC1_CR_ADDR) & (1 << 0)) != 0 {}

    const ADC1_CR_ADCALDIF: u32 = 1 << 30;
    let adc1_cr = read_volatile(ADC1_CR_ADDR);
    write_volatile(ADC1_CR_ADDR, adc1_cr & !(ADC1_CR_ADCALDIF));

    const ADC1_CR_ADCALLIN: u32 = 1 << 16;
    let adc1_cr = read_volatile(ADC1_CR_ADDR);
    write_volatile(ADC1_CR_ADDR, adc1_cr & !(ADC1_CR_ADCALLIN));
    
    const ADC1_CR_ADCAL: u32 = 1 << 31;
    let adc1_cr = read_volatile(ADC1_CR_ADDR);
    write_volatile(ADC1_CR_ADDR, adc1_cr | ADC1_CR_ADCAL);

    // Wait until calibration is finished (ADCAL bit goes to 0)
    while (read_volatile(ADC1_CR_ADDR) & (1 << 31)) != 0 {}
}

pub unsafe fn adc_enable() {
    // Enable the ADC with ADVREGEN still enabled
    const ADC1_CR_ADEN: u32 = 1 << 0;
    let adc1_cr: u32 = read_volatile(ADC1_CR_ADDR);

    write_volatile(ADC1_CR_ADDR,  adc1_cr | ADC1_CR_ADEN); // ADEN = 1
    // Wait for ADC to be ready (ADRDY flag in ISR)
    while ((read_volatile(ADC1_ISR_ADDR) & 1) == 0) {}
    // Clear the ADRDY flag by writing 1 to it
    let adc1_isr = read_volatile(ADC1_ISR_ADDR);
    write_volatile(ADC1_ISR_ADDR, adc1_isr | 1);
}

pub unsafe fn adc_configure_single_conversion(channel: u32) {
    let ADC1_SQR1_CHANNEL: u32 = channel << 6;
    let adc1_sqr1 = read_volatile(ADC1_SQR1_ADDR);
    write_volatile(ADC1_SQR1_ADDR, adc1_sqr1 | ADC1_SQR1_CHANNEL);
}

pub unsafe fn adc_read_sqr() -> u32 {
    read_volatile(ADC1_SQR1_ADDR)
}

pub unsafe fn adc_preselect_channel(channel: u32) {
    const ADC1_PCSEL_16: u32 = 1 << 16;
    let adc1_pcsel = read_volatile(ADC1_PCSEL);
    write_volatile(ADC1_PCSEL, adc1_pcsel | ADC1_PCSEL_16);
}

pub unsafe fn adc_read_pcsel() -> u32 {
    read_volatile(ADC1_PCSEL)
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
    // let adc1_isr = read_volatile(ADC1_ISR_ADDR);
    // Clear the EOC flag by writing 1 to it
    // write_volatile(ADC1_ISR_ADDR, adc1_isr | (1 << 2));
    result
}