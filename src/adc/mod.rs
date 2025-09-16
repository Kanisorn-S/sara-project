use core::ptr::{read_volatile, write_volatile};
use crate::gpio::{gpio_set_pin_mode, rcc_enable_gpio_clock, GpioPort, PinMode};
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


pub struct ADC1 {
    adc_pin_port: GpioPort,
    adc_pin: u8,
    adc_channel: u32,
}

impl ADC1 {
    pub fn new() -> ADC1 {
        Self {
            adc_pin_port: GpioPort::A,
            adc_pin: 0,
            adc_channel: 16,
        }
    }
    pub unsafe fn init(&self) {
        rcc_enable_gpio_clock(self.adc_pin_port);
        self.rcc_enable_adc_clock();
        gpio_set_pin_mode(self.adc_pin_port, self.adc_pin, PinMode::ANALOG);


        self.configure_pwr();
        self.adc_power_up();

        self.configure_single_conversion(self.adc_channel);
        self.preselect_channel(self.adc_channel);
        self.set_diff();

        self.calibrate();

        self.enable();

    }

    pub unsafe fn start_conversion(&self) {
        let cr_val = read_volatile(ADC1_CR_ADDR);
        write_volatile(ADC1_CR_ADDR, cr_val | (1 << 2)); // ADSTART = 1
    }

    pub unsafe fn wait_for_conversion(&self) {
        // Wait for End of Conversion (EOC flag in ISR)
        while (read_volatile(ADC1_ISR_ADDR) & (1 << 2)) == 0 {}
    }

    pub unsafe fn read_value(&self) -> u32 {
        let result = read_volatile(ADC1_DR_ADDR);
        result
    }

}

impl ADC1 {

    unsafe fn configure_pwr(&self) {
        // Set Voltage Reference to 2.5V
        const VREFBUF_CSR_VRS: u32 = 0b111 << 4;
        let mut vrefbuf_csr = read_volatile(VREFBUF_CSR_ADDR);
        vrefbuf_csr &= !(VREFBUF_CSR_VRS);
        write_volatile(VREFBUF_CSR_ADDR, vrefbuf_csr);

        // Connect VREF+ to the Voltage Reference Buffer Output
        const VREFBUF_CSR_HIZ_ENVR: u32 = 1 << 0;
        let mut vrefbuf_csr = read_volatile(VREFBUF_CSR_ADDR);
        vrefbuf_csr &= !(0b11 << 0);
        vrefbuf_csr |= VREFBUF_CSR_HIZ_ENVR;
        write_volatile(VREFBUF_CSR_ADDR, vrefbuf_csr);
    }


    unsafe fn rcc_enable_adc_clock(&self) {
        // Enable ADC12 Peripheral Clock
        const RCC_AHB1ENR_ADC12EN: u32 = 1 << 5;
        let rcc_ahb1enr = read_volatile(RCC_AHB1ENR_ADDR);
        write_volatile(RCC_AHB1ENR_ADDR, rcc_ahb1enr | RCC_AHB1ENR_ADC12EN);

        // Select per_ck as Kernel Clock Source (per_ck = HSI by default)
        let mut rcc_d3ccipr = read_volatile(RCC_D3CCIPR_ADDR);
        rcc_d3ccipr &= !(0b11 << 16);
        rcc_d3ccipr |= 1 << 17;
        write_volatile(RCC_D3CCIPR_ADDR, rcc_d3ccipr);


        // Set the ADC Clock Mode to Synchronous adc_sclk / 2
        let mut adc_ccr = read_volatile(ADC_CCR);
        adc_ccr &= !(0b11 << 16);
        adc_ccr |= 1 << 17;
        write_volatile(ADC_CCR, adc_ccr);
    }

    unsafe fn adc_power_up(&self) {
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

    unsafe fn set_diff(&self) {
        // Set ADC1 Channel 16 Input to be Single-Ended
        const ADC1_DIFSEL_16: u32 = 1 << 16;
        let adc1_difsel = read_volatile(ADC1_DIFSEL);
        write_volatile(ADC1_DIFSEL, adc1_difsel & !(ADC1_DIFSEL_16));
    }

    unsafe fn calibrate(&self) {
        // Ensure ADEN is 0 by Setting ADDIS
        const ADC1_CR_ADDIS: u32 = 1 << 1;
        let adc1_cr = read_volatile(ADC1_CR_ADDR);
        write_volatile(ADC1_CR_ADDR, adc1_cr | ADC1_CR_ADDIS);

        // Wait for ADDIS and ADEN to be 0
        while (read_volatile(ADC1_CR_ADDR) & (1 << 1)) != 0 {}
        while (read_volatile(ADC1_CR_ADDR) & (1 << 0)) != 0 {}

        // Set Calibration Mode to Single-Ended
        const ADC1_CR_ADCALDIF: u32 = 1 << 30;
        let adc1_cr = read_volatile(ADC1_CR_ADDR);
        write_volatile(ADC1_CR_ADDR, adc1_cr & !(ADC1_CR_ADCALDIF));

        // Disable Linearity Calibration
        const ADC1_CR_ADCALLIN: u32 = 1 << 16;
        let adc1_cr = read_volatile(ADC1_CR_ADDR);
        write_volatile(ADC1_CR_ADDR, adc1_cr & !(ADC1_CR_ADCALLIN));

        // Start Calibration Process
        const ADC1_CR_ADCAL: u32 = 1 << 31;
        let adc1_cr = read_volatile(ADC1_CR_ADDR);
        write_volatile(ADC1_CR_ADDR, adc1_cr | ADC1_CR_ADCAL);

        // Wait until calibration is finished (ADCAL bit goes to 0)
        while (read_volatile(ADC1_CR_ADDR) & (1 << 31)) != 0 {}
    }

    unsafe fn enable(&self) {
        // Enable the ADC
        const ADC1_CR_ADEN: u32 = 1 << 0;
        let adc1_cr: u32 = read_volatile(ADC1_CR_ADDR);

        write_volatile(ADC1_CR_ADDR,  adc1_cr | ADC1_CR_ADEN); // ADEN = 1
        // Wait for ADC to be ready (ADRDY flag in ISR)
        while ((read_volatile(ADC1_ISR_ADDR) & 1) == 0) {}
        // Clear the ADRDY flag by writing 1 to it
        let adc1_isr = read_volatile(ADC1_ISR_ADDR);
        write_volatile(ADC1_ISR_ADDR, adc1_isr | 1);
    }

    unsafe fn configure_single_conversion(&self, channel: u32) {
        // Set Channel for First Conversion
        let ADC1_SQR1_CHANNEL: u32 = channel << 6;
        let adc1_sqr1 = read_volatile(ADC1_SQR1_ADDR);
        write_volatile(ADC1_SQR1_ADDR, adc1_sqr1 | ADC1_SQR1_CHANNEL);
    }

    unsafe fn preselect_channel(&self, channel: u32) {
        // Preselect Input Channel
        let ADC1_PCSEL_16: u32 = 1 << channel;
        let adc1_pcsel = read_volatile(ADC1_PCSEL);
        write_volatile(ADC1_PCSEL, adc1_pcsel | ADC1_PCSEL_16);
    }

}