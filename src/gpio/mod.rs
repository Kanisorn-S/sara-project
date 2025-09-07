use core::ptr::{read_volatile, write_volatile};

#[derive(Clone, Copy)]
pub enum GpioPort {
    A, B, C, D, E, F, G, H, I, J, K
}

#[repr(u32)]
pub enum PinMode {
    INPUT = 0b00,
    OUTPUT = 0b01,
    ALTERNATE = 0b10,
    ANALOG = 0b11,
}

const RCC_AHB4ENR_ADDR: *mut u32 = 0x5802_44E0 as *mut u32;
const GPIO_BASE_ADDRS: [u32; 11] = [
    0x5802_0000, // GPIOA
    0x5802_0400, // GPIOB
    0x5802_0800, // GPIOC
    0x5802_0C00, // GPIOD
    0x5802_1000, // GPIOE
    0x5802_1400, // GPIOF
    0x5802_1800, // GPIOG
    0x5802_1C00, // GPIOH
    0x5802_2000, // GPIOI
    0x5802_2400, // GPIOJ
    0x5802_2800, // GPIOK
];

pub unsafe fn rcc_enable_gpio_clock(port: GpioPort) {
    let clock_enable_bit = 1 << (port as u32);
    let rcc_val = read_volatile(RCC_AHB4ENR_ADDR);
    write_volatile(RCC_AHB4ENR_ADDR, rcc_val | clock_enable_bit);
}

pub unsafe fn gpio_set_pin_mode(port: GpioPort, pin: u8, mode: PinMode) {
    let base_addr = GPIO_BASE_ADDRS[port as usize];
    let moder_addr = base_addr as *mut u32;

    let shift = (pin as u32) * 2;
    let current_moder = read_volatile(moder_addr);

    let cleared_moder = current_moder & !(0b11 << shift);
    let new_moder = cleared_moder | (mode as u32) << shift;

    write_volatile(moder_addr, new_moder);
}

pub unsafe fn gpio_set_pin(port: GpioPort, pin: u8, set_value: bool) {
    let base_addr = GPIO_BASE_ADDRS[port as usize];
    let bsrr_addr = (base_addr + 0x18) as *mut u32;

    let shift = if set_value {pin} else {pin + 16};
    let set_mask = 1 << shift;
    write_volatile(bsrr_addr, set_mask);
}

pub unsafe fn gpio_toggle_pin(port: GpioPort, pin: u8) {
    let base_addr = GPIO_BASE_ADDRS[port as usize];
    let odr_addr = (base_addr + 0x14) as *mut u32;

    let current_odr = read_volatile(odr_addr);
    let toggled_odr = current_odr ^ (1 << pin);
    write_volatile(odr_addr, toggled_odr);
}


