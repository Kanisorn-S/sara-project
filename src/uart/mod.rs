use core::ptr::{read_volatile, write_volatile};
use crate::gpio::{gpio_toggle_pin, GpioPort};
use crate::timer::delay;

pub struct Uart {
  base_addr: u32,
}

impl Uart {
  pub fn new(base_addr: u32) -> Uart {
    Self { base_addr }
  }

  pub unsafe fn init(&self, baudrate: u32) {
    let cr1_addr = (self.base_addr + 0x00) as *mut u32;
    let brr_addr = (self.base_addr + 0x0C) as *mut u32;

    const UART_CLK: u32 = 32_000_000;
    let brr_value = UART_CLK / baudrate;
    write_volatile(brr_addr, brr_value);

    let cr1 = read_volatile(cr1_addr);
    write_volatile(cr1_addr, cr1 | (1 << 3) | (1 << 0));
  }

  pub unsafe fn send_byte(&self, byte: u8) {
    let isr_addr = (self.base_addr + 0x1C) as *const u32;
    let tdr_addr = (self.base_addr + 0x28) as *mut u32;
    
    const LED_PORT: GpioPort = GpioPort::E;
    const LED_PIN: u8 = 3;
    
    gpio_toggle_pin(LED_PORT, LED_PIN);

    delay(400_000);

    while (read_volatile(isr_addr) & (1 << 7)) == 0 {}
    
    delay(400_000);

    gpio_toggle_pin(LED_PORT, LED_PIN);

    write_volatile(tdr_addr, byte as u32);
  }

  pub unsafe fn send_string(&self, s: &str) {
    for byte in s.as_bytes() {
      self.send_byte(*byte);
    }
  }

}

pub unsafe fn rcc_enable_uart4_clock() {
  const RCC_D2CFGR: *mut u32 = 0x5802_441C as *mut u32;
  const RCC_APB1LENR_ADDR: *mut u32 = 0x5802_44E8 as *mut u32;
  let rcc_apb1lenr = read_volatile(RCC_APB1LENR_ADDR);
  write_volatile(RCC_APB1LENR_ADDR, rcc_apb1lenr | (1 << 16));

  let mut rcc_d2cfgr = read_volatile(RCC_D2CFGR);
  rcc_d2cfgr &= !(0b111 << 4);
  rcc_d2cfgr |= 1 << 6;
  write_volatile(RCC_D2CFGR, rcc_d2cfgr);
}

pub unsafe fn rcc_enable_uart1_clock() {
  const RCC_D2CFGR: *mut u32 = 0x5802_441C as *mut u32;
  const RCC_APB2ENR_ADDR: *mut u32 = 0x5802_44F0 as *mut u32;

  let rcc_apb2enr = read_volatile(RCC_APB2ENR_ADDR);
  write_volatile(RCC_APB2ENR_ADDR, rcc_apb2enr | (1 << 4));

  let mut rcc_d2cfgr = read_volatile(RCC_D2CFGR);
  rcc_d2cfgr &= !(0b111 << 8);
  rcc_d2cfgr |= 1 << 10;
  write_volatile(RCC_D2CFGR, rcc_d2cfgr);
}