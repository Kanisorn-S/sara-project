#![deny(warnings)]
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use cortex_m_rt::entry;

use stm32h7xx_hal::rcc::rec::{AdcClkSel, UsbClkSel};
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
use stm32h7xx_hal::{adc, delay::Delay, prelude::*, stm32};
use usb_device::prelude::*;

use core::fmt::Write;
use heapless::String;

use libm::{powf, logf};

use panic_halt as _;


static mut EP_MEMORY: MaybeUninit<[u32; 1024]> = MaybeUninit::uninit();

const T_0: f32 = 298.15;
const BETA: f32 = 4050.0;
const V_S: f32 = 3.3;
const R_REF: f32 = 10000.0;
const R_0: f32 = 10000.0;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    // Power
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // RCC
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.sys_ck(80.MHz()).freeze(vos, &dp.SYSCFG);

    // 48MHz CLOCK
    let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

    // Set adc_ker_ck
    ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::Per);

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    // IO
    let (pin_dm, pin_dp) = {
        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        (gpioa.pa11.into_alternate(), gpioa.pa12.into_alternate())
    };

    // LED
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let mut led = gpioe.pe3.into_push_pull_output();

    // ADC
    let mut adc1 = adc::Adc::adc1(
        dp.ADC1,
        4.MHz(),
        &mut delay,
        ccdr.peripheral.ADC12,
        &ccdr.clocks,
    ).enable();
    adc1.set_resolution(adc::Resolution::SixteenBit);

    // Setup GPIOC
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // Configure pc0 as an analog input
    let mut channel = gpioc.pc0.into_analog();

    let usb = USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        pin_dm,
        pin_dp,
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );

    // Initialise EP_MEMORY to zero
    unsafe {
        let buf: &mut [MaybeUninit<u32>; 1024] =
            &mut *(core::ptr::addr_of_mut!(EP_MEMORY) as *mut _);
        for value in buf.iter_mut() {
            value.as_mut_ptr().write(0);
        }
    }

    // Now we may assume that EP_MEMORY is initialised
    #[allow(static_mut_refs)] // TODO: Fix this
    let usb_bus = UsbBus::new(usb, unsafe { EP_MEMORY.assume_init_mut() });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev =
        UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[usb_device::device::StringDescriptors::default()
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST PORT 1")])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();


    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let data: u32 = adc1.read(&mut channel).unwrap();

        let voltage_different = data as f32 * (2.5 / adc1.slope() as f32);

        let v_buffer = (22.0 / 47.0) * voltage_different;

        let to_ln = ((V_S - 2f32 * v_buffer) * R_REF) / ((V_S + 2f32 * v_buffer) * R_0);
        let temp_k = powf((1f32 / T_0) + ((1f32 / BETA) * logf(to_ln)), -1f32);

        let temp_c = temp_k - 273.15;

        let mut m: String<32> = String::new();
        write!(m, "Temperature: {:.2} C\r\n", temp_c).unwrap();

        match serial.write(m.as_bytes()) {
            _ => {}
        }

        led.toggle();

        delay.delay_ms(500_u16);

    }
}