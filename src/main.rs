#![deny(warnings)]
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use cortex_m_rt::entry;

use stm32h7xx_hal::rcc::rec::{AdcClkSel, UsbClkSel, Spi123ClkSel};
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
use stm32h7xx_hal::{spi, adc, delay::Delay, prelude::*, stm32};
use usb_device::prelude::*;

use embedded_hal_compat::{ForwardCompat};
use embedded_hal_bus::spi::ExclusiveDevice;
use mcp4x::{Channel, Mcp4x};

use core::fmt::Write;
use cortex_m::asm::nop;
use heapless::String;

use libm::{powf, logf, roundf};

use panic_halt as _;

static mut EP_MEMORY: MaybeUninit<[u32; 1024]> = MaybeUninit::uninit();

const T_0: f32 = 298.15;
const BETA: f32 = 4050.0;
const V_S: f32 = 3.3;
const R_REF: f32 = 10000.0;
const R_0: f32 = 10000.0;
// POT_VALUE: 0 is for MAX heat, 255 is MIN heat
const POT_VALUE: u8 = 255;

use embedded_hal::delay::DelayNs;

// Mathematical Model for Control
const T_AMBIENT: f32 = 26.5;
const R_TH: f32 = 170f32;
const R_HEAT: f32 = 560f32;
const R_POT_MAX: f32 = 10_000f32;
const POT_VALUE_MAX: f32 = 255f32;

// Kalman Filter
const _A: f32 = 0f32;
const _H: f32 = 1f32;
const Q: f32 = 0.0028;
const R: f32 = 0.395;
const _P_P: f32 = Q;
const K: f32 = Q / (Q + R);
const _P_E: f32 = _P_P - (K * _P_P);

// PI Controller
const KP: f32 = 1_750.0;
const KI: f32 = 5.0;
const DT: f32 = 0.5;     // Control loop time in seconds (since delay is 500 ms)
const POT_MIN: u8 = 0;   // Fully ON (max heat)
const POT_MAX: u8 = 255; // Fully OFF (min heat)
const TARGET_TEMP: f32 = 26.6;

struct NoopDelay;
impl DelayNs for NoopDelay {
  fn delay_ns(&mut self, _ns: u32) {
    nop();
  }
}

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

    ccdr.peripheral.kernel_spi123_clk_mux(Spi123ClkSel::Per);

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    // IO
    let (pin_dm, pin_dp) = {
        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        (gpioa.pa11.into_alternate(), gpioa.pa12.into_alternate())
    };

    // GPIO
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // SPI
    let sck = gpiob.pb13.into_alternate();
    let miso = gpiob.pb14.into_alternate();
    let mosi = gpiob.pb15.into_alternate();
    let cs = gpiob.pb12.into_push_pull_output();

    let spi = dp.SPI2.spi(
      (sck, miso, mosi),
      spi::MODE_0,
      3.MHz(),
      ccdr.peripheral.SPI2,
      &ccdr.clocks,
    );

    let spi_v1= spi.forward();
    let cs_v1 = cs.forward();


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

    let dev = ExclusiveDevice::new(spi_v1, cs_v1, NoopDelay).unwrap();

    let mut mcp41x = Mcp4x::new_mcp41x(dev);

    mcp41x.set_position(Channel::Ch0, POT_VALUE).unwrap();

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

    // Kalman Filter
    let mut is_first_reading = true;
    let mut z: f32;
    let mut x_p: f32;
    let mut x_e: f32;
    let mut c: f32 = T_AMBIENT + (R_TH * (powf(V_S * (R_HEAT / (R_HEAT + R_POT_MAX * (POT_VALUE as f32 / POT_VALUE_MAX))), 2f32) / R_HEAT));

    // PI Controller state
    // let mut old_pot_value = POT_VALUE as f32;
    let mut integral: f32 = 0.0;
    let mut pot_output: u8;

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let data: u32 = adc1.read(&mut channel).unwrap();

        let voltage_different = data as f32 * (2.5 / adc1.slope() as f32);

        let v_buffer = (22.0 / 47.0) * voltage_different;

        let to_ln = ((V_S - 2f32 * v_buffer) * R_REF) / ((V_S + 2f32 * v_buffer) * R_0);
        let temp_k = powf((1f32 / T_0) + ((1f32 / BETA) * logf(to_ln)), -1f32);

        z = temp_k - 273.15;

        if is_first_reading {
            x_e = z;
            is_first_reading = false;
        } else {
            x_p = c;
            x_e = x_p + (K * (z - x_p));
        }

        // Round for sensor precision (to 2 decimal points)
        x_e = roundf(x_e * 100f32) / 100f32;

        // let mut m: String<32> = String::new();
        // write!(m, "Temperature: {:.2} C\r\n", x_e).unwrap();

        // === PI CONTROLLER ===
        let error = TARGET_TEMP - x_e;
        integral += error * DT;

        let control_signal = KP * error + KI * integral;

        // Map control output to potentiometer range
        // Lower pot value = higher heat
        let mut new_pot_value = POT_VALUE as f32 - control_signal;
        if new_pot_value < POT_MIN as f32 { new_pot_value = POT_MIN as f32; }
        if new_pot_value > POT_MAX as f32 { new_pot_value = POT_MAX as f32; }

        // if new_pot_value > POT_MAX as f32 || new_pot_value < POT_MIN as f32 {
        //     pot_output = old_pot_value as u8;
        // } else {
        //     pot_output = new_pot_value as u8;
        //     old_pot_value = new_pot_value;
        // }

        pot_output = new_pot_value as u8;
        c = T_AMBIENT + (R_TH * (powf(V_S * (R_HEAT / (R_HEAT + R_POT_MAX * (pot_output as f32 / POT_VALUE_MAX))), 2f32) / R_HEAT));

        // Apply new potentiometer position
        mcp41x.set_position(Channel::Ch0, pot_output).unwrap();

        // === USB SERIAL OUTPUT ===
        let mut m: String<64> = String::new();
        write!(
            m,
            "Temp: {:.2} C | Err: {:.2} | Pot: {:.2} | Int: {:.2}\r\n",
            x_e, error, pot_output, integral
        ).unwrap();

        match serial.write(m.as_bytes()) {
            _ => {}
        }

        led.toggle();

        delay.delay_ms(500_u16);

    }
}