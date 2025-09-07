use cortex_m::asm::nop;

pub fn delay(count: u32) {
    for _ in 0..count {
        nop();
    }
}