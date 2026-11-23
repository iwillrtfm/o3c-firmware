//! Fatal handler: panics, CPU exceptions and unexpected interrupts.
//! All LEDs on, then stop.

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    hw::interrupts_off();
    show()
}

/// From rust_trap: an exception or an interrupt nobody handles.
pub fn fatal_trap(_mcause: u32) -> ! {
    hw::interrupts_off();
    show()
}

fn show() -> ! {
    crate::leds::stop();
    // All LEDs full on: rows low, columns high.
    hw::gpio::clear_mask(hw::gpio::B, (1 << 2) | (1 << 10) | (1 << 11));
    hw::gpio::set_mask(hw::gpio::C, 1 | (1 << 5) | (1 << 6));
    loop {}
}
