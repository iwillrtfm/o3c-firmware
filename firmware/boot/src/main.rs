//! O3C USB bootloader. Lives in the first 32 KiB of flash.
//!
//! At reset it starts the app at APP_START at once, unless the app header is missing, the
//! app asked to stay here (handoff word in RAM), or the knob is held down at power-on.
//! Otherwise it is a vendor HID device that takes ERASE / WRITE / COMMIT / RUN from the
//! `o3c` host tool. WCH's ROM bootloader (BOOT0 pads under the screen) stays the last
//! resort. Plain polling loop, no interrupts.
#![no_std]
#![no_main]

use hw::flash::{PageWriter, WriteError};
use hw::{gpio, rcc};
use o3c_proto::{self as proto, cmd, msg, status};
use usbhs::{Descriptors, Interface, Usb};

const VERSION: u16 = 5;
const EP_IN: u8 = 2;
const EP_OUT: u8 = 3;

/// Vendor-defined, 64-byte input and output reports, no report ID.
const REPORT: &[u8] = &[
    0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x40, 0x09, 0x02,
    0x81, 0x02, 0x95, 0x40, 0x09, 0x03, 0x91, 0x02, 0xC0,
];

const DEVICE: &[u8] = &[
    18, 1, 0x00, 0x02, 0, 0, 0, 64,
    (proto::VID & 0xFF) as u8, (proto::VID >> 8) as u8, (proto::PID & 0xFF) as u8, (proto::PID >> 8) as u8,
    0x00, 0x01, 1, 2, 0, 1,
];

const CONFIG: &[u8] = &[
    9, 2, 41, 0, 1, 1, 0, 0x80, 50, //
    9, 4, 0, 0, 2, 3, 0, 0, 0, //
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, REPORT.len() as u8, 0, //
    7, 5, 0x80 | EP_IN, 3, 64, 0, 1, //
    7, 5, EP_OUT, 3, 64, 0, 1,
];

static DESC: Descriptors = Descriptors {
    device: DEVICE,
    config: CONFIG,
    interfaces: &[Interface { report: REPORT, hid_desc_offset: 18 }],
    strings: &[&[4, 3, 0x09, 0x04], usbhs::string_descriptor!("o3c"), usbhs::string_descriptor!("O3C bootloader")],
    in_eps: &[EP_IN],
    out_eps: &[EP_OUT],
};

fn handoff() -> *mut u32 {
    proto::HANDOFF_ADDR as *mut u32
}

fn header() -> (u32, u32, u32) {
    let p = proto::HEADER_ADDR as *const u32;
    unsafe { (p.read_volatile(), p.add(1).read_volatile(), p.add(2).read_volatile()) }
}

fn app_valid() -> bool {
    let (magic, len, _) = header();
    let first = unsafe { (proto::APP_START as *const u32).read_volatile() };
    magic == proto::HEADER_MAGIC && len > 0 && len <= proto::APP_MAX && first != 0xFFFF_FFFF
}

/// Knob switch (PB14, pull-up, low = pressed) held at power-on.
fn knob_held() -> bool {
    rcc::enable_apb2(rcc::IOPB);
    gpio::pull_up(gpio::B, 14);
    let mut low = 0;
    for _ in 0..4000 {
        if gpio::read(gpio::B) & (1 << 14) == 0 {
            low += 1;
        }
    }
    low > 3900
}

/// In the bootloader all three LEDs show red (matrix row 0, all columns), steady;
/// backlight on.
fn boot_signs() {
    rcc::enable_apb2(rcc::IOPA | rcc::IOPB | rcc::IOPC);
    for pin in [2, 10, 11] {
        gpio::high(gpio::B, pin);
        gpio::mode(gpio::B, pin, gpio::OUT_OD);
    }
    for pin in [0, 5, 6] {
        gpio::high(gpio::C, pin);
        gpio::mode(gpio::C, pin, gpio::OUT_PP);
    }
    gpio::low(gpio::B, 2); // row 0: red
    gpio::high(gpio::A, 3);
    gpio::mode(gpio::A, 3, gpio::OUT_PP);
}

/// Writes the app: erase, pieces in order, then the CRC check and the header.
struct Writer {
    image: PageWriter,
}

fn status_of(r: Result<(), WriteError>) -> u8 {
    match r {
        Ok(()) => status::OK,
        Err(WriteError::Order) => status::ORDER,
        Err(WriteError::Flash) => status::FLASH,
    }
}

impl Writer {
    fn erase(&mut self, len: u32) -> u8 {
        let len = if len == 0 || len > proto::APP_MAX { proto::APP_MAX } else { len };
        // Header sector first, so a half-erased app is never started.
        if !hw::flash::erase_sector(proto::HEADER_ADDR) {
            return status::FLASH;
        }
        let mut a = proto::APP_START;
        while a < proto::APP_START + len {
            if !hw::flash::erase_sector(a) {
                return status::FLASH;
            }
            a += hw::flash::SECTOR;
        }
        self.image.start(proto::APP_START, proto::APP_MAX);
        status::OK
    }

    fn commit(&mut self, len: u32, crc: u32) -> u8 {
        let image = match self.image.finish(len) {
            Ok(image) => image,
            Err(e) => return status_of(Err(e)),
        };
        if proto::crc32(image) != crc {
            return status::CRC;
        }
        let mut hdr = [0xFFu8; proto::PAGE_SIZE as usize];
        proto::put_u32(&mut hdr, 0, proto::HEADER_MAGIC);
        proto::put_u32(&mut hdr, 4, len);
        proto::put_u32(&mut hdr, 8, crc);
        if !hw::flash::program_page(proto::HEADER_ADDR, &hdr) || !app_valid() {
            return status::FLASH;
        }
        status::OK
    }
}

#[no_mangle]
extern "C" fn rust_trap(_mcause: u32) {
    // No interrupts are enabled here; any trap is a fault. Start over.
    hw::system_reset();
}

#[no_mangle]
extern "C" fn rust_main() -> ! {
    // Decide before clocks or USB are touched, so the app starts from reset state.
    let stay = unsafe { handoff().read_volatile() } == proto::HANDOFF_STAY_IN_BOOT;
    unsafe { handoff().write_volatile(0) };
    if !stay && app_valid() && !knob_held() {
        unsafe { core::arch::asm!("jr {0}", in(reg) proto::APP_START, options(noreturn)) };
    }

    hw::rcc::init_144mhz();
    hw::time::init();
    boot_signs();

    let mut usb = Usb::new(&DESC);
    let mut w = Writer { image: PageWriter::new() };

    let mut req = [0u8; 64];
    loop {
        usb.poll();
        if usb.read(EP_OUT, &mut req).is_none() {
            continue;
        }
        let mut out = [0u8; 64];
        let mut reset_after = false;
        match req[0] {
            cmd::INFO => {
                out[0] = msg::INFO;
                out[1] = proto::mode::BOOT;
                out[2] = proto::PROTO_VERSION;
                proto::put_u16(&mut out, 3, VERSION);
                out[5..17].copy_from_slice(&hw::uid());
                proto::put_u32(&mut out, 17, proto::APP_MAX);
                out[21] = app_valid() as u8;
            }
            c => {
                let s = match c {
                    cmd::ERASE => w.erase(proto::get_u32(&req, 1)),
                    cmd::WRITE => {
                        let len = (req[5] as usize).min(32);
                        status_of(w.image.write(proto::get_u32(&req, 1), &req[8..8 + len]))
                    }
                    cmd::COMMIT => w.commit(proto::get_u32(&req, 1), proto::get_u32(&req, 5)),
                    cmd::RUN => {
                        if app_valid() {
                            reset_after = true;
                            status::OK
                        } else {
                            status::NO_APP
                        }
                    }
                    cmd::RESET => {
                        unsafe { handoff().write_volatile(proto::HANDOFF_STAY_IN_BOOT) };
                        reset_after = true;
                        status::OK
                    }
                    _ => status::UNKNOWN,
                };
                out[0] = msg::ACK;
                out[1] = c;
                out[2] = s;
            }
        }
        while !usb.write(EP_IN, &out) {
            usb.poll();
        }
        if reset_after {
            // Let the host collect the reply, then leave.
            let end = hw::time::ms() + 200;
            while usb.in_busy(EP_IN) && hw::time::ms() < end {
                usb.poll();
            }
            usb.disconnect();
            hw::time::delay_ms(50);
            hw::system_reset();
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    hw::system_reset();
}
