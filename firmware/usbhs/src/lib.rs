//! Minimal polled USB HID device driver for the CH32V30x USBHS controller, run at full speed.
//!
//! Follows WCH's reference driver (EVT/EXAM/USB/USBHS/DEVICE/CompatibilityHID,
//! ch32v30x_usbhs_device.c): the same register sequence, the same manual data toggles. It
//! uses no interrupts: call `poll` often (every pass of the main loop). The controller
//! NAKs the host while a flag is pending (INT_BUSY), so a slow pass only delays the host.
//!
//! Endpoints 1..=3, 64 bytes each, interrupt type. EP0 is 64 bytes.
#![no_std]

use core::ptr::{read_volatile, write_volatile};

const BASE: usize = 0x4002_3400;

// Registers (USBHSD_TypeDef offsets).
const CONTROL: usize = 0x00;
const HOST_CTRL: usize = 0x01;
const INT_EN: usize = 0x02;
const DEV_AD: usize = 0x03;
const INT_FG: usize = 0x0A;
const INT_ST: usize = 0x0B;
const RX_LEN: usize = 0x0C;
const ENDP_CONFIG: usize = 0x10;
const ENDP_TYPE: usize = 0x14;
const BUF_MODE: usize = 0x18;
const UEP0_DMA: usize = 0x1C;
const fn uep_rx_dma(n: usize) -> usize {
    0x1C + 4 * n
}
const fn uep_tx_dma(n: usize) -> usize {
    0x58 + 4 * n
}
const fn uep_max_len(n: usize) -> usize {
    0x98 + 4 * n
}
const fn uep_tx_len(n: usize) -> usize {
    0xD8 + 4 * n
}
const fn uep_tx_ctrl(n: usize) -> usize {
    0xDA + 4 * n
}
const fn uep_rx_ctrl(n: usize) -> usize {
    0xDB + 4 * n
}

const UC_DEV_PU_EN: u8 = 0x10;
const UC_INT_BUSY: u8 = 0x08;
const UC_RESET_SIE: u8 = 0x04;
const UC_CLR_ALL: u8 = 0x02;
const UC_DMA_EN: u8 = 0x01;
const UH_PHY_SUSPENDM: u8 = 0x10;

const UIE_SETUP_ACT: u8 = 0x20;
const UIE_SUSPEND: u8 = 0x04;
const UIE_TRANSFER: u8 = 0x02;
const UIE_BUS_RST: u8 = 0x01;

const UIF_SETUP_ACT: u8 = 0x20;
const UIF_SUSPEND: u8 = 0x04;
const UIF_TRANSFER: u8 = 0x02;
const UIF_BUS_RST: u8 = 0x01;

const UIS_TOG_OK: u8 = 0x40;
const UIS_TOKEN_MASK: u8 = 0x30;
const UIS_TOKEN_OUT: u8 = 0x00;
const UIS_TOKEN_IN: u8 = 0x20;
const UIS_ENDP_MASK: u8 = 0x0F;

const T_TOG_DATA1: u8 = 0x08;
const T_RES_MASK: u8 = 0x03;
const T_RES_ACK: u8 = 0x00;
const T_RES_NAK: u8 = 0x02;
const T_RES_STALL: u8 = 0x03;
const R_TOG_DATA1: u8 = 0x08;
const R_RES_MASK: u8 = 0x03;
const R_RES_ACK: u8 = 0x00;
const R_RES_NAK: u8 = 0x02;
const R_RES_STALL: u8 = 0x03;

pub const EP_SIZE: usize = 64;
const MAX_EP: usize = 4;

#[inline]
fn r8(o: usize) -> u8 {
    unsafe { read_volatile((BASE + o) as *const u8) }
}
#[inline]
fn w8(o: usize, v: u8) {
    unsafe { write_volatile((BASE + o) as *mut u8, v) }
}
#[inline]
fn r16(o: usize) -> u16 {
    unsafe { read_volatile((BASE + o) as *const u16) }
}
#[inline]
fn w16(o: usize, v: u16) {
    unsafe { write_volatile((BASE + o) as *mut u16, v) }
}
#[inline]
fn w32(o: usize, v: u32) {
    unsafe { write_volatile((BASE + o) as *mut u32, v) }
}

#[repr(C, align(4))]
struct Buf([u8; EP_SIZE]);

static mut EP0_BUF: Buf = Buf([0; EP_SIZE]);
static mut IN_BUF: [Buf; MAX_EP] = [const { Buf([0; EP_SIZE]) }; MAX_EP];
static mut OUT_BUF: [Buf; MAX_EP] = [const { Buf([0; EP_SIZE]) }; MAX_EP];
static ZEROS: [u8; EP_SIZE] = [0; EP_SIZE];

fn ep0() -> &'static mut [u8; EP_SIZE] {
    unsafe { &mut (*(&raw mut EP0_BUF)).0 }
}
fn in_buf(e: usize) -> &'static mut [u8; EP_SIZE] {
    unsafe { &mut (*(&raw mut IN_BUF))[e].0 }
}
fn out_buf(e: usize) -> &'static [u8; EP_SIZE] {
    unsafe { &(*(&raw const OUT_BUF))[e].0 }
}

/// One HID interface: its report descriptor and where its HID descriptor sits in the
/// configuration descriptor.
pub struct Interface {
    pub report: &'static [u8],
    pub hid_desc_offset: usize,
}

pub struct Descriptors {
    pub device: &'static [u8],
    pub config: &'static [u8],
    pub interfaces: &'static [Interface],
    /// String descriptors 0 (languages), 1, 2, ... already encoded.
    pub strings: &'static [&'static [u8]],
    /// Endpoint numbers (1..=3) used IN and OUT.
    pub in_eps: &'static [u8],
    pub out_eps: &'static [u8],
}

pub struct Usb {
    d: &'static Descriptors,
    req_type: u8,
    req: u8,
    pending: &'static [u8],
    left: u16,
    addr: u8,
    config: u8,
    idle: u8,
    protocol: u8,
    in_busy: [bool; MAX_EP],
    out_ready: [u8; MAX_EP], // 0 = none, else len + 1
    pub resets: u32,
    pub setups: u32,
}

impl Usb {
    /// Clocks: enables the USBHS peripheral and its PHY PLL from HSE (8 MHz / 2 = 4 MHz
    /// reference), as WCH's USBHS_RCC_Init. Then resets the SIE, sets full speed, and
    /// connects (pull-up on).
    pub fn new(d: &'static Descriptors) -> Self {
        unsafe {
            let cfgr2 = 0x4002_102C as *mut u32;
            let mut v = read_volatile(cfgr2) & !(0xFF << 24);
            v |= 1 << 24; // USBHS prescaler: /2
            v |= 1 << 28; // PHY PLL reference: 4 MHz
            v |= 1 << 30; // PHY PLL alive
            v |= 1 << 31; // 48 MHz clock from the USB PHY
            write_volatile(cfgr2, v); // bit 27 = 0: PLL source HSE
            let ahb = 0x4002_1014 as *mut u32;
            write_volatile(ahb, read_volatile(ahb) | (1 << 11));
        }
        w8(CONTROL, UC_CLR_ALL | UC_RESET_SIE);
        spin(2000);
        w8(CONTROL, r8(CONTROL) & !UC_RESET_SIE);
        w8(HOST_CTRL, UH_PHY_SUSPENDM);
        w8(CONTROL, UC_DMA_EN | UC_INT_BUSY); // speed bits 00 = full speed
        w8(INT_EN, UIE_SETUP_ACT | UIE_TRANSFER | UIE_BUS_RST | UIE_SUSPEND);
        let mut u = Usb {
            d,
            req_type: 0,
            req: 0,
            pending: &[],
            left: 0,
            addr: 0,
            config: 0,
            idle: 0,
            protocol: 1,
            in_busy: [false; MAX_EP],
            out_ready: [0; MAX_EP],
            resets: 0,
            setups: 0,
        };
        u.endpoints_init();
        w8(CONTROL, r8(CONTROL) | UC_DEV_PU_EN);
        u
    }

    /// Pull-up off: the host sees a disconnect. Call before a reset into another image.
    pub fn disconnect(&mut self) {
        w8(CONTROL, r8(CONTROL) & !UC_DEV_PU_EN);
    }

    pub fn configured(&self) -> bool {
        self.config != 0
    }

    fn endpoints_init(&mut self) {
        let mut cfg: u32 = 0x0001_0001; // EP0 T and R
        for &e in self.d.in_eps {
            cfg |= 1 << e;
        }
        for &e in self.d.out_eps {
            cfg |= 1 << (16 + e);
        }
        w32(ENDP_CONFIG, cfg);
        w32(ENDP_TYPE, 0);
        w32(BUF_MODE, 0);
        w16(uep_max_len(0), EP_SIZE as u16);
        w32(UEP0_DMA, (&raw const EP0_BUF) as u32);
        w8(uep_tx_ctrl(0), T_RES_NAK);
        w8(uep_rx_ctrl(0), R_RES_ACK);
        for &e in self.d.in_eps {
            let e = e as usize;
            w16(uep_max_len(e), EP_SIZE as u16);
            w32(uep_tx_dma(e), unsafe { (&raw const IN_BUF[e]) as u32 });
            w8(uep_tx_ctrl(e), T_RES_NAK);
        }
        for &e in self.d.out_eps {
            let e = e as usize;
            w16(uep_max_len(e), EP_SIZE as u16);
            w32(uep_rx_dma(e), unsafe { (&raw const OUT_BUF[e]) as u32 });
            w8(uep_rx_ctrl(e), R_RES_ACK);
        }
        self.in_busy = [false; MAX_EP];
        self.out_ready = [0; MAX_EP];
    }

    /// Queue `data` (<= 64 bytes) on IN endpoint `ep`. False if the previous one is not
    /// collected by the host yet.
    pub fn write(&mut self, ep: u8, data: &[u8]) -> bool {
        let e = ep as usize;
        if self.in_busy[e] || self.config == 0 {
            return false;
        }
        let n = data.len().min(EP_SIZE);
        in_buf(e)[..n].copy_from_slice(&data[..n]);
        w16(uep_tx_len(e), n as u16);
        w8(uep_tx_ctrl(e), (r8(uep_tx_ctrl(e)) & !T_RES_MASK) | T_RES_ACK);
        self.in_busy[e] = true;
        true
    }

    pub fn in_busy(&self, ep: u8) -> bool {
        self.in_busy[ep as usize]
    }

    /// Take a received OUT packet; the endpoint ACKs again afterwards.
    pub fn read(&mut self, ep: u8, out: &mut [u8; EP_SIZE]) -> Option<usize> {
        let e = ep as usize;
        if self.out_ready[e] == 0 {
            return None;
        }
        let n = (self.out_ready[e] - 1) as usize;
        out.fill(0);
        out[..n].copy_from_slice(&out_buf(e)[..n]);
        self.out_ready[e] = 0;
        w8(uep_rx_ctrl(e), (r8(uep_rx_ctrl(e)) & !R_RES_MASK) | R_RES_ACK);
        Some(n)
    }

    pub fn poll(&mut self) {
        let fg = r8(INT_FG);
        if fg & UIF_TRANSFER != 0 {
            let st = r8(INT_ST);
            let ep = (st & UIS_ENDP_MASK) as usize;
            match st & UIS_TOKEN_MASK {
                UIS_TOKEN_IN => {
                    if ep == 0 {
                        self.ep0_in();
                    } else if ep < MAX_EP {
                        w8(uep_tx_ctrl(ep), (r8(uep_tx_ctrl(ep)) & !T_RES_MASK) | T_RES_NAK);
                        w8(uep_tx_ctrl(ep), r8(uep_tx_ctrl(ep)) ^ T_TOG_DATA1);
                        self.in_busy[ep] = false;
                    }
                }
                UIS_TOKEN_OUT => {
                    if ep == 0 {
                        // Status stage of an IN request, or SET_REPORT data (ignored).
                        if st & UIS_TOG_OK != 0 && self.left > 0 {
                            let n = r16(RX_LEN).min(self.left);
                            self.left -= n;
                            w8(uep_rx_ctrl(0), r8(uep_rx_ctrl(0)) ^ R_TOG_DATA1);
                            if self.left == 0 {
                                w16(uep_tx_len(0), 0);
                                w8(uep_tx_ctrl(0), T_TOG_DATA1 | T_RES_ACK);
                            }
                        }
                    } else if ep < MAX_EP && st & UIS_TOG_OK != 0 {
                        w8(uep_rx_ctrl(ep), r8(uep_rx_ctrl(ep)) ^ R_TOG_DATA1);
                        self.out_ready[ep] = r16(RX_LEN).min(EP_SIZE as u16) as u8 + 1;
                        w8(uep_rx_ctrl(ep), (r8(uep_rx_ctrl(ep)) & !R_RES_MASK) | R_RES_NAK);
                    }
                }
                _ => {} // SOF, SETUP: handled by SETUP_ACT
            }
            w8(INT_FG, UIF_TRANSFER);
        } else if fg & UIF_SETUP_ACT != 0 {
            self.setups += 1;
            self.setup();
            w8(INT_FG, UIF_SETUP_ACT);
        } else if fg & UIF_BUS_RST != 0 {
            self.resets += 1;
            self.addr = 0;
            self.config = 0;
            w8(DEV_AD, 0);
            self.endpoints_init();
            w8(INT_FG, UIF_BUS_RST);
        } else if fg & UIF_SUSPEND != 0 {
            w8(INT_FG, UIF_SUSPEND);
        } else if fg != 0 {
            w8(INT_FG, fg);
        }
    }

    fn ep0_in(&mut self) {
        if self.left == 0 {
            w8(uep_rx_ctrl(0), R_TOG_DATA1 | R_RES_ACK);
        }
        if self.req_type & 0x60 == 0 {
            match self.req {
                0x06 => {
                    // GET_DESCRIPTOR: next chunk
                    let n = (self.left as usize).min(EP_SIZE).min(self.pending.len());
                    ep0()[..n].copy_from_slice(&self.pending[..n]);
                    self.pending = &self.pending[n..];
                    self.left -= n as u16;
                    w16(uep_tx_len(0), n as u16);
                    w8(uep_tx_ctrl(0), r8(uep_tx_ctrl(0)) ^ T_TOG_DATA1);
                }
                0x05 => w8(DEV_AD, self.addr), // SET_ADDRESS: after the status stage
                _ => {}
            }
        } else if self.req == 0x01 && self.left > 0 {
            // GET_REPORT continuation: zeros
            let n = (self.left as usize).min(EP_SIZE);
            ep0()[..n].fill(0);
            self.left -= n as u16;
            w16(uep_tx_len(0), n as u16);
            w8(uep_tx_ctrl(0), r8(uep_tx_ctrl(0)) ^ T_TOG_DATA1);
        }
    }

    fn setup(&mut self) {
        w8(uep_tx_ctrl(0), T_TOG_DATA1 | T_RES_NAK);
        w8(uep_rx_ctrl(0), R_TOG_DATA1 | R_RES_NAK);
        let p = *ep0();
        let req_type = p[0];
        let req = p[1];
        let value = u16::from_le_bytes([p[2], p[3]]);
        let index = u16::from_le_bytes([p[4], p[5]]);
        let mut left = u16::from_le_bytes([p[6], p[7]]);
        self.req_type = req_type;
        self.req = req;
        self.pending = &[];
        let mut reply: &'static [u8] = &[];
        let mut small = [0u8; 2];
        let mut small_len = 0usize;
        let mut err = false;

        if req_type & 0x60 == 0x20 {
            // HID class
            match req {
                0x0A => self.idle = (value >> 8) as u8, // SET_IDLE
                0x0B => self.protocol = value as u8,     // SET_PROTOCOL
                0x09 => {}                               // SET_REPORT: data follows, ignored
                0x02 => {
                    small[0] = self.idle;
                    small_len = 1;
                }
                0x03 => {
                    small[0] = self.protocol;
                    small_len = 1;
                }
                0x01 => reply = &ZEROS, // GET_REPORT: zeros
                _ => err = true,
            }
        } else if req_type & 0x60 == 0 {
            match req {
                0x06 => {
                    let kind = (value >> 8) as u8;
                    let i = (value & 0xFF) as usize;
                    match kind {
                        0x01 => reply = self.d.device,
                        0x02 => reply = self.d.config,
                        0x03 => match self.d.strings.get(i) {
                            Some(s) => reply = s,
                            None => err = true,
                        },
                        0x22 => match self.d.interfaces.get(index as usize) {
                            Some(f) => reply = f.report,
                            None => err = true,
                        },
                        0x21 => match self.d.interfaces.get(index as usize) {
                            Some(f) => reply = &self.d.config[f.hid_desc_offset..f.hid_desc_offset + 9],
                            None => err = true,
                        },
                        _ => err = true, // qualifier, BOS, other speed: full speed only
                    }
                }
                0x05 => self.addr = (value & 0x7F) as u8,
                0x08 => {
                    small[0] = self.config;
                    small_len = 1;
                }
                0x09 => self.config = value as u8,
                0x01 => {
                    // CLEAR_FEATURE (endpoint halt): data toggle back to DATA0, response ACK/NAK
                    if req_type & 0x1F == 2 {
                        let e = (index & 0x0F) as usize;
                        if e > 0 && e < MAX_EP {
                            if index & 0x80 != 0 {
                                w8(uep_tx_ctrl(e), T_RES_NAK);
                                self.in_busy[e] = false;
                            } else {
                                w8(uep_rx_ctrl(e), R_RES_ACK);
                            }
                        }
                    }
                }
                0x03 => {
                    // SET_FEATURE: endpoint halt only
                    if req_type & 0x1F == 2 && value == 0 {
                        let e = (index & 0x0F) as usize;
                        if e > 0 && e < MAX_EP {
                            if index & 0x80 != 0 {
                                w8(uep_tx_ctrl(e), (r8(uep_tx_ctrl(e)) & !T_RES_MASK) | T_RES_STALL);
                            } else {
                                w8(uep_rx_ctrl(e), (r8(uep_rx_ctrl(e)) & !R_RES_MASK) | R_RES_STALL);
                            }
                        }
                    } else if req_type & 0x1F != 0 {
                        err = true;
                    }
                }
                0x0A => small_len = 1, // GET_INTERFACE: 0
                0x0B => {}             // SET_INTERFACE
                0x00 => small_len = 2, // GET_STATUS: 0
                _ => err = true,
            }
        } else {
            err = true;
        }

        if err {
            w8(uep_tx_ctrl(0), T_TOG_DATA1 | T_RES_STALL);
            w8(uep_rx_ctrl(0), R_TOG_DATA1 | R_RES_STALL);
            self.left = 0;
            return;
        }

        if req_type & 0x80 != 0 {
            // IN data stage
            let n;
            if small_len > 0 {
                left = left.min(small_len as u16);
                ep0()[..small_len].copy_from_slice(&small[..small_len]);
                n = left as usize;
                left = 0;
            } else {
                left = left.min(reply.len() as u16);
                n = (left as usize).min(EP_SIZE);
                ep0()[..n].copy_from_slice(&reply[..n]);
                self.pending = &reply[n..];
                left -= n as u16;
            }
            self.left = left;
            w16(uep_tx_len(0), n as u16);
            w8(uep_tx_ctrl(0), T_TOG_DATA1 | T_RES_ACK);
        } else {
            self.left = left;
            if left == 0 {
                w16(uep_tx_len(0), 0);
                w8(uep_tx_ctrl(0), T_TOG_DATA1 | T_RES_ACK);
            } else {
                w8(uep_rx_ctrl(0), R_TOG_DATA1 | R_RES_ACK);
            }
        }
    }
}

fn spin(n: u32) {
    for _ in 0..n {
        unsafe { core::arch::asm!("nop") };
    }
}

/// Builds a string descriptor at compile time from ASCII.
#[macro_export]
macro_rules! string_descriptor {
    ($s:expr) => {{
        const S: &[u8] = $s.as_bytes();
        const N: usize = 2 + 2 * S.len();
        const D: [u8; N] = {
            let mut d = [0u8; N];
            d[0] = N as u8;
            d[1] = 3;
            let mut i = 0;
            while i < S.len() {
                d[2 + 2 * i] = S[i];
                i += 1;
            }
            d
        };
        &D
    }};
}
