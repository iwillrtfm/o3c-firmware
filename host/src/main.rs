//! o3c: host tool for the O3C keypad with our firmware.
//!
//!   o3c info                     mode, versions, chip UID
//!   o3c flash <app.bin>          reset into the bootloader, write, verify, start the app
//!   o3c flash-boot <boot.bin>    replace the bootloader (done by the app; about 1 s)
//!   o3c boot | reset             reset into the bootloader / restart the app
//!   o3c monitor                  stream raw sensor values and events
//!   o3c leds <r g b r g b r g b>  set the three LEDs left to right; no values = built-in pattern
//!   o3c text <line> <text> [rgb565 hex]
//!   o3c threshold <key> <press> <release>
//!   o3c blink <led> <r> <g> <b> [count]
//!   o3c pin <port> <pin> <mode> <level>   debug: set a GPIO
//!   o3c run <config.toml>        run a command per key press, knob turn and knob push

mod run;

use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};

use hidapi::{HidApi, HidDevice};
use o3c_proto::{self as proto, cmd, event, msg, status};

type Result<T> = std::result::Result<T, String>;

struct Dev {
    dev: HidDevice,
}

impl Dev {
    fn open(api: &mut HidApi) -> Result<Dev> {
        api.refresh_devices().map_err(|e| e.to_string())?;
        for d in api.device_list() {
            if d.vendor_id() == proto::VID && d.product_id() == proto::PID && d.usage_page() == proto::USAGE_PAGE {
                let dev = d.open_device(api).map_err(|e| format!("open {}: {e}", d.path().to_string_lossy()))?;
                return Ok(Dev { dev });
            }
        }
        Err("no O3C found (VID 1209 PID 0001, usage page FF00)".into())
    }

    fn wait(api: &mut HidApi, mode: Option<u8>, timeout: Duration) -> Result<(Dev, [u8; 64])> {
        let end = Instant::now() + timeout;
        loop {
            if let Ok(d) = Dev::open(api) {
                if let Ok(info) = d.info() {
                    if mode.map_or(true, |m| info[1] == m) {
                        return Ok((d, info));
                    }
                }
            }
            if Instant::now() > end {
                return Err("timed out waiting for the O3C".into());
            }
            sleep(Duration::from_millis(200));
        }
    }

    fn send(&self, report: &[u8; 64]) -> Result<()> {
        let mut buf = [0u8; 65];
        buf[1..].copy_from_slice(report);
        self.dev.write(&buf).map_err(|e| e.to_string())?;
        Ok(())
    }

    fn recv(&self, timeout_ms: i32) -> Result<Option<[u8; 64]>> {
        let mut buf = [0u8; 64];
        let n = self.dev.read_timeout(&mut buf, timeout_ms).map_err(|e| e.to_string())?;
        Ok(if n == 0 { None } else { Some(buf) })
    }

    /// Send and wait for the reply of the given type, skipping events.
    fn request(&self, report: &[u8; 64], reply: u8) -> Result<[u8; 64]> {
        self.send(report)?;
        // ERASE clears the whole app region page by page; that takes several seconds.
        let wait = if report[0] == cmd::ERASE { 60 } else { 5 };
        let end = Instant::now() + Duration::from_secs(wait);
        while Instant::now() < end {
            if let Some(r) = self.recv(500)? {
                if r[0] == reply && (reply != msg::ACK || r[1] == report[0]) {
                    return Ok(r);
                }
            }
        }
        Err(format!("no reply to command {:#04x}", report[0]))
    }

    fn command(&self, report: &[u8; 64]) -> Result<()> {
        let r = self.request(report, msg::ACK)?;
        if r[2] != status::OK {
            return Err(format!("command {:#04x} failed: status {}", report[0], r[2]));
        }
        Ok(())
    }

    fn info(&self) -> Result<[u8; 64]> {
        self.request(&req(cmd::INFO), msg::INFO)
    }
}

fn req(c: u8) -> [u8; 64] {
    let mut r = [0u8; 64];
    r[0] = c;
    r
}

fn print_info(i: &[u8; 64]) {
    let mode = match i[1] {
        proto::mode::BOOT => "bootloader",
        proto::mode::APP => "app",
        _ => "unknown",
    };
    let uid: String = i[5..17].iter().map(|b| format!("{b:02x}")).collect();
    println!(
        "mode {mode}, protocol {}, firmware {}, uid {uid}, app max {} bytes{}",
        i[2],
        proto::get_u16(i, 3),
        proto::get_u32(i, 17),
        if i[1] == proto::mode::BOOT { if i[21] != 0 { ", app valid" } else { ", no valid app" } } else { "" }
    );
}

fn flash(api: &mut HidApi, path: &str) -> Result<()> {
    let image = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if image.is_empty() || image.len() as u32 > proto::APP_MAX {
        return Err(format!("{path}: {} bytes, must be 1..={}", image.len(), proto::APP_MAX));
    }
    let (d, info) = Dev::wait(api, None, Duration::from_secs(5))?;
    let d = if info[1] == proto::mode::APP {
        println!("app running, resetting into the bootloader");
        let _ = d.request(&req(cmd::BOOTLOADER), msg::ACK);
        drop(d);
        sleep(Duration::from_millis(500));
        Dev::wait(api, Some(proto::mode::BOOT), Duration::from_secs(10))?.0
    } else {
        d
    };
    println!("erasing");
    let t = Instant::now();
    let mut r = req(cmd::ERASE);
    proto::put_u32(&mut r, 1, image.len() as u32); // older bootloaders erase everything
    d.command(&r)?;
    println!("erased in {:.1} s", t.elapsed().as_secs_f32());
    let t = Instant::now();
    for (i, chunk) in image.chunks(32).enumerate() {
        let mut r = req(cmd::WRITE);
        proto::put_u32(&mut r, 1, (i * 32) as u32);
        r[5] = chunk.len() as u8;
        r[8..8 + chunk.len()].copy_from_slice(chunk);
        d.command(&r)?;
        if i % 256 == 0 {
            eprint!("\rwriting {} / {} bytes", i * 32, image.len());
        }
    }
    eprintln!("\rwrote {} bytes in {:.1} s        ", image.len(), t.elapsed().as_secs_f32());
    let mut r = req(cmd::COMMIT);
    proto::put_u32(&mut r, 1, image.len() as u32);
    proto::put_u32(&mut r, 5, proto::crc32(&image));
    d.command(&r)?;
    println!("verified (crc {:08x}), starting the app", proto::crc32(&image));
    let _ = d.request(&req(cmd::RUN), msg::ACK);
    drop(d);
    sleep(Duration::from_millis(500));
    let (_, info) = Dev::wait(api, Some(proto::mode::APP), Duration::from_secs(10))?;
    print_info(&info);
    Ok(())
}

/// The app rewrites the bootloader. If this fails half way, only the ROM bootloader (BOOT0
/// pads under the screen) can bring the device back, so it checks everything first.
fn flash_boot(api: &mut HidApi, path: &str) -> Result<()> {
    let image = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if image.len() < 1024 || image.len() as u32 > proto::BOOT_SIZE {
        return Err(format!("{path}: {} bytes, not a bootloader", image.len()));
    }
    let (d, info) = Dev::wait(api, None, Duration::from_secs(5))?;
    let d = if info[1] == proto::mode::BOOT {
        println!("in the bootloader, starting the app first");
        let _ = d.request(&req(cmd::RUN), msg::ACK);
        drop(d);
        sleep(Duration::from_millis(500));
        Dev::wait(api, Some(proto::mode::APP), Duration::from_secs(10))?.0
    } else {
        d
    };
    let mut r = req(cmd::BOOT_ERASE);
    proto::put_u32(&mut r, 1, image.len() as u32);
    d.command(&r)?;
    for (i, chunk) in image.chunks(32).enumerate() {
        let mut r = req(cmd::BOOT_WRITE);
        proto::put_u32(&mut r, 1, (i * 32) as u32);
        r[5] = chunk.len() as u8;
        r[8..8 + chunk.len()].copy_from_slice(chunk);
        d.command(&r)?;
    }
    let mut r = req(cmd::BOOT_COMMIT);
    proto::put_u32(&mut r, 1, image.len() as u32);
    proto::put_u32(&mut r, 5, proto::crc32(&image));
    d.command(&r)?;
    println!("bootloader written and verified ({} bytes, crc {:08x}); testing it", image.len(), proto::crc32(&image));
    let _ = d.request(&req(cmd::BOOTLOADER), msg::ACK);
    drop(d);
    sleep(Duration::from_millis(500));
    let (d, info) = Dev::wait(api, Some(proto::mode::BOOT), Duration::from_secs(10))?;
    print_info(&info);
    let _ = d.request(&req(cmd::RUN), msg::ACK);
    drop(d);
    sleep(Duration::from_millis(500));
    print_info(&Dev::wait(api, Some(proto::mode::APP), Duration::from_secs(10))?.1);
    Ok(())
}

fn monitor(d: &Dev) -> Result<()> {
    let mut r = req(cmd::STREAM);
    r[1] = 1;
    d.command(&r)?;
    let mut last = Instant::now() - Duration::from_secs(1);
    loop {
        let Some(m) = d.recv(1000)? else { continue };
        match m[0] {
            msg::RAW => {
                if last.elapsed() >= Duration::from_millis(100) {
                    last = Instant::now();
                    let v = |o| proto::get_u16(&m, o);
                    println!(
                        "adc {:4} {:4} {:4}  rest {:4} {:4} {:4}  travel {:4} {:4} {:4}  keys {:03b}  pins PB14..12 {:03b}",
                        v(1), v(3), v(5), v(7), v(9), v(11), v(19), v(21), v(23), m[14], m[13]
                    );
                }
            }
            msg::EVENT => println!("event {}", describe(&m)),
            _ => {}
        }
    }
}

fn describe(m: &[u8; 64]) -> String {
    let v = i16::from_le_bytes([m[3], m[4]]);
    match m[1] {
        event::KEY => format!("key {} {}", m[2], if v != 0 { "down" } else { "up" }),
        event::KNOB => format!("knob {v:+}"),
        event::KNOB_BUTTON => format!("knob button {}", if v != 0 { "down" } else { "up" }),
        k => format!("kind {k} index {} value {v}", m[2]),
    }
}

fn text(d: &Dev, line: u8, color: u16, s: &str) -> Result<()> {
    let mut r = req(cmd::TEXT);
    r[1] = line;
    proto::put_u16(&mut r, 2, color);
    let b = s.as_bytes();
    let n = b.len().min(59);
    r[4] = n as u8;
    r[5..5 + n].copy_from_slice(&b[..n]);
    d.command(&r)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let a: Vec<&str> = args.iter().map(|s| s.as_str()).collect();
    let res = (|| -> Result<()> {
        let mut api = HidApi::new().map_err(|e| e.to_string())?;
        match a.as_slice() {
            ["info"] => print_info(&Dev::open(&mut api)?.info()?),
            ["flash", path] => flash(&mut api, path)?,
            ["flash-boot", path] => flash_boot(&mut api, path)?,
            ["boot"] => {
                let d = Dev::open(&mut api)?;
                let _ = d.request(&req(cmd::BOOTLOADER), msg::ACK);
                drop(d);
                sleep(Duration::from_millis(500));
                print_info(&Dev::wait(&mut api, Some(proto::mode::BOOT), Duration::from_secs(10))?.1);
            }
            ["reset"] => {
                let d = Dev::open(&mut api)?;
                let info = d.info()?;
                let c = if info[1] == proto::mode::BOOT { cmd::RUN } else { cmd::RESET };
                let _ = d.request(&req(c), msg::ACK);
                drop(d);
                sleep(Duration::from_millis(500));
                print_info(&Dev::wait(&mut api, None, Duration::from_secs(10))?.1);
            }
            ["monitor"] => monitor(&Dev::open(&mut api)?)?,
            ["leds"] => {
                let r = req(cmd::LEDS);
                Dev::open(&mut api)?.command(&r)?;
            }
            ["leds", rest @ ..] if rest.len() >= 9 => {
                let mut r = req(cmd::LEDS);
                for (i, v) in rest[..9].iter().enumerate() {
                    r[1 + i] = v.parse().map_err(|_| format!("bad level {v}"))?;
                }
                r[10] = 1;
                Dev::open(&mut api)?.command(&r)?;
            }
            ["text", line, rest @ ..] if !rest.is_empty() => {
                let (words, color) = match rest.last().and_then(|c| u16::from_str_radix(c.trim_start_matches("0x"), 16).ok().filter(|_| c.starts_with("0x"))) {
                    Some(c) => (&rest[..rest.len() - 1], c),
                    None => (rest, 0xFFFF),
                };
                text(&Dev::open(&mut api)?, line.parse().map_err(|_| "bad line")?, color, &words.join(" "))?;
            }
            ["threshold", key, press, release] => {
                let mut r = req(cmd::THRESHOLDS);
                r[1] = key.parse().map_err(|_| "bad key")?;
                proto::put_u16(&mut r, 2, press.parse().map_err(|_| "bad press")?);
                proto::put_u16(&mut r, 4, release.parse().map_err(|_| "bad release")?);
                Dev::open(&mut api)?.command(&r)?;
            }
            ["run", path] => run::run(&mut api, path)?,
            ["blink", led, r, g, b, rest @ ..] => {
                let mut q = req(cmd::BLINK);
                q[1] = led.parse().map_err(|_| "bad led")?;
                for (i, v) in [r, g, b].iter().enumerate() {
                    q[2 + i] = v.parse().map_err(|_| "bad colour")?;
                }
                proto::put_u16(&mut q, 5, 250);
                q[7] = rest.first().and_then(|c| c.parse().ok()).unwrap_or(1);
                Dev::open(&mut api)?.command(&q)?;
            }
            ["pin", port, pin, mode, level] => {
                // o3c pin A 3 3 0  = PA3 push-pull output low; mode 4 = floating input
                let mut r = req(cmd::PIN);
                r[1] = match *port { "A" | "a" => 0, "B" | "b" => 1, "C" | "c" => 2, _ => return Err("port A, B or C".into()) };
                r[2] = pin.parse().map_err(|_| "bad pin")?;
                r[3] = u8::from_str_radix(mode.trim_start_matches("0x"), 16).map_err(|_| "bad mode")?;
                r[4] = match *level { "0" => 0, "1" => 1, _ => 2 };
                let a = Dev::open(&mut api)?.request(&r, msg::ACK)?;
                println!("port input register {:08x}", proto::get_u32(&a, 3));
            }
            _ => return Err("usage: o3c info | flash <app.bin> | flash-boot <boot.bin> | boot | reset | monitor | leds [r g b x3] | text <line> <text> [0xRGB565] | threshold <key> <press> <release> | blink <led> <r> <g> <b> [count] | pin <port> <pin> <mode> <level> | run <config.toml>".into()),
        }
        Ok(())
    })();
    match res {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("o3c: {e}");
            ExitCode::FAILURE
        }
    }
}
