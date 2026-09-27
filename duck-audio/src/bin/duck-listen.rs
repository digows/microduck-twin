//! What a duck hears, onto stdout.
//!
//! The `arecord` shim execs this. It connects to that duck's microphone port and copies the
//! mixed room to stdout as raw S16_LE at 16 kHz — byte for byte what `pet-detect` asks
//! `arecord` for, so the worker that reads it cannot tell there is no codec.
//!
//!     duck-listen --port 7952
//!
//! **It waits rather than exits.** `pet-detect` restarts `arecord` whenever it ends, and
//! upstream records what that costs when the binary is missing: the worker fork/execs as
//! fast as the CPU allows, for the life of the daemon. A microphone whose field has not
//! started yet is a microphone that is quiet, so this retries on a slow timer and stays
//! silent in between.

use std::io::{ErrorKind, Read, Write};
use std::net::TcpStream;
use std::time::Duration;

fn main() {
    let mut port: u16 = 0;
    let mut argv = std::env::args().skip(1);
    while let Some(flag) = argv.next() {
        match flag.as_str() {
            "--port" => port = argv.next().and_then(|v| v.parse().ok()).unwrap_or(0),
            other => {
                eprintln!("duck-listen: unknown argument {other}");
                std::process::exit(2);
            }
        }
    }
    if port == 0 {
        eprintln!("duck-listen: --port is required");
        std::process::exit(2);
    }

    let mut out = std::io::stdout();
    let mut buf = [0u8; 4096];
    loop {
        match TcpStream::connect(("127.0.0.1", port)) {
            Ok(mut stream) => {
                let _ = stream.set_nodelay(true);
                // **Whole samples only.** A read returns whatever arrived, which splits a
                // two-byte sample down the middle about half the time; `pet-detect` counts
                // the bytes it gets and calls an odd number a broken `arecord`, then
                // restarts it — which it did, several times a second, until this carried
                // the stray byte over instead of writing it.
                let mut odd: Option<u8> = None;
                loop {
                    match stream.read(&mut buf) {
                        Ok(0) => break,
                        Ok(n) => {
                            let mut frame: Vec<u8> = Vec::with_capacity(n + 1);
                            if let Some(byte) = odd.take() {
                                frame.push(byte);
                            }
                            frame.extend_from_slice(&buf[..n]);
                            if frame.len() % 2 == 1 {
                                odd = frame.pop();
                            }
                            if frame.is_empty() {
                                continue;
                            }
                            // A closed stdout is the reader going away, which is the one
                            // reason to stop rather than reconnect.
                            if let Err(e) = out.write_all(&frame) {
                                if e.kind() == ErrorKind::BrokenPipe {
                                    return;
                                }
                                break;
                            }
                            if out.flush().is_err() {
                                break;
                            }
                        }
                        Err(_) => break,
                    }
                }
            }
            Err(_) => std::thread::sleep(Duration::from_secs(1)),
        }
    }
}
