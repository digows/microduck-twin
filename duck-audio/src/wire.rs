//! The two ports a duck's codec talks to.
//!
//! Raw PCM, signed 16-bit little-endian, mono, and no handshake — the same reasoning the
//! camera port carries in `microduck_rl`: there is nothing to negotiate that both ends do
//! not already have to agree on to be useful. The rates are the ones the software asks for
//! rather than a choice: `sounds` renders at 48 kHz and `pet-detect` runs `arecord` at 16.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc::{Receiver, SyncSender};

/// What the voice bank and the synth produce.
pub const SPEAKER_HZ: u32 = 48_000;

/// What `pet-detect` asks `arecord` for.
pub const MIC_HZ: u32 = 16_000;

/// Ten milliseconds, at either rate. Short enough that a sound arrives when it happens and
/// long enough that the mixer is not woken four thousand times a second.
pub const BLOCK_MS: u32 = 10;
pub const SPEAKER_BLOCK: usize = (SPEAKER_HZ / 1000 * BLOCK_MS) as usize;
pub const MIC_BLOCK: usize = (MIC_HZ / 1000 * BLOCK_MS) as usize;

/// Accept one writer at a time and forward what it sends.
///
/// One at a time because the codec is: `robotd` kills the playing child before starting the
/// next, so a second connection means the first is finished. A listener that queued them
/// would play a stale quack after a live one.
pub fn serve_speaker(port: u16, out: SyncSender<Vec<i16>>) -> std::io::Result<()> {
    let listener = TcpListener::bind(("127.0.0.1", port))?;
    for stream in listener.incoming() {
        let Ok(mut stream) = stream else { continue };
        let out = out.clone();
        std::thread::spawn(move || {
            let mut buf = vec![0u8; SPEAKER_BLOCK * 2];
            loop {
                match stream.read(&mut buf) {
                    Ok(0) | Err(_) => break,
                    Ok(n) => {
                        let samples = buf[..n - (n % 2)]
                            .chunks_exact(2)
                            .map(|b| i16::from_le_bytes([b[0], b[1]]))
                            .collect::<Vec<_>>();
                        if !samples.is_empty() && out.send(samples).is_err() {
                            break;
                        }
                    }
                }
            }
        });
    }
    Ok(())
}

/// Hand every connected reader the same mixed stream.
///
/// `arecord` is one reader and the console is another, and both want what the ear hears. A
/// reader that stops keeping up is dropped rather than allowed to back the mixer up: a real
/// capture device overruns, it does not slow the room down.
pub fn serve_mic(port: u16, blocks: Receiver<Vec<i16>>) -> std::io::Result<()> {
    let listener = TcpListener::bind(("127.0.0.1", port))?;
    let readers: std::sync::Arc<std::sync::Mutex<Vec<TcpStream>>> = Default::default();

    {
        let readers = readers.clone();
        std::thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                let _ = stream.set_nodelay(true);
                readers.lock().unwrap().push(stream);
            }
        });
    }

    for block in blocks {
        let mut bytes = Vec::with_capacity(block.len() * 2);
        for sample in &block {
            bytes.extend_from_slice(&sample.to_le_bytes());
        }
        let mut held = readers.lock().unwrap();
        held.retain_mut(|stream| stream.write_all(&bytes).is_ok());
    }
    Ok(())
}
