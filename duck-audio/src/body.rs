//! Where every ear is, asked of the simulator.
//!
//! The same move `duck-ether` makes for the radio: a TCP connection to the body, a `hello`,
//! and a poll. It asks `hears`, which `twin-body` adds — the ear's pose, and for every other
//! duck the distance and whether anything is in the way. One round trip per poll rather than
//! one per pair, and the ray casting stays where the model is.
//!
//! **A simulator that predates the op answers `unknown op`**, and that is not an error here:
//! the field falls back to silence for that duck rather than refusing to start. An upstream
//! `duck-body` is a body with no located ear, which is a duck the room cannot place.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Geometry does not change at audio rate. Twenty times a second is four times what
/// `duck-ether` uses for the radio and still nothing next to 4800 mixer ticks a second; a
/// head turning through 90 degrees takes about a second, so this resolves it in twenty steps.
pub const POLL: Duration = Duration::from_millis(50);

#[derive(Clone, Debug, Default)]
pub struct Peer {
    pub index: usize,
    pub distance: f32,
    pub blocked: bool,
}

#[derive(Clone, Debug, Default)]
pub struct Heard {
    /// False until the body has answered once. Nothing is mixed for a duck that has not.
    pub known: bool,
    pub peers: Vec<Peer>,
}

pub type Geometry = Arc<Mutex<Vec<Heard>>>;

/// Poll one duck's body forever, writing what it says into the shared table.
pub fn poll(index: usize, port: u16, geometry: Geometry) {
    loop {
        match poll_once(index, port, &geometry) {
            Ok(()) => {}
            Err(_) => {
                // A dead simulator is one bad poll. MuJoCo recompiles its model when the
                // number of ducks changes, so the body goes away and comes back; the
                // connection is the retry timer, exactly as it is for the daemons.
                if let Ok(mut table) = geometry.lock() {
                    if let Some(entry) = table.get_mut(index) {
                        entry.known = false;
                    }
                }
                std::thread::sleep(Duration::from_millis(500));
            }
        }
    }
}

fn poll_once(index: usize, port: u16, geometry: &Geometry) -> std::io::Result<()> {
    let stream = TcpStream::connect(("127.0.0.1", port))?;
    stream.set_nodelay(true)?;
    let mut write = stream.try_clone()?;
    let mut read = BufReader::new(stream);

    write.write_all(b"{\"op\":\"hello\",\"protocol\":1,\"joints\":15}\n")?;
    let mut line = String::new();
    read.read_line(&mut line)?;

    loop {
        line.clear();
        write.write_all(b"{\"op\":\"hears\"}\n")?;
        if read.read_line(&mut line)? == 0 {
            return Err(std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "body closed"));
        }
        let value: serde_json::Value = serde_json::from_str(&line).unwrap_or_default();

        let mut heard = Heard::default();
        if value.get("error").is_none() && value.get("mic").map(|m| !m.is_null()).unwrap_or(false) {
            heard.known = true;
            if let Some(peers) = value.get("peers").and_then(|p| p.as_array()) {
                for peer in peers {
                    heard.peers.push(Peer {
                        index: peer.get("index").and_then(|v| v.as_u64()).unwrap_or(0) as usize,
                        distance: peer.get("distance").and_then(|v| v.as_f64()).unwrap_or(0.0) as f32,
                        blocked: peer.get("blocked").and_then(|v| v.as_bool()).unwrap_or(false),
                    });
                }
            }
        }

        if let Ok(mut table) = geometry.lock() {
            if let Some(entry) = table.get_mut(index) {
                *entry = heard;
            }
        }
        std::thread::sleep(POLL);
    }
}
