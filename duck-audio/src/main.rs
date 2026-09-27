//! duck-audio — the air between simulated ducks.
//!
//! One process, one connection per duck, in the shape `duck-ether` established for the fake
//! radio: geometry polled from the body over TCP, and physics derived from the distance
//! between two of them. What it carries is sound rather than beacons.
//!
//!     duck-audio --ducks 2 --body-port 7801 --audio-port 7951
//!
//! Each duck gets two ports off the audio base: `base + 2i` is its speaker, which the
//! `aplay` shim writes to, and `base + 2i + 1` is its microphone, which `arecord` reads.
//! Raw S16_LE mono both ways, 48 kHz out and 16 kHz in, because those are the rates the
//! software asks for rather than a choice anyone made here.

mod body;
mod field;
mod wire;

use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use body::{Geometry, Heard};
use field::Source;
use wire::{BLOCK_MS, MIC_BLOCK, SPEAKER_BLOCK};

struct Args {
    ducks: usize,
    body_port: u16,
    audio_port: u16,
}

fn parse() -> Args {
    let mut args = Args { ducks: 1, body_port: 7801, audio_port: 7951 };
    let mut argv = std::env::args().skip(1);
    while let Some(flag) = argv.next() {
        let mut value = || argv.next().and_then(|v| v.parse().ok());
        match flag.as_str() {
            "--ducks" => args.ducks = value().unwrap_or(1) as usize,
            "--body-port" => args.body_port = value().unwrap_or(7801) as u16,
            "--audio-port" => args.audio_port = value().unwrap_or(7951) as u16,
            "-h" | "--help" => {
                eprintln!("{}", USAGE);
                std::process::exit(0);
            }
            other => {
                eprintln!("duck-audio: unknown argument {other}\n\n{USAGE}");
                std::process::exit(2);
            }
        }
    }
    args
}

const USAGE: &str = "\
duck-audio --ducks N --body-port 7801 --audio-port 7951

  Duck i speaks on audio-port + 2i and hears on audio-port + 2i + 1.
  Raw S16_LE mono: 48 kHz into a speaker, 16 kHz out of a microphone.";

fn main() {
    let args = parse();

    let geometry: Geometry = Arc::new(Mutex::new(vec![Heard::default(); args.ducks]));
    let sources: Vec<Arc<Mutex<Source>>> =
        (0..args.ducks).map(|_| Arc::new(Mutex::new(Source::default()))).collect();

    // One poller per duck, each holding its own connection to the body.
    for index in 0..args.ducks {
        let geometry = geometry.clone();
        let port = args.body_port + index as u16;
        std::thread::spawn(move || body::poll(index, port, geometry));
    }

    // Speakers in, microphones out.
    let mut mics: Vec<SyncSender<Vec<i16>>> = Vec::with_capacity(args.ducks);
    for (index, source) in sources.iter().enumerate() {
        let speaker_port = args.audio_port + 2 * index as u16;
        let mic_port = speaker_port + 1;

        let (pcm_in, pcm_rx) = sync_channel::<Vec<i16>>(64);
        let source = source.clone();
        std::thread::spawn(move || drain(pcm_rx, source));
        std::thread::spawn(move || {
            if let Err(e) = wire::serve_speaker(speaker_port, pcm_in) {
                eprintln!("duck-audio: speaker {speaker_port}: {e}");
            }
        });

        // Bounded, and a full channel drops rather than blocks: a reader that has stopped
        // keeping up must not slow the room down, the way a real capture device overruns.
        let (mic_tx, mic_rx) = sync_channel::<Vec<i16>>(8);
        mics.push(mic_tx);
        std::thread::spawn(move || {
            if let Err(e) = wire::serve_mic(mic_port, mic_rx) {
                eprintln!("duck-audio: microphone {mic_port}: {e}");
            }
        });

        println!(
            "== duck {}: speaks on {speaker_port}, hears on {mic_port}",
            (b'a' + index as u8) as char
        );
    }

    println!("== the air is up for {} duck(s)", args.ducks);
    mix(args.ducks, sources, geometry, mics);
}

fn drain(pcm: Receiver<Vec<i16>>, source: Arc<Mutex<Source>>) {
    for block in pcm {
        source.lock().unwrap().push(&block);
    }
}

/// The room, ten milliseconds at a time.
///
/// **Paced against the wall clock**, like every loop the daemons run, because the world is
/// meant to run at 1.00x and a mixer with its own idea of time would drift away from the
/// body it is describing.
fn mix(
    ducks: usize,
    sources: Vec<Arc<Mutex<Source>>>,
    geometry: Geometry,
    mics: Vec<SyncSender<Vec<i16>>>,
) {
    let period = Duration::from_millis(BLOCK_MS as u64);
    let mut next = Instant::now();
    let mut accum = vec![0f32; SPEAKER_BLOCK];
    let mut out = Vec::with_capacity(MIC_BLOCK);

    loop {
        next += period;

        // Let every speaker out into the room first, so each listener reads the same instant.
        for source in &sources {
            source.lock().unwrap().advance();
        }

        let table = geometry.lock().unwrap().clone();
        for listener in 0..ducks {
            let heard = &table[listener];
            accum.iter_mut().for_each(|s| *s = 0.0);

            if heard.known {
                // Itself, through its own head.
                sources[listener].lock().unwrap().delayed_block(0, &mut accum, field::SELF_GAIN);

                for peer in &heard.peers {
                    let Some(source) = sources.get(peer.index) else { continue };
                    let gain = field::distance_gain(peer.distance, peer.blocked);
                    let delay = field::delay_samples(peer.distance);
                    source.lock().unwrap().delayed_block(delay, &mut accum, gain);
                }
            }

            field::downsample(&accum, &mut out);
            // A full channel means nobody is reading fast enough; the block is dropped
            // rather than allowed to hold the room up.
            let _ = mics[listener].try_send(out.clone());
        }

        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        } else {
            // Behind: give up the backlog rather than chase it, so a stall does not turn
            // into a burst of stale audio.
            next = now;
        }
    }
}
