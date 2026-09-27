//! What a duck says, into the room rather than at the machine's speakers.
//!
//! The `aplay` shim execs this when a field is running. It takes the two argv shapes the
//! daemons use — a wav from the bank, or raw S16_LE on stdin — and writes canonical PCM to
//! that duck's speaker port. From there the field decides who hears it, how loudly, and how
//! long it takes to arrive.
//!
//!     duck-speak --port 7951 --file greet_a.wav
//!     duck-speak --port 7951 --raw --rate 48000 --channels 1   # PCM on stdin
//!
//! **WAV is parsed here rather than by ffmpeg**, because the bank is the common case and a
//! Linux box with ALSA has no reason to have ffmpeg installed. It is sixteen-bit PCM in a
//! RIFF container; the parsing is forty lines and the dependency would be forty megabytes.
//!
//! Nothing is paced. The field lets each speaker out at the rate of the world, so a one
//! second wav arriving in one burst still plays over one second.

use std::io::{Read, Write};
use std::net::TcpStream;

const TARGET_HZ: u32 = 48_000;

fn main() {
    let mut port: u16 = 0;
    let mut file: Option<String> = None;
    let mut raw = false;
    let mut rate: u32 = TARGET_HZ;
    let mut channels: u16 = 1;

    let mut argv = std::env::args().skip(1);
    while let Some(flag) = argv.next() {
        match flag.as_str() {
            "--port" => port = argv.next().and_then(|v| v.parse().ok()).unwrap_or(0),
            "--file" => file = argv.next(),
            "--raw" => raw = true,
            "--rate" => rate = argv.next().and_then(|v| v.parse().ok()).unwrap_or(TARGET_HZ),
            "--channels" => channels = argv.next().and_then(|v| v.parse().ok()).unwrap_or(1),
            other => {
                eprintln!("duck-speak: unknown argument {other}");
                std::process::exit(2);
            }
        }
    }

    let samples = if let Some(path) = file {
        match std::fs::read(&path).ok().and_then(|bytes| decode_wav(&bytes)) {
            Some((pcm, hz, ch)) => resample(&pcm, hz, ch),
            // A sound that will not decode is a sound the duck does not make. Silence is
            // the right answer and an exit code nobody reads is not: `robotd` logs a failed
            // child at debug and carries on, so a loud failure here would be invisible
            // anyway and a crash loop would not.
            None => {
                eprintln!("duck-speak: cannot read {path} as 16-bit PCM wav");
                return;
            }
        }
    } else if raw {
        let mut bytes = Vec::new();
        if std::io::stdin().read_to_end(&mut bytes).is_err() {
            return;
        }
        let pcm: Vec<i16> = bytes.as_chunks::<2>().0.iter().copied().map(i16::from_le_bytes).collect();
        resample(&pcm, rate, channels)
    } else {
        return;
    };

    if samples.is_empty() || port == 0 {
        return;
    }

    // A field that is not running is a codec that is not fitted. Quiet, not fatal.
    let Ok(mut stream) = TcpStream::connect(("127.0.0.1", port)) else {
        return;
    };
    let mut bytes = Vec::with_capacity(samples.len() * 2);
    for sample in &samples {
        bytes.extend_from_slice(&sample.to_le_bytes());
    }
    let _ = stream.write_all(&bytes);
}

/// 16-bit PCM out of a RIFF container: the rate, the channel count and the samples.
///
/// Chunks are walked rather than assumed to be in order, because a renderer is free to put
/// `LIST` or `fact` between `fmt ` and `data` and several do.
fn decode_wav(bytes: &[u8]) -> Option<(Vec<i16>, u32, u16)> {
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    let mut at = 12;
    let mut rate = 0u32;
    let mut channels = 0u16;
    let mut bits = 0u16;
    let mut pcm = None;

    while at + 8 <= bytes.len() {
        let id = &bytes[at..at + 4];
        let size = u32::from_le_bytes(bytes[at + 4..at + 8].try_into().ok()?) as usize;
        let body = at + 8;
        if body + size > bytes.len() {
            break;
        }
        match id {
            b"fmt " if size >= 16 => {
                channels = u16::from_le_bytes(bytes[body + 2..body + 4].try_into().ok()?);
                rate = u32::from_le_bytes(bytes[body + 4..body + 8].try_into().ok()?);
                bits = u16::from_le_bytes(bytes[body + 14..body + 16].try_into().ok()?);
            }
            b"data" => {
                pcm = Some(
                    bytes[body..body + size]
                        .as_chunks::<2>()
                        .0
                        .iter()
                        .copied()
                        .map(i16::from_le_bytes)
                        .collect::<Vec<_>>(),
                );
            }
            _ => {}
        }
        // Chunks are word-aligned, and a renderer that emits an odd-sized one pads it. Not
        // skipping the pad byte walks into the next header one byte out, which reads as a
        // truncated file rather than as a parser bug.
        at = body + size + (size & 1);
    }

    if bits != 16 || rate == 0 || channels == 0 {
        return None;
    }
    pcm.map(|samples| (samples, rate, channels))
}

/// To 48 kHz mono, which is what the field carries.
///
/// Linear interpolation, and the bank needs none of it: `sounds` renders at exactly this
/// rate in mono, so the common path is a copy. The arithmetic is here for the theremin,
/// whose rate `robotd` passes on the command line and is free to change.
fn resample(pcm: &[i16], rate: u32, channels: u16) -> Vec<i16> {
    let channels = channels.max(1) as usize;
    let mono: Vec<f32> = if channels == 1 {
        pcm.iter().map(|&s| s as f32).collect()
    } else {
        pcm.chunks_exact(channels)
            .map(|frame| frame.iter().map(|&s| s as f32).sum::<f32>() / channels as f32)
            .collect()
    };
    if rate == TARGET_HZ || mono.is_empty() {
        return mono.iter().map(|&s| s as i16).collect();
    }

    let ratio = TARGET_HZ as f64 / rate as f64;
    let out_len = (mono.len() as f64 * ratio).round() as usize;
    let mut out = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let source = i as f64 / ratio;
        let left = source.floor() as usize;
        let frac = (source - left as f64) as f32;
        let a = mono.get(left).copied().unwrap_or(0.0);
        let b = mono.get(left + 1).copied().unwrap_or(a);
        out.push((a + (b - a) * frac) as i16);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wav(rate: u32, channels: u16, bits: u16, samples: &[i16], pad_chunk: bool) -> Vec<u8> {
        let data: Vec<u8> = samples.iter().flat_map(|s| s.to_le_bytes()).collect();
        let mut out = Vec::new();
        out.extend_from_slice(b"RIFF");
        out.extend_from_slice(&0u32.to_le_bytes()); // size: unread by the parser
        out.extend_from_slice(b"WAVE");
        if pad_chunk {
            // An odd-sized chunk between `fmt ` and `data`, which a renderer is free to
            // emit and several do. It is word-aligned with a pad byte the parser must skip.
            out.extend_from_slice(b"LIST");
            out.extend_from_slice(&3u32.to_le_bytes());
            out.extend_from_slice(b"abc\0");
        }
        out.extend_from_slice(b"fmt ");
        out.extend_from_slice(&16u32.to_le_bytes());
        out.extend_from_slice(&1u16.to_le_bytes()); // PCM
        out.extend_from_slice(&channels.to_le_bytes());
        out.extend_from_slice(&rate.to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // byte rate: unread
        out.extend_from_slice(&0u16.to_le_bytes()); // block align: unread
        out.extend_from_slice(&bits.to_le_bytes());
        out.extend_from_slice(b"data");
        out.extend_from_slice(&(data.len() as u32).to_le_bytes());
        out.extend_from_slice(&data);
        out
    }

    #[test]
    fn the_bank_decodes() {
        // What `sounds` renders: 16-bit mono at exactly the rate the field carries.
        let bytes = wav(48_000, 1, 16, &[1, -2, 3, -4], false);
        let (pcm, rate, channels) = decode_wav(&bytes).expect("decodes");
        assert_eq!(pcm, vec![1, -2, 3, -4]);
        assert_eq!((rate, channels), (48_000, 1));
    }

    #[test]
    fn a_chunk_between_fmt_and_data_is_stepped_over() {
        // Not skipping an odd chunk's pad byte walks into the next header one byte out,
        // which reads as a truncated file rather than as a parser bug.
        let bytes = wav(48_000, 1, 16, &[7, 8], true);
        let (pcm, _, _) = decode_wav(&bytes).expect("decodes past the LIST chunk");
        assert_eq!(pcm, vec![7, 8]);
    }

    #[test]
    fn what_is_not_a_wav_is_not_a_sound() {
        assert!(decode_wav(b"").is_none());
        assert!(decode_wav(b"not a riff file at all").is_none());
        // 8-bit is a wav and is not one this handles; saying so beats returning noise.
        assert!(decode_wav(&wav(48_000, 1, 8, &[1, 2], false)).is_none());
    }

    #[test]
    fn stereo_is_folded_to_one_ear() {
        // Interleaved L,R: the field is mono, and averaging is what a single capsule does.
        let out = resample(&[100, 300, -100, -300], 48_000, 2);
        assert_eq!(out, vec![200, -200]);
    }

    #[test]
    fn the_common_rate_is_a_copy() {
        let pcm = vec![5i16, 6, 7];
        assert_eq!(resample(&pcm, TARGET_HZ, 1), pcm);
    }

    #[test]
    fn a_lower_rate_is_stretched_to_the_field() {
        // The theremin's rate comes from robotd's command line and is free to differ.
        let out = resample(&[0, 100], 24_000, 1);
        assert_eq!(out.len(), 4, "half the rate is twice the samples");
    }
}
