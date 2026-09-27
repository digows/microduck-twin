//! The air between the ducks.
//!
//! Three effects, and each is in the model for a reason upstream already argued for the
//! radio: a channel that is perfect hides the bugs a real one causes.
//!
//!  - **Distance.** Sound falls off as 1/r past a reference radius. Two ducks a room apart
//!    do not hear each other the way two beak to beak do, and anything that keys on level
//!    has to cope with that.
//!  - **Delay.** 343 m/s. Across a 7 m flat that is 20 ms, which is a third of a control
//!    tick — small, and exactly the kind of small that a synchroniser gets wrong.
//!  - **Occlusion.** A wall is not a mute button. It is about 18 dB here, so a duck behind
//!    one is faint rather than absent, and a detector tuned on the open-room level has
//!    something to fail against.
//!
//! **A duck hears its own speaker**, because a real microphone on the same head does. It is
//! attenuated rather than full: the Mic3R faces out and the speaker fires through the body,
//! and a `pet-detect` that deafened itself every time the duck quacked would be a worse
//! model of the robot than one that does not.

use std::collections::VecDeque;

use crate::wire::{MIC_BLOCK, SPEAKER_BLOCK, SPEAKER_HZ};

pub const SPEED_OF_SOUND: f32 = 343.0;

/// Inside this radius a source is at full level, outside it falls off as 1/r. Without a
/// reference the gain goes to infinity as two ducks touch.
pub const REFERENCE_M: f32 = 0.25;

/// What a wall costs. Not silence: a duck in the next room is faint, and something that
/// treats faint and absent as the same thing is the bug this exists to catch.
pub const OCCLUDED_GAIN: f32 = 0.12;

/// A duck hearing itself, through its own head rather than through the air.
///
/// A few decibels down from full, not twenty. The Mic3R faces out and the speaker fires
/// through the body, so there is some isolation — but the two are centimetres apart, and a
/// duck has to hear itself louder than it hears a neighbour half a metre away. The first
/// value here was 0.30, which put the neighbour at 0.5 *above* it: measured, duck-b heard a
/// quack at -33 dBFS and the duck making it heard -37.
pub const SELF_GAIN: f32 = 0.70;

/// How much history the delay line holds: a second, against a longest credible flat of a
/// few tens of milliseconds. Cheap, and it means no scene can outrun it.
const RING: usize = SPEAKER_HZ as usize;

/// One duck's speaker: what it has said, and what has been let out into the room.
pub struct Source {
    /// Written by the codec as fast as TCP allows — a 0.9 s wav arrives in one burst.
    queued: VecDeque<i16>,
    /// Let out at the rate of the world, so a burst plays over its own length rather than
    /// in an instant. This is the delay line the listeners read from.
    ring: Vec<i16>,
    /// Absolute samples emitted, ever. The ring is this modulo its length.
    pub emitted: usize,
}

impl Default for Source {
    fn default() -> Self {
        Self { queued: VecDeque::new(), ring: vec![0; RING], emitted: 0 }
    }
}

impl Source {
    pub fn push(&mut self, samples: &[i16]) {
        // A source nobody is draining must not grow without bound. Two seconds is longer
        // than any sound in the bank and longer than the ride's lead.
        if self.queued.len() < SPEAKER_HZ as usize * 2 {
            self.queued.extend(samples.iter().copied());
        }
    }

    /// Let one block out into the room, padding with silence when there is nothing to say.
    pub fn advance(&mut self) {
        for _ in 0..SPEAKER_BLOCK {
            let sample = self.queued.pop_front().unwrap_or(0);
            let slot = self.emitted % RING;
            self.ring[slot] = sample;
            self.emitted += 1;
        }
    }

    /// The block that left this speaker `delay` samples ago.
    pub fn delayed_block(&self, delay: usize, out: &mut [f32], gain: f32) {
        if self.emitted < SPEAKER_BLOCK + delay {
            return;
        }
        let start = self.emitted - SPEAKER_BLOCK - delay;
        for (i, slot) in out.iter_mut().enumerate().take(SPEAKER_BLOCK) {
            *slot += self.ring[(start + i) % RING] as f32 * gain;
        }
    }
}

pub fn distance_gain(distance: f32, blocked: bool) -> f32 {
    let gain = (REFERENCE_M / distance.max(REFERENCE_M)).min(1.0);
    if blocked {
        gain * OCCLUDED_GAIN
    } else {
        gain
    }
}

pub fn delay_samples(distance: f32) -> usize {
    (distance / SPEED_OF_SOUND * SPEAKER_HZ as f32).round() as usize
}

/// 48 kHz to 16 kHz, by averaging each three samples.
///
/// Averaging rather than taking every third, because dropping two in three folds everything
/// above 8 kHz back down into the band — and the duck's own voice reaches 4 kHz, so the
/// aliases would land on top of the signal `pet-detect` is classifying.
pub fn downsample(block: &[f32], out: &mut Vec<i16>) {
    out.clear();
    for chunk in block.as_chunks::<3>().0.iter().take(MIC_BLOCK) {
        let mean = (chunk[0] + chunk[1] + chunk[2]) / 3.0;
        out.push(mean.clamp(i16::MIN as f32, i16::MAX as f32) as i16);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_duck_hears_itself_louder_than_a_neighbour() {
        // The whole reason SELF_GAIN is what it is. Two ducks stand half a metre apart —
        // `build_world` spaces them exactly that — and the one that speaks must not be the
        // quieter of the two.
        assert!(SELF_GAIN > distance_gain(0.5, false));
    }

    #[test]
    fn distance_falls_off_and_never_divides_by_nothing() {
        assert_eq!(distance_gain(REFERENCE_M, false), 1.0);
        assert_eq!(distance_gain(0.0, false), 1.0);
        assert!(distance_gain(2.0, false) < distance_gain(1.0, false));
        // 1/r: twice as far is half as loud.
        let near = distance_gain(1.0, false);
        let far = distance_gain(2.0, false);
        assert!((near / far - 2.0).abs() < 1e-5);
    }

    #[test]
    fn a_wall_muffles_rather_than_mutes() {
        let open = distance_gain(1.0, false);
        let walled = distance_gain(1.0, true);
        assert!(walled > 0.0, "a wall is not a mute button");
        assert!(walled < open / 5.0, "and it is not a curtain either");
    }

    #[test]
    fn sound_takes_time_to_cross_a_room() {
        assert_eq!(delay_samples(0.0), 0);
        // Seven metres is the long diagonal of the apartment: about 20 ms, which is a third
        // of a control tick and exactly the size a synchroniser gets wrong.
        let across = delay_samples(7.0) as f32 / SPEAKER_HZ as f32;
        assert!((across - 0.0204).abs() < 0.001, "got {across}s");
    }

    #[test]
    fn a_burst_plays_over_its_own_length() {
        // The codec writes a one-second wav as fast as TCP allows. The room must let it out
        // at the rate of the world, or every sound is an instantaneous click.
        let mut source = Source::default();
        source.push(&vec![1000i16; SPEAKER_HZ as usize]);
        source.advance();
        assert_eq!(source.emitted, SPEAKER_BLOCK, "one block per tick, not the lot");
    }

    #[test]
    fn a_source_with_nothing_to_say_emits_silence() {
        let mut source = Source::default();
        source.advance();
        let mut out = vec![0f32; SPEAKER_BLOCK];
        source.delayed_block(0, &mut out, 1.0);
        assert!(out.iter().all(|&s| s == 0.0));
    }

    #[test]
    fn downsampling_averages_rather_than_drops() {
        // Taking every third sample folds everything above 8 kHz back into the band, on top
        // of the duck's own voice. The mean of 0, 3, 6 is 3.
        let block: Vec<f32> = (0..SPEAKER_BLOCK).map(|i| (i % 9) as f32).collect();
        let mut out = Vec::new();
        downsample(&block, &mut out);
        assert_eq!(out.len(), MIC_BLOCK);
        assert_eq!(out[0], 1); // (0 + 1 + 2) / 3
        assert_eq!(out[1], 4); // (3 + 4 + 5) / 3
    }
}
