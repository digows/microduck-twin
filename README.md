# microduck-twin

**The simulated duck thinks it quacked.**

[![ci](https://github.com/digows/microduck-twin/actions/workflows/ci.yml/badge.svg)](https://github.com/digows/microduck-twin/actions/workflows/ci.yml)

<!-- HERO — two ducks in a room, one quacks, the waveform arrives at the other's mic,
     a wall goes up between them and it stops. Fifteen seconds. Record it once the
     field lands; a still frame of a console is not the thing worth showing. -->

Pollen's [Microduck](https://github.com/pollen-robotics/microduck) has a digital twin, and it is
a good one. `robotd --sim` runs the real control loop, the real ONNX policies, the real safety
and fall detection against a MuJoCo body and cannot tell the difference. The duck sees, through
a rendered head camera. It feels the floor, through an 8×8 depth sensor and an IMU on the servo
bus. It walks, sits, rolls, kicks and stands back up with the networks that ship on the robot.

It cannot hear. Nothing can hear it either.

`robotctl quack` answers `🦆`. The daemon picks a variant from the duck's own voice bank —
synthesised from its SoC serial, so no two ducks sound alike — and forks `aplay`. On any machine
that is not the robot, that fork fails, is logged at debug, and is skipped by design. The duck is
silent and does not know it.

This project is the device that is missing.

## Status

| | |
|---|---|
| **One command** — `./microduck-twin up`, pinned upstreams, nothing to remember | **works** |
| **Voice** — the duck is audible on a machine with no ALSA | **works** |
| **Battery and thermals** — they move with what the robot is doing | **works** |
| **Ear** — the head microphone as a sensor of the simulated world | in progress |
| **Console** — the duck in 3D, knobs, every sensor, rollers | planned |

## Why this is not the forty-first Microduck simulator

It is not a simulator at all. The body stays Pollen's `duck-body`, the daemons stay Pollen's
daemons, and no policy is retrained. There is one MuJoCo in this picture and we did not write it.

What the ecosystem has built forty times is training: ports of the RL stack to Isaac Lab, Genesis,
JAX, PyTorch, WebAssembly, Unity. What none of them has built — and what Pollen's own design
document does not even enumerate among the things it leaves out — is the microphone as a sensor
of the world. The closest is a roadmap entry in
[microduck-lab](https://github.com/jonathanhawkins/microduck-lab/blob/main/docs/sim-roadmap.md),
unimplemented. Two projects capture from *the developer's laptop microphone*, which is a different
sensor in a different room.

A duck that cannot hear cannot be spoken to, cannot notice another duck, and cannot feel a hand on
its head. That is the difference between a robot you drive and a robot that is present.

## How it works

The whole audio seam in the shipped software is two subprocess spawns with a device string.
There is no in-process ALSA binding to replace:

| Direction | Who forks it | Command |
|---|---|---|
| Playback | `robotd/src/sound.rs`, `sounds/src/main.rs` | `aplay -q -D <dev> <file.wav>`, and `aplay -q -D <dev> -t raw -f S16_LE -c 1 -r <rate>` |
| Capture | `pet-detect/src/worker.rs` | `arecord -D <dev> -f S16_LE -r 16000 -c 1 -t raw` |

Both device strings come from **one** key, `[audio] device` in `robotd.toml`: playback uses it as
written, and `AudioParams::capture_device()` appends `,0` when no subdevice was spelled out.

So a simulated TLV320AIC3104 is a pair of executables on `PATH`, and the daemons are not modified
to gain a voice or an ear. It is the same move `RemoteIo` makes for the servo bus, one layer up.

### The body, extended rather than forked

`duck-body` reports 7.4 V and 32 °C for every duck for ever, and carries nothing about the
head — which is enough for a radio and not enough for an ear. `twin-body` subclasses two of its
classes and rebinds the names its `main()` looks up. That is the whole override; no file in
`microduck_rl` is touched, and a version bump there is a pinned-dependency change.

The battery drains from the torque the servos are holding and the daemon does the rest, by the
code that runs on the robot: it maps volts to a percentage linearly between 8.2 and 6.6 V and
powers the duck down at empty. A standing duck lasts about three quarters of an hour. The servos
warm with the square of their torque and cool toward ambient, and the hips settle near the 32 °C
the constant used to report — the one point upstream had an opinion about.

One op is added, `hears`: where this duck's ear is, and for every other duck how far away it is
and whether anything is in the way. One round trip per audio tick rather than one per pair, with
the ray casting kept where the model is. It is additive, so a simulator that predates it answers
`unknown op` and the field falls back to distance without direction.

Behind them sits the sound field: one process holding a connection per duck, in the shape
[`duck-ether`](https://github.com/pollen-robotics/microduck/blob/main/duck-ether/src/main.rs)
already established for the fake BLE radio — geometry polled from the body over TCP, and physics
derived from the distance between two of them. Attenuation, delay at the speed of sound, and
occlusion by line of sight. Deliberately imperfect, and seeded, for the reason Pollen gives for
their radio: a perfect channel hides the bugs a real one causes.

## What this is not

Pollen drew a boundary on purpose, and it holds here. Absent, and staying absent: the Dynamixel
bus driver, the BLE radio, the camera ISP and rkaiq's 3A, the NPU, the hardware encoder and its
RGA path. They ran a week of real bugs past the twin and it would have caught none of them —
a `videoflip` that cost 22 fps by breaking zero-copy, a 3A engine missing a stream-start event,
an auto-exposure loop that converges once and stops, an INT8 head whose score channel collapses.
Hardware stays the only place the drivers are real.

Also out: reverberation and HRTF, agent verbs, and any fork of a Pollen repository. Upstreams are
pinned by version and consumed as they are.

## Quickstart

```sh
git clone https://github.com/digows/microduck-twin
cd microduck-twin
./microduck-twin doctor        # what the host is missing, as the line to paste
./microduck-twin up            # the body, the daemons, the codec
./microduck-twin ctl quack     # its own voice, from its own serial — audible
./microduck-twin ctl robot do roulade
./microduck-twin down
```

The first `up` clones the two upstreams at the commits in `deps.env`, builds the daemons, makes
the simulator's venv and fetches the official policy set. After that it takes seconds. To work
against checkouts you already have, point `TWIN_MICRODUCK` and `TWIN_MICRODUCK_RL` at them.

You need `git`, `cargo` and `uv`. macOS also wants `ffmpeg` for the raw-PCM path — `afplay` is in
the base system and Linux uses its own ALSA. Without any player the duck is simply quiet, which is
a robot whose codec is not fitted.

The field and the console are not here yet; see **Status**.

## Credits

Everything this project is useful for belongs to [Pollen Robotics](https://pollen-robotics.com):
the robot, the daemons, the policies, the twin, and the design documents that are precise enough
to build against. [`docs/design/simulation.md`](https://github.com/pollen-robotics/microduck/blob/main/docs/design/simulation.md)
is the page to read before this one.

## License

Apache-2.0. The Microduck 3D meshes are CC BY-SA-NC and belong to Pollen; this repository does
not vendor them — the console loads them from a `microduck_rl` checkout at runtime.
