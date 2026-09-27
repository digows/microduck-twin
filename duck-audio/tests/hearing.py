#!/usr/bin/env python3
"""One duck quacks; the other hears it.

The unit tests in `field.rs` check the arithmetic. This checks the whole path, which is the
only place most of it exists: `robotd` picks a variant from its own bank, forks the `aplay`
shim, the shim hands the wav to `duck-speak`, the field lets it out at the rate of the world
and mixes it for every ear, and `duck-listen` carries it to whoever is holding the
microphone port open.

Needs a twin already up with at least two ducks:

    TWIN_DUCKS=2 ./microduck-twin up
    ./microduck-twin test

What it asserts is relative, never absolute. The level of a quack depends on the variant the
daemon picked and on where the ducks happen to be standing, so pinning a number would make
this fail for reasons that are not bugs. What cannot vary is the shape: quiet, then not
quiet, and the duck that spoke hears itself louder than the neighbour half a metre away.
"""

import math
import os
import struct
import subprocess
import sys
import threading
import time

AUDIO_PORT = int(os.environ.get("TWIN_AUDIO_PORT", 7951))
MIC_HZ = 16_000
LISTEN_S = 4.0
QUACK_AT_S = 1.0


def mic_port(index):
    return AUDIO_PORT + 2 * index + 1


def record(port, seconds, out):
    import socket

    try:
        sock = socket.create_connection(("127.0.0.1", port), timeout=5)
    except OSError as e:
        out.append(e)
        return
    sock.settimeout(seconds + 2)
    end = time.time() + seconds
    buf = bytearray()
    while time.time() < end:
        try:
            chunk = sock.recv(8192)
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
    sock.close()
    out.append(bytes(buf))


def dbfs(buf):
    count = len(buf) // 2
    if count == 0:
        return -120.0
    samples = struct.unpack(f"<{count}h", buf[: count * 2])
    rms = math.sqrt(sum(s * s for s in samples) / count) / 32768.0
    return 20 * math.log10(max(rms, 1e-9))


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    twin = os.path.join(os.path.dirname(here), "microduck-twin")

    ears = {}
    threads = []
    for index, name in ((0, "duck-a"), (1, "duck-b")):
        ears[name] = []
        thread = threading.Thread(target=record, args=(mic_port(index), LISTEN_S, ears[name]))
        thread.start()
        threads.append(thread)

    time.sleep(QUACK_AT_S)
    subprocess.run([twin, "ctl", "quack"], capture_output=True)
    for thread in threads:
        thread.join()

    failures = []
    levels = {}
    for name, got in ears.items():
        if not got or isinstance(got[0], Exception):
            failures.append(f"{name}: no microphone on {mic_port(0 if name == 'duck-a' else 1)} — is a twin up with two ducks?")
            continue
        buf = got[0]
        expected = int(MIC_HZ * LISTEN_S * 2 * 0.8)
        if len(buf) < expected:
            failures.append(f"{name}: {len(buf)} bytes in {LISTEN_S}s, expected about {expected}")
            continue
        # Split where the quack was asked for. The first part is the room before anyone
        # spoke; the rest carries the sound plus whatever it takes to arrive.
        cut = int(MIC_HZ * QUACK_AT_S * 2 * 0.9)
        before, after = dbfs(buf[:cut]), dbfs(buf[cut:])
        levels[name] = after
        print(f"  {name}: {before:6.1f} dBFS before, {after:6.1f} dBFS after")
        if after - before < 20:
            failures.append(f"{name}: the quack is only {after - before:.1f} dB above the room")

    if len(levels) == 2:
        a, b = levels["duck-a"], levels["duck-b"]
        if a <= b:
            failures.append(
                f"the duck that quacked hears it at {a:.1f} and its neighbour at {b:.1f} — "
                "a microphone centimetres from its own speaker cannot be the quieter one"
            )
        else:
            print(f"  the one that spoke is {a - b:.1f} dB louder to itself, as it should be")

    print()
    if failures:
        for line in failures:
            print(f"  FAIL  {line}")
        return 1
    print("a duck is heard")
    return 0


if __name__ == "__main__":
    sys.exit(main())
