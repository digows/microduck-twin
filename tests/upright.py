#!/usr/bin/env python3
"""Is the duck standing, and does it go anywhere when told to?

`robotctl health` answers "healthy" for a duck lying on its face with a control loop at
50 Hz, which is how a broken roller mode passed its own test. This asks the body instead:
gravity in the trunk frame is [0, 0, -1] for a duck that is upright, and a command that
produces no displacement produced nothing at all.
"""

import json
import math
import os
import socket
import sys
import time

BODY = int(os.environ.get("TWIN_PORT", 7801))
SOCKET = os.environ.get("TWIN_ROBOT_SOCKET")


def body():
    with socket.create_connection(("127.0.0.1", BODY), timeout=6) as sock:
        stream = sock.makefile("rw")
        stream.write('{"op":"hello","protocol":1,"joints":15}\n')
        stream.flush()
        stream.readline()
        stream.write('{"op":"read"}\n')
        stream.flush()
        return json.loads(stream.readline())


class Driver:
    """One connection, held.

    A socket per command is a connection setup every ten milliseconds, and the duck barely
    moved: the first version of this probe reported 0.16 m where a held connection gets
    well over a metre. `robotd`'s deadman wants the command to keep arriving, not to keep
    being reintroduced.
    """

    def __init__(self, path):
        self.file = None
        if not path:
            return
        try:
            sock = socket.socket(socket.AF_UNIX)
            sock.settimeout(3)
            sock.connect(path)
            self.file = sock.makefile("w")
        except OSError:
            self.file = None

    def move(self, vx):
        if not self.file:
            return
        try:
            self.file.write(json.dumps({
                "jsonrpc": "2.0", "method": "robot.move",
                "params": {"vx": vx, "vy": 0.0, "vyaw": 0.0}}) + "\n")
            self.file.flush()
        except OSError:
            self.file = None


def main():
    reading = body()
    gravity = reading["imu"]["gravity"][2]
    # A duck on its feet or its wheels reads -1. On its side it reads near zero, and on its
    # face the weight moves into x.
    print("UPRIGHT" if gravity < -0.9 else f"FALLEN (gravity z {gravity:+.2f})")

    driver = Driver(SOCKET)
    start = reading["trunk"][:2]
    end = time.time() + 6
    while time.time() < end:
        driver.move(0.4)
        time.sleep(0.1)
    moved = math.dist(start, body()["trunk"][:2])
    print(f"{'MOVED' if moved > 0.15 else 'STILL'} {moved:.2f} m in 6 s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
