#!/usr/bin/env python3
"""The console: one page that shows the duck and lets you act on it.

It speaks three things and invents none of them. To `robotd` it is JSON-RPC 2.0 over the
unix socket, newline-delimited, exactly as `robotctl` and `scripts/duck-sim drive` do. To
the body it is the TCP protocol every daemon uses, including the two ops `twin-body` adds.
To the field it is a speaker port, which is how a sound gets into the room from outside.

The official console is served alongside and untouched, for anyone who wants the page a
robot serves. This is the superset: the duck seen from outside, every sensor including the
two upstream reports as constants, the microphone, and the things you can do *to* a duck
rather than with it — speak to it, scratch its head, shove it.

    twin-console/server.py --port 8090 --robot-socket state/duck-a.sock --body-port 7801
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))


class Robot:
    """One daemon's socket, held open.

    One connection and one lock, because a daemon answers in order on a stream and two
    callers interleaving their lines would each read the other's answer.

    **There is no broker.** Each service owns its socket and answers only what is its own:
    `robotd` refuses `system.info` with "not served by robotd", because the robot's name and
    identity are `configd`'s. `robotctl` has a flag per socket for the same reason, and the
    console routes by method prefix rather than pretending one daemon knows everything.
    """

    def __init__(self, path: str):
        self.path = path
        self.lock = threading.Lock()
        self.sock = None
        self.file = None
        self.next_id = 1

    def _connect(self):
        if self.sock is None:
            sock = socket.socket(socket.AF_UNIX)
            sock.settimeout(5)
            sock.connect(self.path)
            self.sock, self.file = sock, sock.makefile("rw")

    def _drop(self):
        try:
            if self.sock:
                self.sock.close()
        finally:
            self.sock = self.file = None

    def notify(self, method: str, params: dict | None = None) -> None:
        """Fire and forget — what `robot.move` is, fifty times a second on a robot."""
        with self.lock:
            try:
                self._connect()
                self.file.write(
                    json.dumps({"jsonrpc": "2.0", "method": method, "params": params or {}}) + "\n"
                )
                self.file.flush()
            except OSError:
                self._drop()

    def call(self, method: str, params: dict | None = None):
        with self.lock:
            try:
                self._connect()
                self.next_id += 1
                self.file.write(
                    json.dumps(
                        {
                            "jsonrpc": "2.0",
                            "id": self.next_id,
                            "method": method,
                            "params": params or {},
                        }
                    )
                    + "\n"
                )
                self.file.flush()
                line = self.file.readline()
                if not line:
                    raise OSError("robotd closed")
                answer = json.loads(line)
                # **The error is the answer.** Returning `result` and nothing else turned
                # "unknown field `name`, expected `skill`" into a silent `null`, which is
                # the hardest kind of wrong to find: the console looked like it was working.
                if "error" in answer:
                    return {"error": answer["error"]}
                return answer.get("result")
            except (OSError, ValueError) as error:
                self._drop()
                return {"error": str(error)}


class Body:
    """The simulator's own port, with the ops `twin-body` adds."""

    def __init__(self, port: int):
        self.port = port
        self.lock = threading.Lock()
        self.sock = None
        self.file = None

    def _connect(self):
        if self.sock is None:
            sock = socket.create_connection(("127.0.0.1", self.port), timeout=10)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock, self.file = sock, sock.makefile("rw")
            self.file.write('{"op":"hello","protocol":1,"joints":15}\n')
            self.file.flush()
            self.file.readline()

    def ask(self, op: str, **kw):
        with self.lock:
            try:
                self._connect()
                self.file.write(json.dumps({"op": op, **kw}) + "\n")
                self.file.flush()
                return json.loads(self.file.readline())
            except (OSError, ValueError) as error:
                try:
                    if self.sock:
                        self.sock.close()
                finally:
                    self.sock = self.file = None
                return {"error": str(error)}


class Ear:
    """What the duck is hearing, as a number a meter can draw.

    The console does not carry the audio itself — `microduck-twin listen` does that, and a
    browser is a poor place to put a 16 kHz stream nobody asked for. What it shows is the
    level, which is what tells you a quack arrived.
    """

    def __init__(self, port: int):
        self.port = port
        self.dbfs = -120.0
        self.peak = -120.0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                sock = socket.create_connection(("127.0.0.1", self.port), timeout=5)
            except OSError:
                time.sleep(1)
                continue
            carry = b""
            while True:
                try:
                    chunk = sock.recv(4096)
                except OSError:
                    break
                if not chunk:
                    break
                chunk = carry + chunk
                if len(chunk) % 2:
                    chunk, carry = chunk[:-1], chunk[-1:]
                else:
                    carry = b""
                count = len(chunk) // 2
                if not count:
                    continue
                samples = struct.unpack(f"<{count}h", chunk)
                rms = math.sqrt(sum(s * s for s in samples) / count) / 32768.0
                self.dbfs = 20 * math.log10(max(rms, 1e-9))
                # Decays rather than latching, so the meter falls back after a sound — but
                # slowly enough to still be there when the page next asks. Reads arrive
                # about a hundred times a second, so 3 dB each was 300 dB a second and a
                # quack that had plainly been heard showed as silence half a second later.
                self.peak = max(self.dbfs, self.peak - 0.25)
            sock.close()


def speak(port: int, samples: list[int]) -> None:
    """Put a sound into the room at a duck's head, through its speaker port."""
    try:
        sock = socket.create_connection(("127.0.0.1", port), timeout=2)
    except OSError:
        return
    sock.sendall(b"".join(struct.pack("<h", int(s)) for s in samples))
    sock.close()


def scratch(seconds: float = 0.6, rate: int = 48_000) -> list[int]:
    """A hand on the duck's head, as a sound.

    `pet-detect` classifies 40-band log-mel windows, so what it needs is broadband noise
    with the shape of a scratch rather than a tone: bursts rather than a hiss, because a
    hand moves. Whether the classifier calls it petting is its business and the point of
    running the real one.
    """
    import random

    count = int(rate * seconds)
    out = []
    envelope = 0.0
    for i in range(count):
        if i % (rate // 14) == 0:
            envelope = 1.0
        envelope *= 0.9997
        out.append(int(random.uniform(-1, 1) * 9000 * envelope))
    return out


# Which daemon owns which namespace. Taken from `robotctl`'s own flags, which is the only
# place this is written down.
OWNERS = {
    "system.": "config", "net.": "config", "pad.": "config",
    "update.": "updater",
    "tof.": "tof",
}


class Console(BaseHTTPRequestHandler):
    robot: Robot
    daemons: dict
    body: Body
    ear: Ear
    speaker_port: int

    @classmethod
    def owner(cls, method: str) -> Robot:
        for prefix, name in OWNERS.items():
            if method.startswith(prefix):
                return cls.daemons.get(name, cls.robot)
        return cls.robot

    def log_message(self, *args):  # the access log is noise on a console
        pass

    def _send(self, code: int, body: bytes, kind: str):
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload, code=200):
        self._send(code, json.dumps(payload).encode(), "application/json")

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as page:
                return self._send(200, page.read(), "text/html; charset=utf-8")
        if path == "/state":
            return self._json(self.state())
        if path == "/view":
            answer = self.body.ask("view", **self.query_numbers())
            jpeg = answer.get("jpeg") or ""
            if not jpeg:
                return self._json({"error": answer.get("error", "no frame")}, 503)
            return self._send(200, base64.b64decode(jpeg), "image/jpeg")
        return self._json({"error": "no such thing here"}, 404)

    def query_numbers(self) -> dict:
        if "?" not in self.path:
            return {}
        out = {}
        for pair in self.path.split("?", 1)[1].split("&"):
            if "=" not in pair:
                continue
            key, value = pair.split("=", 1)
            try:
                out[key] = float(value) if "." in value else int(value)
            except ValueError:
                pass
        return out

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            payload = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            return self._json({"error": "not json"}, 400)
        path = self.path.split("?")[0]

        if path == "/move":
            self.robot.notify("robot.move", {
                "vx": float(payload.get("vx", 0.0)),
                "vy": float(payload.get("vy", 0.0)),
                "vyaw": float(payload.get("vyaw", 0.0)),
            })
            return self._json({"ok": True})
        if path == "/look":
            self.robot.notify("robot.look", {
                "x": float(payload.get("x", 1.0)),
                "y": float(payload.get("y", 0.0)),
                "z": float(payload.get("z", 0.0)),
            })
            return self._json({"ok": True})
        if path == "/do":
            # `skill`, not `name`. The daemon says so — "unknown field `name`, expected
            # `skill`" — and said it into a response this console used to discard.
            return self._json(self.robot.call("robot.do", {"skill": payload.get("name", "")}))
        if path == "/rpc":
            method = payload.get("method", "")
            return self._json(self.owner(method).call(method, payload.get("params")))
        if path == "/push":
            return self._json(self.body.ask("push", vx=payload.get("vx", 0.7),
                                            vy=payload.get("vy", 0.0)))
        if path == "/roller":
            # **This restarts the twin, including this server.** Wheels are another robot
            # and another MJCF, and MuJoCo compiles its model — so the toggle is a bring-up,
            # not a switch. Detached and unwaited, because the reply has to leave before
            # `down` reaches the console's own pidfile; the page notices the gap and comes
            # back when the new one answers.
            import subprocess

            want = "on" if payload.get("on") else "off"
            twin = os.path.join(os.path.dirname(HERE), "microduck-twin")
            subprocess.Popen([twin, "roller", want], start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return self._json({"restarting": want})
        if path == "/pet":
            threading.Thread(target=speak, args=(self.speaker_port, scratch()),
                             daemon=True).start()
            return self._json({"ok": True})
        if path == "/say":
            # 48 kHz mono S16_LE, base64, straight into the room at this duck's head.
            raw = base64.b64decode(payload.get("pcm", ""))
            count = len(raw) // 2
            samples = list(struct.unpack(f"<{count}h", raw[: count * 2])) if count else []
            if samples:
                threading.Thread(target=speak, args=(self.speaker_port, samples),
                                 daemon=True).start()
            return self._json({"samples": len(samples)})
        return self._json({"error": "no such thing here"}, 404)

    def state(self) -> dict:
        read = self.body.ask("read")
        slow = self.body.ask("slow")
        tof = self.body.ask("tof")
        hears = self.body.ask("hears")
        health = self.robot.call("robot.health")
        return {
            "sim_time": read.get("sim_time"),
            "trunk": read.get("trunk"),
            "trunk_z": read.get("trunk_z"),
            "imu": read.get("imu"),
            "positions": read.get("positions"),
            "velocities": read.get("velocities"),
            "currents_ma": read.get("currents_ma"),
            "volts": slow.get("volts"),
            "temps_c": slow.get("temps_c"),
            "tof": tof.get("distance_mm"),
            "tof_status": tof.get("status"),
            "peers": hears.get("peers", []),
            "mode": (self.robot.call("robot.mode") or {}).get("mode"),
            "mic_dbfs": round(self.ear.dbfs, 1),
            "mic_peak": round(self.ear.peak, 1),
            "health": health,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--robot-socket", required=True)
    parser.add_argument("--config-socket")
    parser.add_argument("--updater-socket")
    parser.add_argument("--tof-socket")
    parser.add_argument("--body-port", type=int, default=7801)
    parser.add_argument("--audio-port", type=int, default=7951)
    parser.add_argument("--duck-index", type=int, default=0)
    args = parser.parse_args()

    Console.robot = Robot(args.robot_socket)
    Console.daemons = {
        name: Robot(path)
        for name, path in (("config", args.config_socket),
                           ("updater", args.updater_socket),
                           ("tof", args.tof_socket))
        if path
    }
    Console.body = Body(args.body_port)
    Console.speaker_port = args.audio_port + 2 * args.duck_index
    Console.ear = Ear(Console.speaker_port + 1)

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Console)
    print(f"== console on http://127.0.0.1:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
