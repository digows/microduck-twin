"""The console, in the window the duck is already in.

There were two surfaces: a MuJoCo window with no numbers on it, and a browser page with all
of them. Two places to look is the thing this project keeps saying it does not want, and
MuJoCo's passive viewer turns out to be able to host the numbers — `set_texts` puts two
columns in each corner, `set_images` puts a bitmap at a viewport, and `key_callback` takes
the keys. So the window is the console, and the page is what a window cannot be: reachable
from another machine, and able to hear you.

**Wrapped, not copied.** `body_server.run` owns the viewer and a carefully paced real-time
loop; taking a copy of that to add an overlay would be a fork of the part most worth not
forking. Instead `launch_passive` is wrapped, the handle is kept, and a thread writes to it
while their loop goes on calling `sync`.

The keys are the ones this ecosystem already uses — `infer_policy.py` has taught arrows for
velocity, `Y` for sit and `R` for roulade to everyone who has driven this robot.
"""

from __future__ import annotations

import json
import math
import os
import socket
import struct
import threading
import time

import mujoco
import numpy as np

HZ = 10.0

# What a key means. The walking policy was trained to about 0.3 m/s and the twin's own
# rehearsal script caps there; a turn of 0.8 rad/s is brisk without spinning the duck.
WALK = 0.3
STRAFE = 0.2
TURN = 0.8

# `mjtGridPos`, which is where a corner is.
TOPLEFT = mujoco.mjtGridPos.mjGRID_TOPLEFT
TOPRIGHT = mujoco.mjtGridPos.mjGRID_TOPRIGHT
BOTTOMLEFT = mujoco.mjtGridPos.mjGRID_BOTTOMLEFT
FONT = mujoco.mjtFontScale.mjFONTSCALE_150


class Daemon:
    """`robotd`'s socket, for a window that wants to drive.

    A body talking up to the daemon is an inversion, and it is deliberate: this is the
    twin's console rather than part of the robot. The socket is the same one `robotctl`
    uses, and nothing here is reachable from a robot.
    """

    def __init__(self, path: str | None):
        self.path = path
        self.lock = threading.Lock()
        self.sock = None
        self.file = None

    def _connect(self):
        if self.sock is None and self.path:
            sock = socket.socket(socket.AF_UNIX)
            sock.settimeout(2)
            sock.connect(self.path)
            self.sock, self.file = sock, sock.makefile("rw")

    def _drop(self):
        try:
            if self.sock:
                self.sock.close()
        finally:
            self.sock = self.file = None

    def notify(self, method: str, params: dict | None = None) -> None:
        if not self.path:
            return
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
        if not self.path:
            return None
        with self.lock:
            try:
                self._connect()
                self.file.write(
                    json.dumps({"jsonrpc": "2.0", "id": 1, "method": method,
                                "params": params or {}}) + "\n"
                )
                self.file.flush()
                return json.loads(self.file.readline()).get("result")
            except (OSError, ValueError):
                self._drop()
                return None


class Ear:
    """The level at this duck's microphone, for a meter in the corner."""

    def __init__(self, port: int | None):
        self.port = port
        self.dbfs = -120.0
        if port:
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
                carry = b""
                if len(chunk) % 2:
                    chunk, carry = chunk[:-1], chunk[-1:]
                count = len(chunk) // 2
                if not count:
                    continue
                samples = struct.unpack(f"<{count}h", chunk)
                rms = math.sqrt(sum(s * s for s in samples) / count) / 32768.0
                self.dbfs = 20 * math.log10(max(rms, 1e-9))
            sock.close()


def bar(value: float, low: float, high: float, width: int = 12) -> str:
    """A meter drawn in the only font a viewer overlay has.

    ASCII, deliberately. The overlay is rendered by MuJoCo's own bitmap font, and a block
    character that font does not carry comes out as a box or as nothing — and a meter made
    of boxes is worse than no meter, because it reads as a bug in the numbers beside it.
    """
    filled = int(round(max(0.0, min(1.0, (value - low) / (high - low))) * width))
    return "#" * filled + "." * (width - filled)


class Hud:
    def __init__(self, handle, world, bodies, daemon: Daemon, ear: Ear):
        self.handle = handle
        self.world = world
        self.bodies = bodies
        self.daemon = daemon
        self.ear = ear
        self.said = ""
        self.said_until = 0.0
        self.velocity = [0.0, 0.0, 0.0]
        self.gaze = 0.0
        self.powered = True
        threading.Thread(target=self._run, daemon=True).start()
        threading.Thread(target=self._drive, daemon=True).start()

    # ── the keys ─────────────────────────────────────────────────────────────

    def say(self, text: str) -> None:
        self.said = text
        self.said_until = time.monotonic() + 2.5

    def key(self, code: int) -> None:
        """One keypress. GLFW codes; the arrows are 262-265.

        **A press walks.** The first version added a tenth of a metre a second per press,
        which is `infer_policy.py`'s convention for tuning a gait and is wrong for a
        console: it took four presses to go anywhere, and the robot had already stopped by
        then because nothing was being sent in between. A key sets the command outright
        and the opposite key reverses it.
        """
        if code == 265:    # up
            self.velocity[0] = WALK; self.say("forward")
        elif code == 264:  # down
            self.velocity[0] = -WALK; self.say("back")
        elif code == 263:  # left
            self.velocity[2] = TURN; self.say("turning left")
        elif code == 262:  # right
            self.velocity[2] = -TURN; self.say("turning right")
        elif code == ord("A"):
            self.velocity[1] = STRAFE; self.say("stepping left")
        elif code == ord("D"):
            self.velocity[1] = -STRAFE; self.say("stepping right")
        elif code == 32:   # space
            self.velocity = [0.0, 0.0, 0.0]
            self.daemon.notify("robot.stop", {})
            self.say("stop")
        elif code == ord("W"):
            self.look(0.35); self.say("looking up")
        elif code == ord("S"):
            self.look(-0.35); self.say("looking down")
        elif code == ord("Q"):
            self.daemon.notify("robot.sound", {"tag": "chirp"}); self.say("quack")
        elif code == ord("Y"):
            self.daemon.call("robot.do", {"skill": "sit_toggle"}); self.say("sit / stand")
        elif code == ord("R"):
            self.daemon.call("robot.do", {"skill": "roulade"}); self.say("roulade")
        elif code == ord("G"):
            self.daemon.call("robot.do", {"skill": "ground_pick"}); self.say("ground pick")
        elif code == ord("K"):
            self.daemon.call("robot.do", {"skill": "kick_left"}); self.say("kick, left")
        elif code == ord("L"):
            self.daemon.call("robot.do", {"skill": "kick_right"}); self.say("kick, right")
        elif code == ord("T"):
            self.torque(); self.say("torque " + ("on" if self.powered else "off"))
        elif code == ord("P"):
            self.push(); self.say("shoved")
        elif code == ord("Z"):
            self.respawn(); self.say("put back")

    def look(self, height: float) -> None:
        """Point the head at a spot in the trunk frame: X forward, Z up, metres.

        The daemon runs the gaze IK itself, so this is a place to look rather than four
        joint angles — `1 0 0` is straight ahead.
        """
        self.gaze = max(-0.4, min(0.4, self.gaze + height))
        self.daemon.notify("robot.look", {"x": 1.0, "y": 0.0, "z": self.gaze})

    def torque(self) -> None:
        """Hand the robot to its policy, or take it back.

        `robot.enable` wants `{"on": …}` and says so — "missing field `on`" — rather than
        being two methods. `robot.relax` exists as well and is a different thing: it cuts
        power outright, where this is the policy's leash.
        """
        self.powered = not self.powered
        self.daemon.call("robot.enable", {"on": self.powered})

    def respawn(self) -> None:
        """Put the duck back where it started.

        `scene_apartment.xml` lays its own floors and has no ground plane — walk out of the
        flat and there is nothing there, which is the scene saying so rather than a bug.
        What follows is a duck a kilometre down with its servos saturated and its battery
        reading empty, and no key in `robotctl` brings it home: the daemon has no opinion
        about where a robot is in the world, because a robot cannot be moved by asking.
        The body can, and does here, because this is the twin's console.
        """
        body = self.bodies[0]
        with self.world.lock:
            data = self.world.data
            data.qpos[body.trunk + 0] = 0.0
            data.qpos[body.trunk + 1] = 0.0
            data.qpos[body.trunk + 2] = 0.20
            data.qpos[body.trunk + 3:body.trunk + 7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[body.trunk_dof:body.trunk_dof + 6] = 0.0

    def push(self) -> None:
        body = self.bodies[0]
        with self.world.lock:
            self.world.data.qvel[body.trunk_dof + 0] = 0.7
            self.world.data.qvel[body.trunk_dof + 1] = 0.2

    def _drive(self):
        """The velocity command, resent while it is non-zero.

        `robotd`'s deadman zeroes an intent that stops arriving after 500 ms, which is the
        robot's own safety and not something to work around: a key sets a velocity and this
        keeps saying it until a key sets it back.
        """
        while True:
            if any(self.velocity):
                self.daemon.notify("robot.move", {
                    "vx": self.velocity[0], "vy": self.velocity[1], "vyaw": self.velocity[2],
                })
            time.sleep(0.1)

    # ── the numbers ──────────────────────────────────────────────────────────

    def _run(self):
        # **Say why it stopped.** A daemon thread that swallows its exception and returns
        # leaves a window with no numbers on it and a log with no reason in it, which is
        # the same symptom as a HUD that was never installed.
        period = 1.0 / HZ
        while True:
            try:
                if not self.handle.is_running():
                    return
                self.draw()
            except Exception as error:
                print(f"== hud stopped: {type(error).__name__}: {error}", flush=True)
                return
            time.sleep(period)

    def draw(self):
        body = self.bodies[0]
        slow = body.slow_sensors()
        volts = slow["volts"]
        temps = slow["temps_c"]
        hears = None
        try:
            hears = body.head_pose()
        except Exception:
            pass

        with self.world.lock:
            trunk = [float(v) for v in self.world.data.qpos[body.trunk:body.trunk + 3]]
            sim_time = float(self.world.data.time)

        percent = max(0.0, min(100.0, (volts - 6.6) / 1.6 * 100.0))
        hottest = max(temps)
        mic = self.ear.dbfs

        left_labels = "\n".join(["battery", "motors", "microphone", "trunk", "sim"])
        left_values = "\n".join([
            f"{bar(volts, 6.6, 8.2)}  {volts:5.2f} V  {percent:3.0f}%",
            f"{bar(hottest, 24, 85)}  {hottest:5.1f} C max",
            f"{bar(mic, -60, 0)}  {mic:5.0f} dBFS",
            f"{trunk[0]:+.2f} {trunk[1]:+.2f} {trunk[2]:.2f} m",
            f"{sim_time:.0f} s",
        ])

        # The apartment has no ground plane, so a duck driven out of the flat falls for
        # ever — and every other number goes strange with it. Say which it is.
        lost = trunk[2] < -0.5
        right_labels = "\n".join(["ducks", "ear height", "command", "", "depth 8x8"])
        right_values = "\n".join([
            str(len(self.bodies)),
            "out of the flat — press Z" if lost else (f"{hears[0][2]:.2f} m" if hears else "unplaced"),
            f"vx {self.velocity[0]:+.2f}  vy {self.velocity[1]:+.2f}  vyaw {self.velocity[2]:+.2f}",
            "",
            "below: near is warm, dark is nothing",
        ])

        keys_labels = "arrows\nA / D\nW / S\nspace\nQ / Y\nR / G\nK / L\nT / P\nZ"
        keys_values = ("walk and turn\nstep sideways\nlook up, down\nstop\n"
                       "quack, sit\nroulade, ground pick\nkick left, right\n"
                       "torque, shove\nput the duck back")
        if time.monotonic() < self.said_until:
            keys_values = f"{keys_values}\n\n{self.said}"
            keys_labels = f"{keys_labels}\n\n>"

        self.handle.set_texts([
            (FONT, TOPLEFT, left_labels, left_values),
            (FONT, TOPRIGHT, right_labels, right_values),
            (FONT, BOTTOMLEFT, keys_labels, keys_values),
        ])
        self.handle.set_images(self.tof_image(body))

    def tof_image(self, body):
        """The 8x8 depth grid, as a bitmap in the corner.

        Near is warm and far is cold, and a zone with no target is left dark — the status
        byte is the difference between "nothing there" and "could not measure", and a
        picture that painted them the same would lose the thing the sensor is for.
        """
        with self.world.lock:
            distance, status = body.tof.frame(self.world.data)

        cell = 18
        image = np.zeros((8 * cell, 8 * cell, 3), dtype=np.uint8)
        for row in range(8):
            for col in range(8):
                index = row * 8 + col
                mm = distance[index]
                if status[index] != 5 or mm <= 0:
                    colour = (26, 28, 34)
                else:
                    near = max(0.0, 1.0 - mm / 2500.0)
                    colour = (int(40 + 215 * near), int(70 + 80 * near), int(200 - 150 * near))
                image[row * cell:(row + 1) * cell, col * cell:(col + 1) * cell] = colour

        # **Bottom right, out of the keys' way.** The first placement was a fixed rectangle
        # at the bottom left, which is where `set_texts` also puts the key list — so the
        # grid sat on top of the letters and read as a bug rather than a sensor. The
        # viewport is read each frame rather than assumed, because a window is resizable.
        size = 8 * cell
        margin = 16
        window = self.handle.viewport
        left = max(margin, window.width - size - margin)
        viewport = mujoco.MjrRect(left, margin, size, size)
        return [(viewport, image)]


def install(world, bodies) -> None:
    """Wrap `launch_passive` so the window comes up with the console on it."""
    import mujoco.viewer

    if getattr(mujoco.viewer, "_twin_hud", False):
        return
    original = mujoco.viewer.launch_passive

    daemon = Daemon(os.environ.get("TWIN_ROBOT_SOCKET"))
    audio = os.environ.get("TWIN_AUDIO_PORT")
    ear = Ear(int(audio) + 1 if audio and audio.isdigit() else None)

    def launch_passive(model, data, **kwargs):
        hud = {}

        def on_key(code):
            if "hud" in hud:
                hud["hud"].key(code)

        kwargs.setdefault("key_callback", on_key)
        handle = original(model, data, **kwargs)
        hud["hud"] = Hud(handle, world, bodies, daemon, ear)
        print("== hud: the console is on the window", flush=True)
        return handle

    mujoco.viewer.launch_passive = launch_passive
    mujoco.viewer._twin_hud = True
