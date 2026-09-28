"""The console, in the window the duck is already in.

There were two surfaces: a MuJoCo window with no numbers on it, and a browser page with all
of them. Two places to look is the thing this project keeps saying it does not want, and
MuJoCo's passive viewer turns out to be able to host the numbers — `set_texts` puts two
columns in each corner and `set_images` puts a bitmap at a viewport. So the window is where
the duck is watched and where its numbers live.

**Every letter is already taken, and taken back.** MuJoCo's own viewer binds all of A-Z to
a visualisation flag — `W` is wireframe, `D` hides static bodies and blacks the room out —
and a user `key_callback` is called *in addition to* that handling rather than instead of
it, from C++ that Python cannot reach.

What it cannot reach it can undo. Twenty of those letters toggle a flag in `mjvOption`,
which the handle exposes and which is writable, so a key can do its job here and have its
side effect put back in the same frame. The six that do not — `G K L R S W`, whose flags
live in the scene rather than the option — are left alone and unused. The map is read from
`mjVISSTRING` at run time rather than transcribed, so a MuJoCo that moves a flag moves this
with it.

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

# The tags `sounds` renders into every duck's bank, in the order V steps through them.
VOICES = ["chirp", "greet", "coo", "inquire", "peck", "alarm", "wheee"]


def recoverable_keys() -> dict[str, int]:
    """Letter to `mjvOption` flag index, for the keys whose side effect can be undone.

    Read from `mjVISSTRING` rather than transcribed: it is the table simulate itself binds
    from, so a MuJoCo that moves a flag moves this with it. The render flags in
    `mjRNDSTRING` — G, K, L, R, S, W — live in the scene instead and are not reachable from
    the handle, which is why no key here is one of them.
    """
    return {
        row[2].upper(): index
        for index, row in enumerate(mujoco.mjVISSTRING)
        if row[2].strip() and row[2].upper().isalpha()
    }


VIS_KEYS = recoverable_keys()

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

    def call(self, method: str, params: dict | None = None) -> str | None:
        """The daemon's objection, or None when it did the thing.

        **A refusal is the answer.** Returning the result and nothing else turned "unknown
        field `name`, expected `skill`" into a silent success, and a duck that ignores a key
        with no word about why is the hardest kind of wrong to find. The daemon also refuses
        politely inside a result — `{"accepted": false, "reason": …}` — and that counts too.
        """
        if not self.path:
            return "no daemon socket"
        with self.lock:
            try:
                self._connect()
                self.file.write(
                    json.dumps({"jsonrpc": "2.0", "id": 1, "method": method,
                                "params": params or {}}) + "\n"
                )
                self.file.flush()
                answer = json.loads(self.file.readline())
            except (OSError, ValueError) as error:
                self._drop()
                return f"unreachable: {error}"
        if "error" in answer:
            return f"refused: {answer['error'].get('message', answer['error'])}"
        result = answer.get("result") or {}
        if isinstance(result, dict) and result.get("accepted") is False:
            return f"refused: {result.get('reason', 'no reason given')}"
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
        self.voice = 0
        audio = os.environ.get("TWIN_AUDIO_PORT")
        self.speaker_port = int(audio) if audio and audio.isdigit() else None
        # What the room looked like before anyone pressed anything. Every key this handles
        # is a letter MuJoCo binds to one of these, and every frame puts them back.
        self.flag_baseline = {
            index: bool(handle.opt.flags[index]) for index in VIS_KEYS.values()
        }
        threading.Thread(target=self._run, daemon=True).start()
        threading.Thread(target=self._drive, daemon=True).start()

    # ── the keys ─────────────────────────────────────────────────────────────

    # ── the keys ─────────────────────────────────────────────────────────────

    def key(self, code: int) -> None:
        """One keypress. GLFW codes; the arrows are 262-265.

        **A press walks.** An earlier version added a tenth of a metre a second per press,
        which is `infer_policy.py`'s convention for tuning a gait and is wrong for driving:
        it took four presses to go anywhere and the duck had already stopped by then,
        because the deadman zeroes an intent that stops arriving.
        """
        if code == 265:    # up
            self.velocity[0] = WALK; self.say("forward")
        elif code == 264:  # down
            self.velocity[0] = -WALK; self.say("back")
        elif code == 263:  # left
            self.velocity[2] = TURN; self.say("turning left")
        elif code == 262:  # right
            self.velocity[2] = -TURN; self.say("turning right")
        elif code == 32:   # space
            self.velocity = [0.0, 0.0, 0.0]
            self.daemon.notify("robot.stop", {})
            self.say("stopped")
        elif code == ord("A"):
            self.velocity[1] = STRAFE; self.say("stepping left")
        elif code == ord("D"):
            self.velocity[1] = -STRAFE; self.say("stepping right")
        elif code in (ord("I"), ord("M")):
            self.gaze = max(-0.4, min(0.4, self.gaze + (0.2 if code == ord("I") else -0.2)))
            self.daemon.notify("robot.look", {"x": 1.0, "y": 0.0, "z": self.gaze})
            self.say(f"looking at z {self.gaze:+.1f}")
        elif code == ord("Q"):
            self.skill_say(self.daemon.call("robot.sound", {"tag": VOICES[self.voice]}),
                           f"said {VOICES[self.voice]}")
        elif code == ord("V"):
            self.voice = (self.voice + 1) % len(VOICES)
            self.say(f"voice: {VOICES[self.voice]}")
        elif code in (ord("Y"), ord("E"), ord("C"), ord("N"), ord("B")):
            skill = {ord("Y"): "sit_toggle", ord("E"): "roulade", ord("C"): "ground_pick",
                     ord("N"): "kick_left", ord("B"): "kick_right"}[code]
            self.skill_say(self.daemon.call("robot.do", {"skill": skill}), skill)
        elif code == ord("T"):
            self.powered = not self.powered
            self.skill_say(self.daemon.call("robot.enable", {"on": self.powered}),
                           "torque on" if self.powered else "torque off")
        elif code == ord("H"):
            threading.Thread(target=self.scratch, daemon=True).start()
            self.say("scratching — pet-detect decides")
        elif code == ord("P"):
            self.push(); self.say("shoved")
        elif code == ord("Z"):
            self.respawn(); self.say("put back")
        elif code == ord("O"):
            self.wheels()

        # **Put the room back.** The viewer has already toggled whatever flag this letter
        # is bound to, in C++, before or after this runs — the order is not ours to know.
        # Flipping it here covers one case and the baseline enforced every frame covers the
        # other, so `D` drives the duck sideways instead of blacking the flat out.
        self.restore_flags()

    def restore_flags(self) -> None:
        for index, wanted in self.flag_baseline.items():
            if self.handle.opt.flags[index] != wanted:
                self.handle.opt.flags[index] = wanted

    def skill_say(self, refusal, done: str) -> None:
        """The daemon's own words when it refuses, rather than a cheerful lie."""
        self.say(refusal or done)

    def look_away(self) -> None:
        self.gaze = 0.0

    def push(self) -> None:
        body = self.bodies[0]
        with self.world.lock:
            self.world.data.qvel[body.trunk_dof + 0] = 0.7
            self.world.data.qvel[body.trunk_dof + 1] = 0.2

    def respawn(self) -> None:
        """Put the duck back where it started.

        `scene_apartment.xml` lays its own floors and has no ground plane — drive out of
        the flat and there is nothing under you. No `robotctl` verb brings a robot home,
        and rightly: a robot cannot be moved by asking. The body can.
        """
        body = self.bodies[0]
        with self.world.lock:
            data = self.world.data
            data.qpos[body.trunk + 0] = 0.0
            data.qpos[body.trunk + 1] = 0.0
            data.qpos[body.trunk + 2] = 0.20
            data.qpos[body.trunk + 3:body.trunk + 7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[body.trunk_dof:body.trunk_dof + 6] = 0.0

    def scratch(self) -> None:
        """A hand on the duck's head, as a sound at its own speaker.

        `pet-detect` classifies 40-band log-mel windows, so what it wants is broadband noise
        shaped like a scratch — bursts, because a hand moves — rather than a tone. Whether
        it calls that petting is its business, and running the real one is the point.
        """
        import random

        if not self.speaker_port:
            self.say("no field — nothing to scratch into")
            return
        rate = 48_000
        envelope = 0.0
        samples = bytearray()
        for i in range(int(rate * 0.6)):
            if i % (rate // 14) == 0:
                envelope = 1.0
            envelope *= 0.9997
            samples += struct.pack("<h", int(random.uniform(-1, 1) * 9000 * envelope))
        try:
            with socket.create_connection(("127.0.0.1", self.speaker_port), timeout=2) as s:
                s.sendall(bytes(samples))
        except OSError:
            pass

    def wheels(self) -> None:
        """Wheels on, or off.

        Another robot and another MJCF, which MuJoCo compiles — so this is a bring-up, and
        the window it is pressed in goes away and comes back with it. Detached and unwaited,
        because `down` reaches this process before the key has finished being handled.
        """
        import subprocess

        binary = os.environ.get("TWIN_BIN")
        state = os.environ.get("TWIN_STATE", "")
        if not binary:
            self.say("no twin binary in the environment")
            return
        here = "walk"
        try:
            with open(os.path.join(state, "mode")) as handle:
                here = handle.read().strip()
        except OSError:
            pass
        want = "off" if here == "roller" else "on"
        subprocess.Popen([binary, "roller", want], start_new_session=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.say(f"wheels {want} — the twin is coming back up")

    def _drive(self):
        """The command, resent while it is non-zero.

        `robotd`'s deadman zeroes an intent that stops arriving after 500 ms, which is the
        robot's own safety and not a thing to work around: a key sets a velocity and this
        keeps saying it until another key changes it.
        """
        while True:
            if any(self.velocity):
                self.daemon.notify("robot.move", {
                    "vx": self.velocity[0], "vy": self.velocity[1], "vyaw": self.velocity[2],
                })
            time.sleep(0.1)

    def say(self, text: str) -> None:
        self.said = text
        self.said_until = time.monotonic() + 2.5

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
                self.restore_flags()
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
            data = self.world.data
            trunk = [float(v) for v in data.qpos[body.trunk:body.trunk + 3]]
            sim_time = float(data.time)
            # What it is doing, not what it was told. The command lives in the terminal
            # now, and a panel that echoed it would be reporting the driver back to itself.
            velocity = [float(v) for v in data.qvel[body.trunk_dof:body.trunk_dof + 3]]
            yaw = float(data.qvel[body.trunk_dof + 5])
        speed = math.hypot(velocity[0], velocity[1])

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
        right_labels = "\n".join(["ducks", "ear height", "moving", "", "depth 8x8"])
        right_values = "\n".join([
            str(len(self.bodies)),
            "out of the flat — press Z" if lost else (f"{hears[0][2]:.2f} m" if hears else "unplaced"),
            f"{speed:+.2f} m/s  {yaw:+.2f} rad/s",
            "",
            "below: near is warm, dark is nothing",
        ])

        keys_labels = ("arrows\nA / D\nI / M\nspace\nQ / V\nY / E\nC / N / B\n"
                       "H / T / P\nZ / O")
        keys_values = ("walk and turn\nstep sideways\nlook up, down\nstop\n"
                       "quack, next voice\nsit, roulade\nground pick, kicks\n"
                       "scratch, torque, shove\nput back, wheels")
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
