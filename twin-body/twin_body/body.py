"""`Body`, with a battery that empties and servos that warm."""

from __future__ import annotations

import os
import time

import mujoco
import numpy as np
from mjlab_microduck.sim.body_server import JOINT_NAMES, Body

from twin_body.power import SERVO_MA_PER_NM, Battery, Thermals

# Where the ear and the mouth are. The MJCF has no microphone site — it has `imu`,
# `head_camera`, `tof` and `head_imu` — so the field borrows one, and which one matters:
# a frame is not only a position.
#
# `head_imu` is the wrong borrow. Its axes are the BMI088's own, tilted, and the daemon
# says as much where it publishes them; measured here, its x points almost straight down.
# A field taking that for forward would put every source behind the duck's ear.
#
# `tof` is the right one. It is on the same part, centimetres from the Mic3R and the
# speaker, and it is the site the depth sensor already casts from with +x forward, +y left,
# +z up — a convention `tof.py` states and relies on.
HEAD_SITE = "tof"


class TwinBody(Body):
    """One duck, with the two quantities upstream reports as constants.

    The battery drains from the same `|torque| * 100` current the body already claims to
    draw, so the two numbers cannot disagree. The daemon does the rest: it maps volts to a
    percentage and powers the robot down at 6.6 V, by the code that runs on the robot.
    """

    def __init__(self, world, index: int, limp: bool = False, kp: float = 200.0):
        super().__init__(world, index, limp=limp, kp=kp)

        capacity = float(os.environ.get("TWIN_BATTERY_MAH", 0)) or None
        self.battery = Battery(capacity) if capacity else Battery()
        self.thermals = Thermals(len(JOINT_NAMES))
        self._last_slow = None

        # The head, for whoever is listening. Absent on a model without the site rather than
        # fatal: a scene can carry a robot that predates it, and a duck with no located ear
        # is a duck the field leaves out — not a simulator that refuses to start.
        found = mujoco.mj_name2id(
            world.model, mujoco.mjtObj.mjOBJ_SITE, self.prefix + HEAD_SITE
        )
        self.head_site = found if found >= 0 else None

    # ── what the daemon asks for slowly ──────────────────────────────────────

    def slow_sensors(self) -> dict:
        """Volts and per-joint temperature, advanced by however long since the last ask.

        Driven by the wall clock rather than by sim time, because `slow` is polled by the
        daemon on its own schedule and the world runs at 1.00x by design. A simulator that
        fell behind would drain its battery slower than the robot it is standing in for,
        which is the wrong way round for anything testing a power-down.
        """
        now = time.monotonic()
        dt = 0.0 if self._last_slow is None else min(now - self._last_slow, 5.0)
        self._last_slow = now

        with self.world.lock:
            forces = self.world.data.actuator_force[self.actuator_slice].copy()

        servo_ma = float(np.abs(forces).sum()) * SERVO_MA_PER_NM

        if dt > 0.0:
            self.battery.drain(servo_ma, dt)
            self.thermals.step(np.abs(forces), dt)

        temps = list(self.thermals.temps)
        # The wire is indexed by JOINT_NAMES, which has fifteen entries; the model drives
        # fourteen. The missing one is the mouth, and it reports ambient rather than nothing.
        wire = [self.thermals.temps[0]] * len(JOINT_NAMES)
        for slot, wire_index in enumerate(self.to_wire):
            if slot < len(temps):
                wire[wire_index] = round(temps[slot], 1)

        return {"volts": round(self.battery.volts(servo_ma), 3), "temps_c": wire}

    # ── geometry, for whoever is listening ───────────────────────────────────

    def head_pose(self):
        """The ear's position and orientation in the world, or None on a model without it."""
        if self.head_site is None:
            return None
        data = self.world.data
        return (
            data.site_xpos[self.head_site].copy(),
            data.site_xmat[self.head_site].reshape(3, 3).copy(),
        )
