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

# **Wheel bearing friction, set here because it cannot be set in the XML.**
#
# `scripts/infer_policy.py` says why in one line: non-zero `frictionloss` on a passive joint
# breaks training, so the roller MJCF ships its four wheels at zero and every consumer that
# actually rolls has to put it back. `duck-body` never did — it was written for the walking
# scenes, and knows nothing about a wheel — so a duck on wheels here stood on frictionless
# castors, went over on its face and had no standing policy to get up with.
WHEEL_FRICTIONLOSS = 0.003

# What the wheels add under the trunk. The roller keyframes carry the walking robot's
# heights, so a duck placed at one starts thirteen millimetres inside the floor.
WHEEL_RISE_M = 0.0135

# How long a duck on wheels is held after its torque comes on.
#
# `robotd` ramps to the home pose over two seconds before handing the robot to its policy —
# `HOME_RAMP` in `robotd/src/main.rs`, a constant rather than a setting — and during that
# ramp the joints are driven to a pose and nothing is balancing. A duck with feet is folded
# on the floor and does not mind. A duck on castors is an inverted pendulum on wheels, and
# it goes over in about a second: the log reads "ramping to home", then "the fall verdict
# changed fallen=true", then "the policy has the robot", which by then is lying down with no
# standing policy in roller mode to get it up.
#
# Measured, not argued: the same policy in `infer_policy.py` stands and glides on this
# machine with or without BAM and at either action scale, and falls exactly like this the
# moment two seconds of rigid home pose are inserted before it starts.
#
# So the body holds the duck through the ramp and lets go when the policy has it — which is
# what a hand does to a robot on wheels while it boots. The machinery is upstream's own: a
# body that has not been released is restored to where it was on every step.
WHEEL_HOLD_S = 2.5


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

        # The wheels, if this model has any. Done once per body and idempotent: the
        # model is shared, and writing the same number four times costs nothing.
        self.wheels = []
        for joint in range(world.model.njnt):
            name = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if name and name.startswith(self.prefix + "passive_") and "wheel" in name:
                self.wheels.append(joint)
                world.model.dof_frictionloss[world.model.jnt_dofadr[joint]] = WHEEL_FRICTIONLOSS
        if self.wheels:
            print(f"== duck {index}: on {len(self.wheels)} wheels — bearing friction "
                  f"{WHEEL_FRICTIONLOSS}, {WHEEL_RISE_M * 1000:.1f} mm of rise, held "
                  f"{WHEEL_HOLD_S}s at each power-on", flush=True)

        # The head, for whoever is listening. Absent on a model without the site rather than
        # fatal: a scene can carry a robot that predates it, and a duck with no located ear
        # is a duck the field leaves out — not a simulator that refuses to start.
        found = mujoco.mj_name2id(
            world.model, mujoco.mjtObj.mjOBJ_SITE, self.prefix + HEAD_SITE
        )
        self.head_site = found if found >= 0 else None

    # ── held until the policy has it ─────────────────────────────────────────

    @property
    def released(self) -> bool:
        """Whether the world may let this duck move.

        Upstream's own flag, with a delay in front of it for wheels. `World.step` restores
        any body that is not released, so returning False here is a hand under the duck.
        """
        until = getattr(self, "_hold_until", None)
        if until is not None and time.monotonic() < until:
            return False
        return getattr(self, "_released", False)

    @released.setter
    def released(self, value: bool) -> None:
        self._released = bool(value)

    def set_torque(self, on: bool) -> None:
        super().set_torque(on)
        # Only on wheels, and only when the torque is arriving: a duck with feet has
        # nothing to fall off, and one being switched off does not need holding.
        self._hold_until = time.monotonic() + WHEEL_HOLD_S if (on and self.wheels) else None

    def place(self, pose, trunk_z: float, offset_y: float) -> None:
        """Where a duck starts, raised by the height of its own wheels.

        The roller scene's keyframes are the walking robot's, so `STAND` puts the trunk at
        0.12 m — which is the right height for a duck standing on its feet and thirteen
        millimetres inside the floor for one standing on castors.
        """
        super().place(pose, trunk_z + (WHEEL_RISE_M if self.wheels else 0.0), offset_y)

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
