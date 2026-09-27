"""A battery that empties and servos that warm, in place of two constants.

`duck-body` reports 7.4 V and 32 °C for every duck for ever, which is honest about being a
stand-in and useless to anything above it. `robotd` already implements the rest: it maps
volts to a percentage linearly between `BATTERY_FULL_V` (8.2) and `BATTERY_EMPTY_V` (6.6),
and it powers the robot down when the average reaches empty. So the only thing missing on
this side is a voltage that moves.

**Both models are shapes, not fits.** No datasheet for the pack or the servos went into
them, and they are not evidence about how long a real duck runs. What they buy is that the
number responds to what the robot is doing, so anything above — an agent deciding to go and
charge, a test for the power-down path — has something real to read.
"""

from __future__ import annotations

# `duck_control::model`, which is where the daemon's percentage comes from.
FULL_V = 8.2
EMPTY_V = 6.6

# What the board draws with the servos idle: SoC, camera, sensors, radios.
IDLE_MA = 450.0

# What a servo draws per newton-metre it is holding.
#
# **Not the body's `currents_ma`.** That one is `|torque| * 100`, and upstream says plainly
# that it is a stand-in with the right shape rather than amps — a consumer watching load
# sees load, and nothing more is claimed. A battery cannot use it: at that scale fifteen
# servos holding a stand draw ninety milliamps and the duck runs for three hours, which is
# wrong by about a factor of six and wrong in the flattering direction.
#
# This is the XL330's stall current over its stall torque, near enough. Both numbers read
# the same torque; only this one is pretending to be amps.
SERVO_MA_PER_NM = 3000.0

# Sized so a duck holding a stand runs for something like half an hour, which is the order a
# 2S pack in an 800 g robot is worth. Measured against a standing duck rather than guessed,
# and still a shape: no pack datasheet went into it. Override with TWIN_BATTERY_MAH.
CAPACITY_MAH = 1500.0

# Internal resistance, in ohms, for the sag under load. A pack this small sags visibly when
# fifteen servos catch a fall, and a voltage that only falls slowly hides that.
PACK_OHMS = 0.12


class Battery:
    """Charge drawn, and the voltage that follows from it."""

    def __init__(self, capacity_mah: float = CAPACITY_MAH, charge: float = 1.0):
        self.capacity_mah = capacity_mah
        self.charge = charge          # 0..1
        self.drawn_mah = 0.0

    def drain(self, servo_ma: float, dt: float) -> None:
        total_ma = IDLE_MA + servo_ma
        self.drawn_mah += total_ma * dt / 3600.0
        self.charge = max(0.0, 1.0 - self.drawn_mah / self.capacity_mah)

    def volts(self, servo_ma: float) -> float:
        # Open-circuit voltage linear in charge, because the daemon's percentage is linear in
        # voltage: a curve here would make `robotctl health` report a percentage that does not
        # match the charge, which is a lie that costs more than the realism buys.
        open_circuit = EMPTY_V + (FULL_V - EMPTY_V) * self.charge
        sag = PACK_OHMS * (IDLE_MA + servo_ma) / 1000.0
        return max(0.0, open_circuit - sag)


# Where a servo sits with no load, and where the old constant was.
AMBIENT_C = 24.0
RESTING_C = 32.0

# First-order: each servo heats with the square of its torque, because the loss that warms a
# motor is I²R and current follows torque, and cools toward ambient. The gain is set so the
# hips holding a stand settle around 40 °C — warm, and near the 32 °C the constant used to
# report, which is the one point upstream had an opinion about.
#
# The square means a joint held at peak torque for ever would cook, which is true of the
# real servo too; what saves both is that peak torque is a burst. A roulade adds about four
# degrees and then the joint cools for forty-five seconds.
HEAT_GAIN = 25.0
COOL_TAU_S = 45.0

# Where a servo gives up. The XL330 shuts itself down around here, and a model with no
# ceiling reports temperatures that no consumer has a sensible response to.
MAX_C = 85.0


class Thermals:
    """One temperature per joint, rising with load."""

    def __init__(self, count: int):
        self.temps = [RESTING_C] * count

    def step(self, forces, dt: float) -> None:
        for i, force in enumerate(forces):
            if i >= len(self.temps):
                break
            heat = HEAT_GAIN * float(force) * float(force)
            cool = (self.temps[i] - AMBIENT_C) / COOL_TAU_S
            self.temps[i] = min(MAX_C, self.temps[i] + (heat - cool) * dt)
