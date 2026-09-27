#!/usr/bin/env python
"""A wall between two ducks has to change what one hears from the other.

This is the one thing a two-duck bring-up cannot show: `build_world` spaces ducks half a
metre apart, which puts them in the same room of any scene worth having, and line of sight
is then trivially clear. So the ray is tested directly, against the real apartment, at the
height a duck's head actually sits.

    TWIN_MICRODUCK_RL=<checkout> <checkout>/.venv/bin/python twin-body/tests/occlusion.py
    …/python twin-body/tests/occlusion.py --map     # re-probe, when the apartment changes

The coordinates below were measured with `--map`, not guessed: the first attempt picked two
points "a metre apart in the open" that turned out to have a wall between them, because a
six-room flat in 7×6 m has a wall about every two metres. When upstream changes the
apartment this file should fail, and `--map` is how to pick new ones.
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from mjlab_microduck.sim.body_server import World  # noqa: E402

from twin_body.handler import TwinHandler  # noqa: E402

# The head sits here, and the flat's walls are 1.6 m, so a ray at head height is inside a
# wall rather than over it. At floor level it would pass along a threshold and prove nothing.
HEAD_Z = 0.23


def load():
    rl = os.environ.get("TWIN_MICRODUCK_RL")
    if not rl:
        sys.exit("set TWIN_MICRODUCK_RL to a microduck_rl checkout")
    scene = Path(rl) / "src/mjlab_microduck/robot/microduck/scene_apartment.xml"
    world = World(scene, 1)
    mujoco.mj_forward(world.model, world.data)
    return world


def blocked(world, here, there):
    return TwinHandler._blocked(world, np.array(here, float), np.array(there, float))


def draw_map(world):
    print(f"line of sight from the origin at z={HEAD_Z} — '.' clear, '#' blocked")
    for y in np.arange(2.5, -2.6, -0.5):
        row = "".join(
            "#" if blocked(world, (0, 0, HEAD_Z), (x, y, HEAD_Z)) else "."
            for x in np.arange(-3.5, 3.6, 0.5)
        )
        print(f"  y={y:+.1f}  {row}")
    print("  x from -3.5 to +3.5 in 0.5 m steps")


def main():
    world = load()
    if "--map" in sys.argv:
        draw_map(world)
        return 0

    failures = 0

    def check(label, here, there, want):
        nonlocal failures
        got = blocked(world, here, there)
        if got == want:
            print(f"  ok    {label}")
        else:
            print(f"  FAIL  {label}: wanted blocked={want}, got {got} — try --map")
            failures += 1

    print("the apartment, at head height")
    # The corridor runs north-south through the origin; east of it is a wall.
    check("a metre up the corridor", (0, 0, HEAD_Z), (0, 1.0, HEAD_Z), False)
    check("half a metre west", (0, 0, HEAD_Z), (-0.5, 0, HEAD_Z), False)
    check("two metres east, through the wall", (0, 0, HEAD_Z), (2.0, 0, HEAD_Z), True)
    check("across the whole flat", (-1.5, 0, HEAD_Z), (1.5, 0, HEAD_Z), True)

    print()
    print("the degenerate cases mj_ray must never see")
    # A zero-length direction makes `mj_ray` abort the process, so the guard that keeps one
    # from reaching it is worth a test of its own.
    check("a duck hears itself from where it stands", (0, 0, HEAD_Z), (0, 0, HEAD_Z), False)
    check("closer than twice the margin", (0, 0, HEAD_Z), (0.05, 0, HEAD_Z), False)

    print()
    if failures:
        print(f"{failures} check(s) failed")
        return 1
    print("a wall is a wall")
    return 0


if __name__ == "__main__":
    sys.exit(main())
