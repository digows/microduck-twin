"""`Handler`, with the geometry an acoustic field needs.

The body protocol carries `trunk`, `trunk_z` and the trunk's IMU, and nothing about the
head. That is enough for a radio — `duck-ether` derives RSSI from where two bodies are — and
not enough for an ear: the microphone is on the head, the head turns, and a wall between two
ducks is the difference between hearing one and not.

So one op, `hears`, answers all of it in a single round trip: where this duck's ear is, and
for every other duck how far away it is and whether anything is in the way. One request per
audio tick rather than one per pair, and the ray casting stays where the model is. The field
knowing how to intersect MuJoCo geometry would be a second place for scale, ordering and
frames to live, which is the argument upstream already makes for reporting in the robot's
own units.

**Additive, so an older simulator degrades rather than breaks.** A `duck-body` that predates
this answers `unknown op`, and a field that gets that falls back to the trunk positions it
can already read — distance without direction, and no walls.
"""

from __future__ import annotations

import numpy as np
from mjlab_microduck.sim.body_server import Handler

# How far along the ray to start, and to stop short. Both ducks' own geometry sits at the
# ends of the line, and the nearest hit is otherwise the listener's own beak — which would
# report every duck as blocked by itself. Eight centimetres clears a head.
MARGIN_M = 0.08


class TwinHandler(Handler):
    def dispatch(self, body, request: dict) -> dict:
        op = request.get("op")
        if op == "hears":
            return self.hears(body)
        if op == "push":
            return self.push(body, request)
        if op == "place":
            return self.place(body)
        return super().dispatch(body, request)

    def push(self, body, request: dict) -> dict:
        """Shove the trunk, the way the training event does.

        Overwrites the world-frame velocity rather than adding to it, so holding the key
        does not accumulate into a launch. A metre a second is the cap the velstand push
        curriculum ends at, which is what the standing policy was trained to survive.
        """
        vx = float(request.get("vx", 0.7))
        vy = float(request.get("vy", 0.0))
        speed = (vx * vx + vy * vy) ** 0.5
        if speed > 1.0:
            vx, vy = vx / speed, vy / speed
        with body.world.lock:
            body.world.data.qvel[body.trunk_dof + 0] = vx
            body.world.data.qvel[body.trunk_dof + 1] = vy
        return {"vx": vx, "vy": vy}

    def place(self, body) -> dict:
        """Put the duck back where it started.

        `scene_apartment.xml` lays its own floors and has no ground plane — drive out of
        the flat and there is nothing under you. No `robotctl` verb brings a robot home,
        and rightly: a robot cannot be moved by asking. The body can, and this is the only
        op here that exists for a console rather than for a daemon.
        """
        from mjlab_microduck.sim.body_server import HOME_TRUNK_Z, SPACING

        # `Body.place` is what `main` uses to put a duck down in the first place: home pose,
        # upright, still, and the torque re-applied. Doing it by hand here set the trunk to
        # an arbitrary 0.20 m and left the joints in whatever shape the fall had left them,
        # so a relaxed duck simply dropped the twenty centimetres again. The height comes
        # from the body too, and `TwinBody.place` raises it by the wheels when there are any.
        with body.world.lock:
            body.place(None, HOME_TRUNK_Z, body.index * SPACING)
        return {"placed": True, "trunk_z": float(body.world.data.qpos[body.trunk + 2])}

    def hears(self, body) -> dict:
        world = body.world
        with world.lock:
            pose = body.head_pose()
            if pose is None:
                return {"mic": None, "peers": [], "sim_time": float(world.data.time)}
            here, xmat = pose

            peers = []
            for other in world.bodies:
                if other is body:
                    continue
                other_pose = other.head_pose() if hasattr(other, "head_pose") else None
                if other_pose is None:
                    continue
                there = other_pose[0]
                peers.append(
                    {
                        "index": other.index,
                        "pos": [float(v) for v in there],
                        "distance": float(np.linalg.norm(there - here)),
                        "blocked": self._blocked(world, here, there),
                    }
                )

            return {
                "mic": {
                    "pos": [float(v) for v in here],
                    # Row-major 3x3, the same shape MuJoCo stores it in. The field rotates a
                    # source into this frame to get a bearing.
                    "xmat": [float(v) for v in xmat.reshape(9)],
                },
                "peers": peers,
                "sim_time": float(world.data.time),
            }

    @staticmethod
    def _blocked(world, here, there) -> bool:
        """Is anything between these two points that is not either duck's own head?"""
        span = there - here
        length = float(np.linalg.norm(span))
        if length <= 2 * MARGIN_M:
            return False
        direction = span / length
        origin = here + direction * MARGIN_M
        reach = length - 2 * MARGIN_M

        # A zero-length direction makes `mj_ray` abort the whole process, and the length
        # check above is what keeps one from reaching it.
        geom = np.zeros(1, dtype=np.int32)
        hit = mujoco_ray(world, origin, direction, geom)
        return 0.0 <= hit < reach


def mujoco_ray(world, origin, direction, geom):
    import mujoco

    return mujoco.mj_ray(
        world.model,
        world.data,
        np.ascontiguousarray(origin, dtype=np.float64),
        np.ascontiguousarray(direction, dtype=np.float64),
        None,
        1,
        -1,
        geom,
    )
