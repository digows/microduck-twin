"""The duck, seen from outside.

The spec said this would be fifteen joint angles a tick and a `three.js` scene in the
browser. It is a rendered frame instead, and the reason is worth writing down: rebuilding
the duck in a browser means parsing the MJCF for its body tree, loading eighty-six STLs and
reproducing the kinematic chain — a project of its own — and what it would produce is a
reconstruction. This is the geometry MuJoCo is actually stepping, with the scene, the
contacts and the room around it.

Measured at 480x360 on an M1: about 20 ms a frame, 15 KB as JPEG. At ten frames a second
that is a fifth of a core — the same trade `DUCK_SIM_CAMERAS` already exists to let someone
decline, and the console asks for frames rather than being sent them.

**One thread owns the renderer, and every request is handed to it.**

MuJoCo's renderer holds a GL context belonging to the thread that made it. The body server
answers each connection on its own thread, so the first `view` built a context on one and
the second reached for it from another — which does not fail, it *hangs*, taking the socket
with it and leaving a duck that answers `read` and nothing else. `camera.py` upstream says
the same thing in one line: the renderer is not thread-safe and only the frame loop touches
it. Here the frame loop is this module's, and callers wait on a result.
"""

from __future__ import annotations

import base64
import io
import queue
import threading

import mujoco
import numpy as np


class _Request:
    __slots__ = ("world", "at", "size", "camera", "quality", "done", "jpeg", "error")

    def __init__(self, world, at, size, camera, quality):
        self.world = world
        self.at = at
        self.size = size
        self.camera = camera
        self.quality = quality
        self.done = threading.Event()
        self.jpeg = ""
        self.error = None


_requests: queue.Queue[_Request] = queue.Queue(maxsize=8)
_started = threading.Event()


def _serve() -> None:
    renderers: dict[tuple[int, int], mujoco.Renderer] = {}
    import PIL.Image

    while True:
        request = _requests.get()
        try:
            width, height = request.size
            key = (width, height)
            if key not in renderers:
                renderers[key] = mujoco.Renderer(
                    request.world.model, height=height, width=width
                )
            renderer = renderers[key]

            camera = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(camera)
            camera.lookat[:] = request.at
            camera.distance, camera.azimuth, camera.elevation = request.camera

            # `update_scene` reads the whole of MjData and the step loop writes it, so the
            # scene copy is taken under the world's lock — a millisecond — and the render,
            # which is twenty and touches nothing shared, is not.
            with request.world.lock:
                renderer.update_scene(request.world.data, camera)
            pixels = np.asarray(renderer.render())

            buffer = io.BytesIO()
            PIL.Image.fromarray(pixels).save(buffer, "JPEG", quality=request.quality)
            request.jpeg = base64.b64encode(buffer.getvalue()).decode("ascii")
        except Exception as error:  # a bad frame must not take the renderer down with it
            request.error = str(error)
        finally:
            request.done.set()


def frame(world, at, width=480, height=360, distance=2.2, azimuth=130.0, elevation=-55.0,
          quality=70, timeout=5.0) -> str:
    """One JPEG of the world, centred on `at`, base64 for a JSON line."""
    if not _started.is_set():
        _started.set()
        threading.Thread(target=_serve, name="twin-view", daemon=True).start()

    request = _Request(world, at, (int(width), int(height)),
                       (float(distance), float(azimuth), float(elevation)), int(quality))
    try:
        _requests.put_nowait(request)
    except queue.Full:
        # A console asking faster than the renderer can answer gets the frame it already
        # has, which is a dropped frame rather than a growing queue of stale ones.
        return ""
    if not request.done.wait(timeout):
        return ""
    if request.error:
        raise RuntimeError(request.error)
    return request.jpeg
