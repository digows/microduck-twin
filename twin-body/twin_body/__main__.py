"""Run `duck-body` with the twin's classes in place of its own.

`body_server.main()` looks `Body` and `Handler` up as module globals when it builds each
duck, so rebinding the two names is the entire override. No file in `microduck_rl` is
touched, and a version bump there is a pinned-dependency change rather than a merge.

    python -m twin_body --port 7801 --ducks 2 --scene .../scene_apartment.xml

Every argument is `duck-body`'s; this adds none of its own. What it reads instead is the
environment: `TWIN_ROBOT_SOCKET` lets the window drive the daemon, and `TWIN_AUDIO_PORT`
gives it a microphone level to draw. Neither is set on a robot.
"""

from mjlab_microduck.sim import body_server

from twin_body import hud
from twin_body.body import TwinBody
from twin_body.handler import TwinHandler

body_server.Body = TwinBody
body_server.Handler = TwinHandler

# `run` is where the world finally exists and where the viewer is opened, and it is also
# fifty lines of carefully paced real-time loop that must not be forked to add an overlay.
# So it is wrapped rather than replaced: take the world on the way past, install the HUD's
# `launch_passive` wrapper, and hand the call straight on.
_run = body_server.run


def run(world, headless: bool) -> None:
    if not headless:
        hud.install(world, world.bodies)
    return _run(world, headless)


body_server.run = run

body_server.main()
