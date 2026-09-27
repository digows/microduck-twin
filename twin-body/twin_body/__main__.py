"""Run `duck-body` with the twin's classes in place of its own.

`body_server.main()` looks `Body` and `Handler` up as module globals when it builds each
duck, so rebinding the two names is the entire override. No file in `microduck_rl` is
touched, and a version bump there is a pinned-dependency change rather than a merge.

    python -m twin_body --port 7801 --ducks 2 --scene .../scene_apartment.xml

Every argument is `duck-body`'s; this adds none of its own.
"""

from mjlab_microduck.sim import body_server

from twin_body.body import TwinBody
from twin_body.handler import TwinHandler

body_server.Body = TwinBody
body_server.Handler = TwinHandler

body_server.main()
