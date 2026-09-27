"""The simulator's body, with the parts a twin needs and upstream leaves constant.

`duck-body` in `microduck_rl` is the body every daemon talks to, and it is not forked here.
This package subclasses two of its classes and rebinds the names its `main()` looks up, which
is the whole of the override: `Body` gains a battery that drains and servos that warm, and
`Handler` gains the geometry an acoustic field needs to know who can hear whom.
"""
