#!/bin/sh
# Wheels on, wheels off, and the duck is healthy in both.
#
# The daemon distinguishes the two sets itself — `roller.onnx` in the walk slot, no standing
# policy, the crouch in the ground-pick slot — so what this checks is that the twin asks for
# the right one and comes up behind it. It is slow, because it is two bring-ups: the wheels
# are a different robot in a different MJCF and MuJoCo compiles its model, so the world goes
# away and comes back both times.
#
#     TWIN_TEST_ROLLER=1 ./microduck-twin test
#     tests/roller.sh                              # or on its own

set -eu

HERE=$(cd "$(dirname "$0")/.." && pwd)
twin="$HERE/microduck-twin"
failures=0

check() {
    label=$1
    want=$2
    got=$3
    case "$got" in
        *"$want"*) printf '  ok    %s\n' "$label" ;;
        *)
            printf '  FAIL  %s\n        wanted %s\n        got    %s\n' "$label" "$want" "$got"
            failures=$((failures + 1))
            ;;
    esac
}

echo "wheels on"
"$twin" roller on > /dev/null 2>&1
sleep 6
policies=$("$twin" ctl policy list 2>&1 || true)
health=$("$twin" ctl health 2>&1 || true)
check "the daemon says roller"        "mode: roller"   "$policies"
check "the walk slot holds roller"    "roller.onnx"    "$policies"
check "the crouch is the ground pick" "roller_crouch"  "$policies"
# Left unnamed these resolve into the release directory, which exists on a robot and not
# here — and the duck comes up unhealthy over a file nobody asked for.
check "a duck on wheels does not kick" "switched off"  "$policies"
check "and it is healthy"             "healthy"        "$health"

echo
echo "wheels off"
"$twin" roller off > /dev/null 2>&1
sleep 6
policies=$("$twin" ctl policy list 2>&1 || true)
health=$("$twin" ctl health 2>&1 || true)
check "the daemon says walk"          "mode: walk"        "$policies"
check "the walk slot holds walking"   "alpha_walking"     "$policies"
check "the kicks are back"            "ball_kick_left"    "$policies"
check "and it is healthy"             "healthy"           "$health"

echo
if [ "$failures" -gt 0 ]; then
    printf '%s check(s) failed\n' "$failures"
    exit 1
fi
printf 'the wheels go on and come off\n'
