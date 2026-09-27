#!/bin/sh
# The codec's contract with the daemons, checked without a sound card.
#
# Two things are being tested and they are kept apart on purpose. **What the argv meant** is
# a contract with `robotd` and `sounds`: the same on every machine, and a regression here
# makes a theremin play at the wrong rate in a way nobody traces back to a flag. **Which
# player answers** depends on what the host has installed, so a runner with no ffmpeg is
# allowed to resolve to nothing — that is a duck with no codec, which is a robot that walks
# identically and stays quiet.
#
# `TWIN_CODEC_DRY_RUN` prints both, one per line, instead of playing.

set -eu

here=$(cd "$(dirname "$0")" && pwd)
codec=$(dirname "$here")
failures=0

say_fail() {
    printf '  FAIL  %s\n        wanted %s\n        got    %s\n' "$1" "$2" "$3"
    failures=$((failures + 1))
}

# What the argv meant.
parse() {
    label=$1
    want=$2
    shift 2
    got=$(TWIN_CODEC_DRY_RUN=1 "$codec/aplay" "$@" | sed -n 's/^parse //p')
    if [ "$got" = "$want" ]; then
        printf '  ok    %s\n' "$label"
    else
        say_fail "$label" "$want" "$got"
    fi
}

# Which player answered, if any.
resolved() {
    TWIN_CODEC_DRY_RUN=1 "$codec/aplay" "$@" | sed -n 's/^exec //p'
}

echo "the argv the daemons send"
parse "a wav from the bank — robotd/src/sound.rs:305" \
    "raw=0 rate=48000 channels=1 file=/tmp/greet_a.wav" \
    -q -D default /tmp/greet_a.wav
parse "raw PCM on stdin — robotd/src/sound.rs:343" \
    "raw=1 rate=48000 channels=1 file=" \
    -q -D default -t raw -f S16_LE -c 1 -r 48000 --buffer-time=40000 --period-time=10000
parse "the rate survives — robotd/src/sound.rs:742 passes its own" \
    "raw=1 rate=16000 channels=1 file=" \
    -q -D plughw:aic3104 -t raw -f S16_LE -c 1 -r 16000
parse "the sounds CLI leaves -D out — sounds/src/main.rs:170" \
    "raw=1 rate=48000 channels=1 file=" \
    -q -t raw -f S16_LE -c 1 -r 48000
parse "an unknown flag is ignored rather than refused" \
    "raw=1 rate=44100 channels=1 file=" \
    -q -D default -t raw -f S16_LE -c 1 -r 44100 --some-future-alsa-flag
parse "a device is never mistaken for a file" \
    "raw=0 rate=48000 channels=1 file=" \
    -q -D plughw:aic3104,0

echo
echo "the player it resolves to"

# We are called `aplay` and sit ahead of the system's on PATH on purpose. Resolving with
# `command -v aplay` would find this script, which would find it again until the stack gave
# out, so the guard that walks PATH and skips our own directory gets a real test — with our
# directory actually on PATH, which is the only way it can fail.
PATH="$codec:$PATH"
export PATH
self=$(resolved -q -D default /tmp/greet_a.wav)
case "$self" in
    *"$codec/aplay"*) say_fail "it never execs itself" "anything but $codec/aplay" "$self" ;;
    *)                printf '  ok    it never execs itself\n' ;;
esac

# A file with no player is silence, not a player invoked with an empty argument.
empty=$(resolved -q -D default)
if [ -z "$empty" ]; then
    printf '  ok    nothing to play is a no-op\n'
else
    say_fail "nothing to play is a no-op" "" "$empty"
fi

# Where a player does exist, it has to be a real one. Both halves of this are reported
# rather than asserted, because which players a host has is not this project's business.
for shape in "wav:-q -D default /tmp/greet_a.wav" "raw:-q -D default -t raw -f S16_LE -c 1 -r 48000"; do
    kind=${shape%%:*}
    # shellcheck disable=SC2086  # the argv is the point; it has to split
    player=$(resolved ${shape#*:})
    if [ -n "$player" ]; then
        printf '  note  %s plays through %s\n' "$kind" "${player%% *}"
    else
        printf '  note  %s has no player on this host — the duck is quiet\n' "$kind"
    fi
done

echo
if [ "$failures" -gt 0 ]; then
    printf '%s check(s) failed\n' "$failures"
    exit 1
fi
printf 'the codec answers the daemons\n'
