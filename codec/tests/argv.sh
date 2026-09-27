#!/bin/sh
# The codec's contract with the daemons, checked without a sound card.
#
# Both argv shapes come verbatim from the callers, so this is a regression test on the
# parser rather than on the player: `TWIN_CODEC_DRY_RUN` prints the command that would have
# been exec'd. What it asserts is that the shapes are recognised and the numbers survive —
# a duck whose theremin plays at the wrong rate sounds broken in a way nobody traces back
# to argv.

set -eu

here=$(cd "$(dirname "$0")" && pwd)
codec=$(dirname "$here")
failures=0

check() {
    want=$1
    shift
    got=$(TWIN_CODEC_DRY_RUN=1 "$codec/aplay" "$@" || true)
    case "$got" in
        *"$want"*) printf '  ok    %s\n' "$want" ;;
        *)
            printf '  FAIL  wanted %s\n        got    %s\n        argv   %s\n' \
                "$want" "$got" "$*"
            failures=$((failures + 1))
            ;;
    esac
}

echo "playback, a wav from the bank — robotd/src/sound.rs:305"
check "greet_a.wav" -q -D default /tmp/greet_a.wav

echo "playback, raw PCM on stdin — robotd/src/sound.rs:343"
check "48000" -q -D default -t raw -f S16_LE -c 1 -r 48000 \
    --buffer-time=40000 --period-time=10000

echo "the rate survives, whatever it is — robotd/src/sound.rs:742 passes its own"
check "16000" -q -D plughw:aic3104 -t raw -f S16_LE -c 1 -r 16000

echo "the sounds CLI puts -D last, or leaves it out — sounds/src/main.rs:170"
check "48000" -q -t raw -f S16_LE -c 1 -r 48000

echo "an unknown flag is ignored rather than refused"
check "44100" -q -D default -t raw -f S16_LE -c 1 -r 44100 --some-future-alsa-flag

echo "nothing to play is a no-op, not a player with an empty argument"
empty=$(TWIN_CODEC_DRY_RUN=1 "$codec/aplay" -q -D default || true)
if [ -z "$empty" ]; then
    printf '  ok    silence\n'
else
    printf '  FAIL  wanted nothing\n        got    %s\n' "$empty"
    failures=$((failures + 1))
fi

if [ "$failures" -gt 0 ]; then
    printf '\n%s check(s) failed\n' "$failures"
    exit 1
fi
printf '\nthe codec answers the daemons\n'
