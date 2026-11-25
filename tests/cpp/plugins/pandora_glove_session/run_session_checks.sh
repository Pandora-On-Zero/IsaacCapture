#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# The plugin's session checks against the stand-in receiver:
#   run_session_checks.sh <pandora_glove_session_check> <golden recordings dir>
set -uo pipefail

CHECK="$1"
GOLDEN="$2"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
FAKE="$ROOT/examples/pandora_glove/python/isaaccapture_examples/pandora_glove/fake_receiver.py"
RECORDINGS=("$GOLDEN/right_recorded.json" "$GOLDEN/left_bare_recorded.json")
WORK="$(mktemp -d)"
LINK="$WORK/rx"
PROFILES="$WORK/profiles"
FAKE_PID=""
FAILED=()

start_fake() {
    rm -f "$LINK"
    python3 -I "$FAKE" "${RECORDINGS[@]}" --link "$LINK" --profiles-dir "$PROFILES" \
        --command-log "$WORK/commands.jsonl" "$@" >> "$WORK/fake.log" 2>&1 &
    FAKE_PID=$!
    for _ in $(seq 50); do
        [[ -e "$LINK" ]] && return 0
        sleep 0.1
    done
    echo "the stand-in receiver did not start:" >&2
    cat "$WORK/fake.log" >&2
    exit 1
}

stop_fake() {
    [[ -n "$FAKE_PID" ]] && kill "$FAKE_PID" 2>/dev/null
    wait "$FAKE_PID" 2>/dev/null
    FAKE_PID=""
}
trap 'stop_fake; rm -rf "$WORK"' EXIT

run() {
    local name="$1"
    shift
    echo
    echo "=================== $name"
    if ! "$CHECK" --port="$LINK" --profiles-dir="$PROFILES" --glove-profiles=off "$@"; then
        FAILED+=("$name")
    fi
}

# 1-3: push rates, every wrist mode
start_fake --rate 200
run "90 Hz, fixed wrist" --seconds=6 --expect-rate=90 --rate=90 --wrist=fixed --datasets=hand,joints
run "200 Hz, IMU wrist" --seconds=6 --expect-rate=200 --rate=200 --wrist=imu --datasets=hand,joints
# No offset stored: the glove's zero offset places the hand root on the controller itself.
run "every frame, controller wrist" --seconds=6 --expect-rate=200 --rate=max --wrist=controller --datasets=hand,joints \
    --expect-offset=0,0,0,0,0,0
stop_fake

# 4: haptics: scripted commands through HapticCommandReaderTracker to the receiver
: > "$WORK/commands.jsonl"
start_fake --rate 200
run "haptics" --seconds=4 --rate=200 --wrist=fixed
stop_fake
echo
echo "=================== haptics at the receiver"
if ! python3 -I - "$WORK/commands.jsonl" <<'EOF'; then
import json, sys
rtp = [json.loads(line) for line in open(sys.argv[1]) if '"rtp"' in line]
failed = False
for device in sorted({r["device"] for r in rtp}):
    seen = [r["amplitudes"] for r in rtp if r["device"] == device]
    order = []
    for amps in seen:
        lit = [i for i, a in enumerate(amps) if a]
        key = lit[0] if len(lit) == 1 and amps[lit[0]] == 76 else ("stop" if not lit else tuple(amps))
        if not order or order[-1] != key:
            order.append(key)
    holds = sorted({max(r["durations_ms"]) for r in rtp if r["device"] == device and any(r["amplitudes"])})
    print(f"{device}: {len(seen)} frames, motors in order {order}, holds {holds} ms")
    if order != [0, 1, 2, 3, 4, 5, "stop"]:
        print("  FAIL: expected each motor 0..5 at 76 in turn, then a stop")
        failed = True
if len({r["device"] for r in rtp}) != 2:
    print("  FAIL: expected both gloves")
    failed = True
print("RESULT:", "FAIL" if failed else "PASS")
sys.exit(1 if failed else 0)
EOF
    FAILED+=("haptics at the receiver")
fi

# 5: one glove goes quiet, then comes back
start_fake --rate 200 --silence 1:2.5:4.5
run "left glove silent 2.5-4.5 s" --seconds=7 --rate=200 --wrist=fixed --datasets=hand,joints
stop_fake

# 6: the receiver goes away and comes back
start_fake --rate 200
( sleep 2.5; kill "$FAKE_PID" 2>/dev/null; sleep 2; python3 -I "$FAKE" "${RECORDINGS[@]}" --link "$LINK" \
    --profiles-dir "$PROFILES" --rate 200 >> "$WORK/fake.log" 2>&1 & echo $! > "$WORK/second.pid" ) &
run "receiver unplugged 2.5-4.5 s" --seconds=8 --rate=200 --wrist=fixed --datasets=hand,joints
kill "$(cat "$WORK/second.pid" 2>/dev/null)" 2>/dev/null
stop_fake

# 7-8: the hand placed on a tracker through the glove's stored offset, held through a loss
OFFSET="-2,-3.5,5,0,-15,0"
start_fake --rate 200 --tracker-offset="$OFFSET"
run "controller wrist with the glove's offset, tracker lost 2-3.5 s" --seconds=5 --rate=200 --wrist=controller \
    --datasets=hand,joints --tracker-loss=2:3.5 --expect-offset="$OFFSET"
run "SE3 tracker wrist with the glove's offset, tracker lost 2-2.5 s" --seconds=4 --rate=200 \
    --wrist=se3:fake_tracker_left,fake_tracker_right --datasets=hand,joints --tracker-loss=2:2.5 \
    --expect-offset="$OFFSET"
stop_fake

echo
if [[ ${#FAILED[@]} -gt 0 ]]; then
    printf 'FAILED: %s\n' "${FAILED[@]}"
    exit 1
fi
echo "ALL SESSION CHECKS PASSED"
