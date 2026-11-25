#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# 01 Glove test environment: the stand-in receiver replaying recordings (or a
# real receiver), the CloudXR service, the plugin, and the DeviceIO live view
# in the browser. Run from the IsaacCapture repository root after building
# with -DBUILD_PLUGIN_PANDORA_GLOVE=ON and installing the wheel into .venv.
#
#   examples/pandora_glove/run_test_env.sh --recordings DIR [options] [--accept-eula]
#   examples/pandora_glove/run_test_env.sh --real [options] [--accept-eula]
#
# Options: --rate HZ (default 200), --wrist fixed|imu|auto|controller|hand_tracking
# (default imu), --plugin-arg ARG (repeatable), --logs DIR (default /tmp/pandora_glove).
set -euo pipefail

RECORDINGS=""
REAL=0
RATE=200
WRIST=imu
LOGS=/tmp/pandora_glove
EULA=()
PLUGIN_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --recordings) RECORDINGS="$2"; shift 2 ;;
        --real) REAL=1; shift ;;
        --rate) RATE="$2"; shift 2 ;;
        --wrist) WRIST="$2"; shift 2 ;;
        --logs) LOGS="$2"; shift 2 ;;
        --plugin-arg) PLUGIN_ARGS+=("$2"); shift 2 ;;
        --accept-eula) EULA=(--accept-eula); shift ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done

PLUGIN=install/plugins/pandora_glove/pandora_glove_plugin
EXAMPLE=examples/pandora_glove/python/isaaccapture_examples/pandora_glove
if [[ ! -x "$PLUGIN" ]]; then
    echo "no $PLUGIN: build with -DBUILD_PLUGIN_PANDORA_GLOVE=ON and run cmake --install build" >&2
    exit 1
fi
if [[ $REAL -eq 0 && -z "$RECORDINGS" ]]; then
    echo "give --recordings DIR (libpandora golden files) or --real" >&2
    exit 2
fi
# shellcheck disable=SC1091
source .venv/bin/activate
mkdir -p "$LOGS"
PIDS=()
cleanup() {
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "stopped the plugin and receiver; the CloudXR service keeps running" \
         "(python -m isaaccapture.cloudxr.service stop)"
}
trap cleanup EXIT

if [[ $REAL -eq 0 ]]; then
    shopt -s nullglob
    files=()
    for name in right_recorded left_bare_recorded; do
        [[ -f "$RECORDINGS/$name.json" ]] && files+=("$RECORDINGS/$name.json")
    done
    [[ ${#files[@]} -gt 0 ]] || files=("$RECORDINGS"/*.json)
    python "$EXAMPLE/fake_receiver.py" "${files[@]}" --rate "$RATE" \
        --command-log "$LOGS/commands.jsonl" > "$LOGS/fake_receiver.log" 2>&1 &
    PIDS+=($!)
    PLUGIN_ARGS+=(--port=/tmp/pandora-fake-rx --profiles-dir=/tmp/pandora-fake-profiles --glove-profiles=off)
    sleep 1
fi

python -m isaaccapture.cloudxr.service start "${EULA[@]}"
# shellcheck disable=SC1090
source ~/.cloudxr/run/cloudxr.env

"$PLUGIN" --rate="$RATE" --wrist="$WRIST" "${PLUGIN_ARGS[@]}" > "$LOGS/plugin.log" 2>&1 &
PIDS+=($!)
echo "plugin log: $LOGS/plugin.log; receiver log: $LOGS/fake_receiver.log; haptics: $LOGS/commands.jsonl"

python -m isaaccapture_examples.deviceio_live_view "${EULA[@]}" --host 127.0.0.1
