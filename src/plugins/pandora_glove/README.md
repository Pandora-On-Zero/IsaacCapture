<!--
SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# 01 Glove plugin

Drives Isaac Teleop from Pandora-On-Zero's 01 Glove: both hands as OpenXR hand
tracking, each glove's 20 joint angles, and Isaac's haptic commands on the
glove's six motors.

```text
01 Glove --BLE--> receiver --USB serial--> libpandora (in this process)
   calibration, finger pipeline, 31-bone skeleton
        |
        v
pandora_glove_plugin
   bones -> 26 OpenXR joints on a wrist pose  -> HandInjector (hand tracking)
   joint angles                               -> pandora_glove_joints_{left,right}
   HapticCommand on pandora_glove_haptic      -> glove motors
```

The plugin owns the receiver; nothing else (the 01 Glove app, `pandora watch
--local`) may have it open at the same time.

## Build

Needs libpandora, the 01 Glove's C library: a `pandora-c-X.Y.Z-<platform>`
package from a `core-rs-vX.Y.Z` release (Linux x86_64 and aarch64), extracted.

```bash
cmake -B build -DBUILD_PLUGIN_PANDORA_GLOVE=ON -DPANDORA_C_DIR=/path/to/pandora-c
cmake --build build --parallel && cmake --install build
```

The plugin installs to `install/plugins/pandora_glove/` with `libpandora.so`
beside it. Let it open the receiver without root:

```bash
install/plugins/pandora_glove/install_udev_rules.sh
```

## Calibrate first

The plugin reads a glove's calibration; it never writes one. For each glove it
uses, in order:

1. `<glove id>.json` in the profiles folder (`--profiles-dir`, default
   `~/.pandora/profiles`), written by `pandora calibrate --hand left|right`
   (`pip install pandora-sdk`), which runs headless on the robot computer.
2. Otherwise the calibration stored on the glove, saved by the 01 Glove app or
   Pandora-Web (`--glove-profiles=off` turns this off).

A glove with neither sends no frames; the device status names it. Stop the
plugin before calibrating through the receiver: only one program can hold it.

## Run

Through a `TeleopSession`:

```python
PluginConfig(plugin_name="pandora_glove_plugin", plugin_root_id="pandora_glove",
             plugin_args=["--rate=200"])
```

Or by hand, after `source ~/.cloudxr/run/cloudxr.env`:

```bash
install/plugins/pandora_glove/pandora_glove_plugin --rate=200
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `--rate=HZ\|max` | `90` | Frames pushed per hand per second, 1 to 1000; `max` pushes every frame the glove sends |
| `--wrist=MODE` | `auto` | Where the hands are: `auto` (headset hand tracking, else a controller), `hand_tracking`, `controller`, `fixed` (both hands at a table in front of the origin, for tests) or `imu` (fixed positions, turned by the glove's own wrist IMU) |
| `--left-aim-to-wrist=px,py,pz,qx,qy,qz,qw` | identity | Controller aim pose to wrist, for a controller mounted on the glove; measure it for your mount |
| `--right-aim-to-wrist=...` | identity | The same for the right hand |
| `--port=PATH` | the receiver plugged in | The receiver's serial port |
| `--profiles-dir=DIR` | `~/.pandora/profiles` | Where `pandora calibrate` wrote profiles |
| `--glove-profiles=on\|off` | `on` | Use the calibration stored on a glove that has no profile file |
| `--datasets=hand,haptic,joints` | all | What to run |
| `--haptic-collection=ID` | `pandora_glove_haptic` | The haptic collection to read |
| `--haptic-hold-ms=MS` | `60` | How long the glove holds each haptic level; refreshed while a level stays |
| `--stale-ms=MS` | `200` | A hand with no new frame for this long goes inactive |

The glove has no position of its own: in `auto` and `controller` modes the
hand is placed on a Quest controller fixed to the glove, or on the headset's
optical hand tracking.

## Data

**Hand tracking.** 26 joints per hand through `plugin_utils::HandInjector`:
OpenVR's 25 hand bones map one to one onto OpenXR's joints 1 to 25, and the
palm is the centre of the middle metacarpal. Bones are re-expressed in the
OpenXR joint basis (-Z toward the fingertip, +Y out of the back of the hand).
A frame is pushed when the glove sends a new one, at most `--rate` a second.

**Joint angles.** `JointStateOutput` on `pandora_glove_joints_left` and
`pandora_glove_joints_right` (tensor `joint_state`), one sample per glove
frame, in radians: per finger three flexions from the knuckle outward, then the
abduction at the base (`thumb_cmc_flexion`, `thumb_mcp_flexion`,
`thumb_ip_flexion`, `thumb_cmc_abduction`, `index_mcp_flexion`, ...,
`little_mcp_abduction`). `device_id` is the glove's id.

**Haptics.** `HapticCommand` on `--haptic-collection`, endpoints `left` and
`right`: five values thumb to pinky (Isaac's `FingerPowerVector`, as
`haptic_glove_device(collection_id)` sends), or six with the back of the hand
last (`haptic_glove_device(collection_id, num_fingers=6)`), each 0 to 1. A
command older than four holds is treated as silence and the motors stop.

**Device status.** With `--plugin-root-id`, `/hand/left` and `/hand/right` are
published as connected, degraded (no current frame), disconnected (no receiver
or no glove, naming uncalibrated gloves) or failed (firmware libpandora cannot
read).

## Test without a glove

`examples/pandora_glove/` has a stand-in receiver that replays libpandora's
recorded gloves through a pseudo-terminal, so libpandora, the plugin and
Isaac run exactly as with hardware, plus a checker and a haptic sweep. See its
README.

## Troubleshooting

- **`receiver not open (no_receiver: ...)`**: nothing plugged in, or the udev
  rule is missing (`open_failed: Permission denied`). The plugin retries every
  second.
- **A hand never appears**: the glove has no calibration (see the device
  status), or it is not switched on and paired with the receiver.
- **`Failed to create OpenXR instance`**: CloudXR is not running, or
  `cloudxr.env` was not sourced. CloudXR needs a native Linux NVIDIA driver; it
  does not start under WSL2.
