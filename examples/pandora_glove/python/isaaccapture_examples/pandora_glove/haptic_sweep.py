# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Buzz each 01 Glove motor in turn through Isaac Teleop's haptic path.

Pushes ``HapticCommand`` frames the way a ``HapticSink`` does (Isaac's
``haptic_glove_device``) on the plugin's collection: thumb, index, middle,
ring, pinky, then the back of the hand, on both gloves, at ``--level``. The
plugin turns each frame into the glove's drive levels; with the stand-in
receiver they show in its log and ``--command-log``.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

import isaaccapture.deviceio as deviceio
import isaaccapture.oxr as oxr
from isaaccapture.cloudxr import CloudXRLauncher
from isaaccapture.haptic_devices.glove import haptic_glove_device

MOTORS = ("thumb", "index", "middle", "ring", "pinky", "back of hand")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--collection-id",
        default="pandora_glove_haptic",
        help="the plugin's --haptic-collection",
    )
    parser.add_argument(
        "--level", type=float, default=0.6, help="power for each motor in turn, 0 to 1"
    )
    parser.add_argument("--step", type=float, default=0.5, help="seconds per motor")
    parser.add_argument("--rounds", type=int, default=1)
    CloudXRLauncher.add_launcher_arguments(parser)
    args = parser.parse_args()

    device = haptic_glove_device(args.collection_id, num_fingers=len(MOTORS))
    tracker = device.get_tracker()
    with CloudXRLauncher.launch_context(args):
        extensions = deviceio.DeviceIOSession.get_required_extensions([tracker])
        with oxr.OpenXRSession("PandoraGloveHapticSweep", extensions) as oxr_session:
            with deviceio.DeviceIOSession.run(
                [tracker], oxr_session.get_handles()
            ) as session:
                for _ in range(args.rounds):
                    for motor, name in enumerate(MOTORS):
                        print(f"{name}: {args.level:.2f}")
                        powers = np.zeros(len(MOTORS), dtype=np.float32)
                        powers[motor] = args.level
                        end = time.monotonic() + args.step
                        while time.monotonic() < end:
                            session.update()
                            for endpoint in device.endpoints():
                                device.apply(endpoint, powers)
                            device.flush(session)
                            time.sleep(1 / 60)
                silence = np.zeros(len(MOTORS), dtype=np.float32)
                for _ in range(10):
                    session.update()
                    for endpoint in device.endpoints():
                        device.apply(endpoint, silence)
                    device.flush(session)
                    time.sleep(1 / 60)
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
