# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Check what Isaac Teleop reads from the 01 Glove plugin.

Reads both hands through ``HandTracker`` (what every retargeter sees) and the
plugin's joint angles through ``JointStateTracker``, for ``--seconds``, then
reports per hand: how often a new pose arrived, whether the 26 joints form a
plausible OpenXR hand (each bone's -Z toward the next joint, the thumb on the
correct side, hand length), and the joint-angle stream. Exits non-zero when a
check fails.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

import numpy as np

import isaaccapture.deviceio as deviceio
import isaaccapture.oxr as oxr
from isaaccapture.cloudxr import CloudXRLauncher

log = logging.getLogger("isaaccapture.examples.pandora_glove.check_hands")

# OpenXR joint chains: metacarpal (or thumb metacarpal) to tip.
CHAINS = [list(range(2, 6))] + [
    list(range(start, start + 5)) for start in (6, 11, 16, 21)
]
JOINT_NAMES = [
    f"{finger}_{joint}"
    for finger, joints in [
        ("thumb", ("cmc_flexion", "mcp_flexion", "ip_flexion", "cmc_abduction")),
        *[
            (f, ("mcp_flexion", "pip_flexion", "dip_flexion", "mcp_abduction"))
            for f in ("index", "middle", "ring", "little")
        ],
    ]
    for joint in joints
]


def joints_of(hand) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Positions (26x3), orientations as xyzw (26x4) and validity (26) of a HandPose."""
    pos, rot, valid = np.zeros((26, 3)), np.zeros((26, 4)), np.zeros(26, dtype=bool)
    for i in range(26):
        joint = hand.joints.poses(i)
        p, q = joint.pose.position, joint.pose.orientation
        pos[i] = (p.x, p.y, p.z)
        rot[i] = (q.x, q.y, q.z, q.w)
        valid[i] = joint.is_valid
    return pos, rot, valid


def axis(q: np.ndarray, which: int) -> np.ndarray:
    x, y, z, w = q
    return [
        np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)]),
        np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)]),
        np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]),
    ][which]


def unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def shape_problems(pos: np.ndarray, rot: np.ndarray, is_left: bool) -> list[str]:
    problems = []
    for chain in CHAINS:
        for a, b in zip(chain, chain[1:]):
            if -axis(rot[a], 2) @ unit(pos[b] - pos[a]) < 0.95:
                problems.append(f"joint {a}: -Z does not point at joint {b}")
    toward_thumb = unit(pos[7] - pos[22])
    x = axis(rot[1], 0) @ toward_thumb
    if (x < 0.5) if is_left else (x > -0.5):
        problems.append(f"wrist: thumb on the wrong side (x·thumb = {x:+.2f})")
    # Wrist to middle fingertip along the finger, the same open or curled.
    path = [1, 11, 12, 13, 14, 15]
    length = sum(np.linalg.norm(pos[b] - pos[a]) for a, b in zip(path, path[1:]))
    if not 0.12 < length < 0.30:
        problems.append(f"wrist to middle tip along the finger {length:.3f} m")
    return problems


class HandStats:
    def __init__(self, name: str):
        self.name = name
        self.samples = 0
        self.active = 0
        self.changes = 0
        self.last: np.ndarray | None = None
        self.problems: dict[str, int] = {}
        self.checked = 0
        self.wrist: np.ndarray | None = None
        self.curl_range = [np.inf, -np.inf]

    def add(self, hand, is_left: bool, check_shape: bool) -> None:
        self.samples += 1
        if hand is None:
            return
        pos, rot, valid = joints_of(hand)
        if not valid.all():
            return
        self.active += 1
        if self.last is None or not np.array_equal(pos, self.last):
            self.changes += 1
            self.last = pos
        self.wrist = pos[1]
        # Index tip height over the wrist plane tracks the curl.
        tip = (pos[10] - pos[1]) @ axis(rot[1], 1)
        self.curl_range = [min(self.curl_range[0], tip), max(self.curl_range[1], tip)]
        if check_shape:
            self.checked += 1
            for problem in shape_problems(pos, rot, is_left):
                self.problems[problem] = self.problems.get(problem, 0) + 1


class JointStats:
    def __init__(self, collection: str):
        self.tracker = deviceio.JointStateTracker(collection)
        self.collection = collection
        self.changes = 0
        self.last = None
        self.names: list[str] = []
        self.max_abs = 0.0
        self.device = ""

    def add(self, session) -> None:
        tracked = self.tracker.get_data(session)
        data = getattr(tracked, "data", tracked)
        if data is None:
            return
        values = []
        names = []
        for joint in data.joints:
            name = joint.name.decode() if isinstance(joint.name, bytes) else joint.name
            names.append(name)
            values.append(float(joint.position))
        device = (
            data.device_id.decode()
            if isinstance(data.device_id, bytes)
            else data.device_id
        )
        self.device = device or ""
        self.names = names
        self.max_abs = max(self.max_abs, max((abs(v) for v in values), default=0.0))
        if values != self.last:
            self.changes += 1
            self.last = values


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument(
        "--expect-rate",
        type=float,
        default=0.0,
        help="fail when a hand updates slower than 90%% of this many times a second",
    )
    parser.add_argument(
        "--joints",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="also check the joint-angle collections",
    )
    CloudXRLauncher.add_launcher_arguments(parser)
    args = parser.parse_args()

    hand_tracker = deviceio.HandTracker()
    joint_stats = (
        [
            JointStats("pandora_glove_joints_left"),
            JointStats("pandora_glove_joints_right"),
        ]
        if args.joints
        else []
    )
    trackers = [hand_tracker] + [j.tracker for j in joint_stats]
    stats = [HandStats("left"), HandStats("right")]

    with CloudXRLauncher.launch_context(args):
        extensions = deviceio.DeviceIOSession.get_required_extensions(trackers)
        with oxr.OpenXRSession("PandoraGloveCheck", extensions) as oxr_session:
            with deviceio.DeviceIOSession.run(
                trackers, oxr_session.get_handles()
            ) as session:
                start = time.monotonic()
                next_shape = start
                while (now := time.monotonic()) - start < args.seconds:
                    session.update()
                    check_shape = now >= next_shape
                    if check_shape:
                        next_shape = now + 0.1
                    stats[0].add(hand_tracker.get_left_hand(session), True, check_shape)
                    stats[1].add(
                        hand_tracker.get_right_hand(session), False, check_shape
                    )
                    for joints in joint_stats:
                        joints.add(session)
                    time.sleep(0.0005)
                elapsed = time.monotonic() - start

    failed = False
    print(
        f"\n01 Glove check over {elapsed:.1f} s ({stats[0].samples / elapsed:.0f} polls/s)"
    )
    for s in stats:
        rate = s.changes / elapsed
        print(
            f"\n{s.name} hand: active {100 * s.active / max(s.samples, 1):.0f}% of polls, "
            f"{rate:.0f} new poses/s, {s.checked} shape checks"
        )
        if s.wrist is not None:
            print(
                f"  wrist at ({s.wrist[0]:+.3f}, {s.wrist[1]:+.3f}, {s.wrist[2]:+.3f}) m; "
                f"index tip height over the palm {s.curl_range[0]:+.3f} to {s.curl_range[1]:+.3f} m"
            )
        if s.active == 0:
            print("  FAIL: never active")
            failed = True
            continue
        for problem, count in sorted(s.problems.items(), key=lambda kv: -kv[1])[:8]:
            print(f"  FAIL: {problem} ({count} of {s.checked} checks)")
            failed = True
        if args.expect_rate and rate < 0.9 * args.expect_rate:
            print(
                f"  FAIL: {rate:.0f} new poses/s, expected about {args.expect_rate:.0f}"
            )
            failed = True
        if not s.problems:
            print(
                "  ok: every bone's -Z points at the next joint, thumb on the correct side, plausible size"
            )
    for j in joint_stats:
        rate = j.changes / elapsed
        print(
            f"\n{j.collection}: {j.device or 'no data'}, {rate:.0f} new samples/s, "
            f"{len(j.names)} joints, largest |angle| {j.max_abs:.2f} rad"
        )
        if j.changes == 0:
            print("  FAIL: no samples")
            failed = True
        elif j.names != JOINT_NAMES:
            print(f"  FAIL: joint names {j.names}")
            failed = True
        elif j.max_abs > 3.2:
            print("  FAIL: an angle beyond pi")
            failed = True
        else:
            print("  ok: 20 named joints in radians")
    print("\nRESULT:", "FAIL" if failed else "PASS")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
