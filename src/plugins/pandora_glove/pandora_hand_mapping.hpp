// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <openxr/openxr.h>

#include <array>

namespace plugins
{
namespace pandora_glove
{

// libpandora's skeleton: OpenVR's 31 hand bones, each relative to its parent,
// as x, y, z (metres) then qw, qx, qy, qz.
inline constexpr int kPandoraBoneCount = 31;
using PandoraBones = double[kPandoraBoneCount][7];

using HandJoints = std::array<XrHandJointLocationEXT, XR_HAND_JOINT_COUNT_EXT>;

// The 26 OpenXR joints of one hand, relative to its WRIST joint and in the
// XR_EXT_hand_tracking joint basis (-Z toward the fingertip, +Y out of the
// back of the hand). Flags are POSITION/ORIENTATION_VALID only; the caller
// places the hand with compose_on_wrist().
HandJoints map_bones_to_openxr(const PandoraBones& bones, bool is_left);

// libpandora's pose: x, y, z (metres), then qw, qx, qy, qz; OpenVR axes, which are OpenXR's.
using PandoraPose = std::array<double, 7>;

XrPosef from_pandora_pose(const PandoraPose& pose);
PandoraPose to_pandora_pose(const XrPosef& pose);

// `b` (a pose in `a`'s frame) composed onto `a`.
XrPosef compose_poses(const XrPosef& a, const XrPosef& b);

// The OpenXR WRIST joint's pose in the hand root's frame (libpandora's
// placement gives the root), in the same basis as map_bones_to_openxr.
XrPosef wrist_in_root(const PandoraBones& bones, bool is_left);

// Place wrist-relative joints at `wrist` (a pose in the session base space).
// TRACKED bits are added when `tracked` is true.
void compose_on_wrist(HandJoints& joints, const XrPosef& wrist, bool tracked);

// The glove's wrist IMU (pandora_hand_frame.wrist_deg: pitch, roll, yaw in
// degrees, body to earth) as a rotation in OpenXR axes (+Y up, -Z forward),
// relative to the orientation `start_deg` was read at.
XrQuaternionf imu_rotation_since(const double (&wrist_deg)[3], const double (&start_deg)[3]);

} // namespace pandora_glove
} // namespace plugins
