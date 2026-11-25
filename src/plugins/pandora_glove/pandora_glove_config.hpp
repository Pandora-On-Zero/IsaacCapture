// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <openxr/openxr.h>

#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace plugins
{
namespace pandora_glove
{

enum class WristMode
{
    Auto, //!< headset hand tracking, else the controller aim pose (plugin_utils)
    HandTracking,
    Controller,
    Fixed, //!< no tracker: both hands at fixed poses in front of the origin
    Imu, //!< fixed positions, turned by the glove's own wrist IMU
    Se3, //!< an SE3 tracker stream per hand (e.g. vive_se3_tracker's vive_tracker_<serial>)
};

struct PandoraGloveConfig
{
    //! Set by TeleopSession; enables device-status publishing.
    std::optional<std::string> plugin_root_id;
    //! Receiver serial port; empty: the one plugged in.
    std::string port;
    //! Profiles folder; empty: libpandora's default (~/.pandora/profiles).
    std::string profiles_dir;
    //! Read a glove with no profile file with the calibration it stores itself.
    bool read_glove_profiles = true;
    //! Frames pushed per second per hand; 0 pushes every frame the glove sends.
    double rate_hz = 90.0;
    WristMode wrist = WristMode::Auto;
    XrPosef left_aim_to_wrist{ { 0.0f, 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f } };
    XrPosef right_aim_to_wrist{ { 0.0f, 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f } };
    //! Given on the command line; otherwise the glove's stored tracker offset places the hand.
    bool left_aim_given = false;
    bool right_aim_given = false;
    //! --wrist=se3:<left>,<right>: the SE3 tracker collections fixed to each glove.
    std::string se3_left;
    std::string se3_right;
    bool hands = true;
    bool haptics = true;
    bool joints = true;
    std::string haptic_collection = "pandora_glove_haptic";
    std::chrono::milliseconds haptic_hold{ 60 };
    std::chrono::milliseconds stale_after{ 200 };
};

//! Parses `--key=value` arguments; throws std::runtime_error on a bad value.
//! Unknown arguments are returned in `unknown` for the caller to report.
PandoraGloveConfig parse_args(const std::vector<std::string>& args, std::vector<std::string>* unknown = nullptr);

//! libpandora's tracker_offset message, as far as the plugin needs it.
struct TrackerMountReply
{
    //! The hand root's pose in the tracker's frame: x, y, z, qw, qx, qy, qz.
    std::array<double, 7> pose{ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
    //! False when the glove keeps no offset (the pose is then the identity).
    bool stored = false;
};

//! The "pose" and "stored" of a tracker_offset message (pandora_tracker_offset's JSON).
std::optional<TrackerMountReply> parse_tracker_offset_reply(const std::string& json);

//! The joint names pushed as JointStateOutput, finger-major: three flexions
//! from the knuckle outward, then the abduction at the base.
const std::vector<std::string>& joint_names();

} // namespace pandora_glove
} // namespace plugins
