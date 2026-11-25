// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <openxr/openxr.h>

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

//! The joint names pushed as JointStateOutput, finger-major: three flexions
//! from the knuckle outward, then the abduction at the base.
const std::vector<std::string>& joint_names();

} // namespace pandora_glove
} // namespace plugins
