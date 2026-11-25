// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "pandora_glove_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace plugins
{
namespace pandora_glove
{

namespace
{

bool take(const std::string& arg, const char* flag, std::string& value)
{
    const std::string prefix = std::string(flag) + "=";
    if (arg.compare(0, prefix.size(), prefix) != 0)
    {
        return false;
    }
    value = arg.substr(prefix.size());
    return true;
}

[[noreturn]] void bad(const std::string& flag, const std::string& value, const std::string& want)
{
    throw std::runtime_error("PandoraGlovePlugin: " + flag + " expects " + want + ", got '" + value + "'");
}

double parse_number(const std::string& flag, const std::string& value, double lo, double hi)
{
    std::size_t used = 0;
    double out = 0.0;
    try
    {
        out = std::stod(value, &used);
    }
    catch (const std::exception&)
    {
        bad(flag, value, "a number");
    }
    if (used != value.size() || !std::isfinite(out) || out < lo || out > hi)
    {
        std::ostringstream want;
        want << "a number from " << lo << " to " << hi;
        bad(flag, value, want.str());
    }
    return out;
}

// px,py,pz,qx,qy,qz,qw: metres, then a quaternion (normalised here).
XrPosef parse_pose(const std::string& flag, const std::string& value)
{
    XrPosef pose{};
    int consumed = -1;
    const int parsed = std::sscanf(value.c_str(), "%f ,%f ,%f ,%f ,%f ,%f ,%f %n", &pose.position.x, &pose.position.y,
                                   &pose.position.z, &pose.orientation.x, &pose.orientation.y, &pose.orientation.z,
                                   &pose.orientation.w, &consumed);
    if (parsed != 7 || consumed < 0 || static_cast<std::size_t>(consumed) != value.size())
    {
        bad(flag, value, "px,py,pz,qx,qy,qz,qw");
    }
    const std::array<float, 7> all{ pose.position.x,    pose.position.y,    pose.position.z,   pose.orientation.x,
                                    pose.orientation.y, pose.orientation.z, pose.orientation.w };
    if (!std::all_of(all.begin(), all.end(), [](float c) { return std::isfinite(c); }))
    {
        bad(flag, value, "finite values");
    }
    XrQuaternionf& q = pose.orientation;
    const double n = std::sqrt(static_cast<double>(q.x) * q.x + static_cast<double>(q.y) * q.y +
                               static_cast<double>(q.z) * q.z + static_cast<double>(q.w) * q.w);
    if (n < 1e-6)
    {
        bad(flag, value, "a non-zero quaternion");
    }
    q.x = static_cast<float>(q.x / n);
    q.y = static_cast<float>(q.y / n);
    q.z = static_cast<float>(q.z / n);
    q.w = static_cast<float>(q.w / n);
    return pose;
}

WristMode parse_wrist(const std::string& value)
{
    if (value == "auto")
    {
        return WristMode::Auto;
    }
    if (value == "hand_tracking")
    {
        return WristMode::HandTracking;
    }
    if (value == "controller")
    {
        return WristMode::Controller;
    }
    if (value == "fixed")
    {
        return WristMode::Fixed;
    }
    if (value == "imu")
    {
        return WristMode::Imu;
    }
    bad("--wrist", value, "auto, hand_tracking, controller, fixed or imu");
}

} // namespace

PandoraGloveConfig parse_args(const std::vector<std::string>& args, std::vector<std::string>* unknown)
{
    PandoraGloveConfig config;
    std::string value;
    for (const auto& arg : args)
    {
        if (take(arg, "--plugin-root-id", value))
        {
            if (value.empty())
            {
                bad("--plugin-root-id", value, "a non-empty id");
            }
            config.plugin_root_id = value;
        }
        else if (take(arg, "--port", value))
        {
            config.port = value;
        }
        else if (take(arg, "--profiles-dir", value))
        {
            config.profiles_dir = value;
        }
        else if (take(arg, "--glove-profiles", value))
        {
            if (value != "on" && value != "off")
            {
                bad("--glove-profiles", value, "on or off");
            }
            config.read_glove_profiles = value == "on";
        }
        else if (take(arg, "--rate", value))
        {
            config.rate_hz = value == "max" ? 0.0 : parse_number("--rate", value, 1.0, 1000.0);
        }
        else if (take(arg, "--wrist", value))
        {
            config.wrist = parse_wrist(value);
        }
        else if (take(arg, "--left-aim-to-wrist", value))
        {
            config.left_aim_to_wrist = parse_pose("--left-aim-to-wrist", value);
        }
        else if (take(arg, "--right-aim-to-wrist", value))
        {
            config.right_aim_to_wrist = parse_pose("--right-aim-to-wrist", value);
        }
        else if (take(arg, "--datasets", value))
        {
            config.hands = config.haptics = config.joints = false;
            std::stringstream list(value);
            std::string item;
            while (std::getline(list, item, ','))
            {
                if (item == "hand")
                {
                    config.hands = true;
                }
                else if (item == "haptic")
                {
                    config.haptics = true;
                }
                else if (item == "joints")
                {
                    config.joints = true;
                }
                else
                {
                    bad("--datasets", value, "a list of hand, haptic, joints");
                }
            }
            if (!config.hands && !config.haptics && !config.joints)
            {
                bad("--datasets", value, "at least one of hand, haptic, joints");
            }
        }
        else if (take(arg, "--haptic-collection", value))
        {
            if (value.empty())
            {
                bad("--haptic-collection", value, "a non-empty collection id");
            }
            config.haptic_collection = value;
        }
        else if (take(arg, "--haptic-hold-ms", value))
        {
            config.haptic_hold =
                std::chrono::milliseconds(static_cast<int>(parse_number("--haptic-hold-ms", value, 10.0, 1000.0)));
        }
        else if (take(arg, "--stale-ms", value))
        {
            config.stale_after =
                std::chrono::milliseconds(static_cast<int>(parse_number("--stale-ms", value, 20.0, 5000.0)));
        }
        else if (unknown != nullptr)
        {
            unknown->push_back(arg);
        }
    }
    return config;
}

const std::vector<std::string>& joint_names()
{
    static const std::vector<std::string> names = []
    {
        const std::array<const char*, 5> fingers = { "thumb", "index", "middle", "ring", "little" };
        const std::array<const char*, 4> thumb = { "cmc_flexion", "mcp_flexion", "ip_flexion", "cmc_abduction" };
        const std::array<const char*, 4> finger = { "mcp_flexion", "pip_flexion", "dip_flexion", "mcp_abduction" };
        std::vector<std::string> out;
        for (std::size_t f = 0; f < fingers.size(); ++f)
        {
            for (const char* joint : f == 0 ? thumb : finger)
            {
                out.push_back(std::string(fingers[f]) + "_" + joint);
            }
        }
        return out;
    }();
    return names;
}

} // namespace pandora_glove
} // namespace plugins
