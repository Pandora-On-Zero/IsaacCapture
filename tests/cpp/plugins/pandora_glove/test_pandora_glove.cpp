// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// The 01 Glove plugin's pure parts: skeleton mapping, haptic levels, arguments.

#include "pandora_glove_config.hpp"
#include "pandora_hand_mapping.hpp"
#include "pandora_haptics.hpp"
#include "recorded_bones.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace plugins::pandora_glove;

namespace
{

int g_failures = 0;

void check(bool ok, const std::string& what)
{
    if (!ok)
    {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        ++g_failures;
    }
}

struct V
{
    double x, y, z;
};

V sub(const XrVector3f& a, const XrVector3f& b)
{
    return { double(a.x) - b.x, double(a.y) - b.y, double(a.z) - b.z };
}

double dot(const V& a, const V& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

V unit(const V& v)
{
    const double n = std::sqrt(dot(v, v));
    return { v.x / n, v.y / n, v.z / n };
}

// Joint axis `axis` (0 x, 1 y, 2 z) of a pose, in the frame the pose is in.
V axis_of(const XrQuaternionf& q, int axis)
{
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    switch (axis)
    {
    case 0:
        return { 1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y) };
    case 1:
        return { 2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x) };
    default:
        return { 2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y) };
    }
}

// Each finger joint's -Z points at the next joint of its finger.
void check_bone_directions(const HandJoints& j, const char* hand)
{
    const std::vector<std::vector<int>> chains = {
        { XR_HAND_JOINT_THUMB_METACARPAL_EXT, XR_HAND_JOINT_THUMB_PROXIMAL_EXT, XR_HAND_JOINT_THUMB_DISTAL_EXT,
          XR_HAND_JOINT_THUMB_TIP_EXT },
        { XR_HAND_JOINT_INDEX_METACARPAL_EXT, XR_HAND_JOINT_INDEX_PROXIMAL_EXT, XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT,
          XR_HAND_JOINT_INDEX_DISTAL_EXT, XR_HAND_JOINT_INDEX_TIP_EXT },
        { XR_HAND_JOINT_MIDDLE_METACARPAL_EXT, XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT, XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT,
          XR_HAND_JOINT_MIDDLE_DISTAL_EXT, XR_HAND_JOINT_MIDDLE_TIP_EXT },
        { XR_HAND_JOINT_RING_METACARPAL_EXT, XR_HAND_JOINT_RING_PROXIMAL_EXT, XR_HAND_JOINT_RING_INTERMEDIATE_EXT,
          XR_HAND_JOINT_RING_DISTAL_EXT, XR_HAND_JOINT_RING_TIP_EXT },
        { XR_HAND_JOINT_LITTLE_METACARPAL_EXT, XR_HAND_JOINT_LITTLE_PROXIMAL_EXT, XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT,
          XR_HAND_JOINT_LITTLE_DISTAL_EXT, XR_HAND_JOINT_LITTLE_TIP_EXT },
    };
    for (const auto& chain : chains)
    {
        for (size_t k = 0; k + 1 < chain.size(); ++k)
        {
            const V to_child = unit(sub(j[chain[k + 1]].pose.position, j[chain[k]].pose.position));
            const V minus_z = axis_of(j[chain[k]].pose.orientation, 2);
            const double d = -dot(minus_z, to_child);
            check(d > 0.98, std::string(hand) + ": joint " + std::to_string(chain[k]) + " -Z toward its child (" +
                                std::to_string(d) + ")");
        }
    }
}

// The thumb lies on the -X side of a right hand's joints and the +X side of
// a left hand's (OpenXR frames are right-handed with -Z distal, +Y dorsal).
void check_thumb_side(const HandJoints& j, bool is_left, const char* hand)
{
    for (int joint : { XR_HAND_JOINT_INDEX_METACARPAL_EXT, XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT, XR_HAND_JOINT_WRIST_EXT })
    {
        const V toward_thumb = unit(
            sub(j[XR_HAND_JOINT_INDEX_PROXIMAL_EXT].pose.position, j[XR_HAND_JOINT_LITTLE_PROXIMAL_EXT].pose.position));
        const double x = dot(axis_of(j[joint].pose.orientation, 0), toward_thumb);
        check(is_left ? x > 0.7 : x < -0.7,
              std::string(hand) + ": thumb side of joint " + std::to_string(joint) + " (" + std::to_string(x) + ")");
    }
}

void test_mapping(const PandoraBones& bones, bool is_left)
{
    const char* hand = is_left ? "left" : "right";
    const HandJoints j = map_bones_to_openxr(bones, is_left);
    const auto& w = j[XR_HAND_JOINT_WRIST_EXT].pose;
    check(std::abs(w.position.x) + std::abs(w.position.y) + std::abs(w.position.z) < 1e-6,
          std::string(hand) + ": wrist at origin");
    check(std::abs(w.orientation.w) > 0.999999, std::string(hand) + ": wrist unrotated");
    for (const auto& joint : j)
    {
        check(joint.locationFlags == (XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT),
              std::string(hand) + ": every joint valid, none tracked");
    }
    // The wrist joint's -Z points into the hand.
    const V into_hand = unit(sub(j[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT].pose.position, w.position));
    check(-dot(axis_of(w.orientation, 2), into_hand) > 0.9, std::string(hand) + ": wrist -Z into the hand");
    check_bone_directions(j, hand);
    check_thumb_side(j, is_left, hand);
    // A hand is about 18 cm from wrist to middle fingertip, measured along the finger.
    double length = 0.0;
    for (int a : { XR_HAND_JOINT_WRIST_EXT, XR_HAND_JOINT_MIDDLE_METACARPAL_EXT, XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT,
                   XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT, XR_HAND_JOINT_MIDDLE_DISTAL_EXT })
    {
        const int b = a == XR_HAND_JOINT_WRIST_EXT ? XR_HAND_JOINT_MIDDLE_METACARPAL_EXT : a + 1;
        const V bone = sub(j[b].pose.position, j[a].pose.position);
        length += std::sqrt(dot(bone, bone));
    }
    check(length > 0.12 && length < 0.30, std::string(hand) + ": plausible hand length " + std::to_string(length));
    // PALM halfway along the middle metacarpal.
    const V a = sub(j[XR_HAND_JOINT_PALM_EXT].pose.position, j[XR_HAND_JOINT_MIDDLE_METACARPAL_EXT].pose.position);
    const V b = sub(j[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT].pose.position, j[XR_HAND_JOINT_PALM_EXT].pose.position);
    check(std::abs(dot(a, a) - dot(b, b)) < 1e-6, std::string(hand) + ": palm centred on the metacarpal");

    // Composing on a wrist moves the wrist joint there and adds TRACKED bits.
    HandJoints placed = j;
    const XrPosef at{ { 0.0f, 0.0f, 0.0f, 1.0f }, { 0.1f, 1.0f, -0.3f } };
    compose_on_wrist(placed, at, true);
    check(std::abs(placed[XR_HAND_JOINT_WRIST_EXT].pose.position.y - 1.0f) < 1e-6, std::string(hand) + ": composed");
    check((placed[XR_HAND_JOINT_INDEX_TIP_EXT].locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0,
          std::string(hand) + ": tracked bits");
}

void test_imu()
{
    const double start[3] = { 10.0, -20.0, 30.0 };
    const XrQuaternionf same = imu_rotation_since(start, start);
    check(std::abs(same.w) > 0.999999, "imu: no rotation at the start pose");
    const double yawed[3] = { 0.0, 0.0, 90.0 };
    const double zero[3] = { 0.0, 0.0, 0.0 };
    // Yaw is a turn about the vertical: +Y in OpenXR.
    const XrQuaternionf q = imu_rotation_since(yawed, zero);
    check(std::abs(std::abs(q.y) - std::sqrt(0.5)) < 1e-6 && std::abs(q.x) < 1e-6 && std::abs(q.z) < 1e-6,
          "imu: yaw turns about +Y");
}

void test_haptics()
{
    const float five[5] = { 0.0f, 0.5f, 1.0f, 2.0f, -1.0f };
    auto a = amplitudes_from_powers(five, 5);
    check(a.has_value() && (*a)[0] == 0 && (*a)[1] == 64 && (*a)[2] == 127 && (*a)[3] == 127 && (*a)[4] == 0 &&
              (*a)[5] == 0,
          "haptics: five finger powers, clamped");
    const float six[6] = { 0, 0, 0, 0, 0, 1.0f };
    a = amplitudes_from_powers(six, 6);
    check(a.has_value() && (*a)[5] == 127, "haptics: sixth value is the back of the hand");
    check(!amplitudes_from_powers(six, 4).has_value(), "haptics: four values refused");
    const float nan[5] = { std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 0 };
    check(!amplitudes_from_powers(nan, 5).has_value(), "haptics: NaN refused");

    using Clock = HapticStream::Clock;
    using ms = std::chrono::milliseconds;
    HapticStream s(ms(60));
    const Clock::time_point t0{};
    Amplitudes on{};
    on[1] = 100;
    const Amplitudes off{};
    check(s.update(off, t0) == HapticStream::Action::None, "haptics: silence sends nothing");
    check(s.update(on, t0) == HapticStream::Action::Play, "haptics: a level plays");
    check(s.update(on, t0 + ms(10)) == HapticStream::Action::None, "haptics: a steady level is not resent at once");
    check(s.update(on, t0 + ms(31)) == HapticStream::Action::Play, "haptics: refreshed before the hold ends");
    Amplitudes more = on;
    more[1] = 120;
    check(s.update(more, t0 + ms(33)) == HapticStream::Action::None, "haptics: at most one frame per 5 ms");
    check(s.update(more, t0 + ms(37)) == HapticStream::Action::Play, "haptics: a change plays after 5 ms");
    check(s.update(off, t0 + ms(38)) == HapticStream::Action::Stop, "haptics: silence stops at once");
    check(s.update(off, t0 + ms(50)) == HapticStream::Action::None, "haptics: stopped once");
}

// The joints placed on wrist_in_root land where the bone chain puts them in the root frame.
void test_wrist_in_root(const PandoraBones& bones, bool is_left)
{
    const char* hand = is_left ? "left" : "right";
    // The bone chain in the root frame, by hand: position of bone i.
    const int parent[kPandoraBoneCount] = { -1, 0,  1,  2,  3,  4, 1,  6,  7,  8,  9, 1, 11, 12, 13, 14,
                                            1,  16, 17, 18, 19, 1, 21, 22, 23, 24, 0, 0, 0,  0,  0 };
    XrPosef chain[kPandoraBoneCount];
    for (int i = 0; i < kPandoraBoneCount; ++i)
    {
        const XrPosef local{ { static_cast<float>(bones[i][4]), static_cast<float>(bones[i][5]),
                               static_cast<float>(bones[i][6]), static_cast<float>(bones[i][3]) },
                             { static_cast<float>(bones[i][0]), static_cast<float>(bones[i][1]),
                               static_cast<float>(bones[i][2]) } };
        chain[i] = parent[i] < 0 ? local : compose_poses(chain[parent[i]], local);
    }
    const XrPosef wrist = wrist_in_root(bones, is_left);
    check(std::abs(wrist.position.x - chain[1].position.x) < 1e-6f &&
              std::abs(wrist.position.z - chain[1].position.z) < 1e-6f,
          std::string(hand) + ": wrist_in_root sits on the wrist bone");
    HandJoints joints = map_bones_to_openxr(bones, is_left);
    compose_on_wrist(joints, wrist, false);
    for (int j : { XR_HAND_JOINT_INDEX_TIP_EXT, XR_HAND_JOINT_THUMB_TIP_EXT, XR_HAND_JOINT_LITTLE_PROXIMAL_EXT })
    {
        const V d = sub(joints[j].pose.position, chain[j].position);
        check(std::sqrt(dot(d, d)) < 1e-5, std::string(hand) + ": joint " + std::to_string(j) + " in the root frame");
    }
}

void test_tracker_offset_reply()
{
    const std::string reply = R"({"type":"tracker_offset","device":"g","stored":true,"offset":{"pos_x":-2.0},)"
                              R"("pose":[-0.02,-0.035,0.05,0.9914448613738104,-0.13052619222005157,0.0,0.0]})";
    const auto mount = parse_tracker_offset_reply(reply);
    check(mount.has_value() && mount->stored && std::abs(mount->pose[0] + 0.02) < 1e-12 &&
              std::abs(mount->pose[4] + 0.13052619222005157) < 1e-12,
          "tracker offset: stored pose parsed");
    const auto none = parse_tracker_offset_reply(R"({"stored": false, "pose": [0, 0, 0, 1, 0, 0, 0]})");
    check(none.has_value() && !none->stored && none->pose[3] == 1.0, "tracker offset: none stored");
    check(!parse_tracker_offset_reply(R"({"stored":true,"pose":[1,2,3]})").has_value(),
          "tracker offset: short pose refused");
    check(!parse_tracker_offset_reply("{}").has_value(), "tracker offset: no pose refused");
}

void test_config()
{
    PandoraGloveConfig c = parse_args({});
    check(c.rate_hz == 90.0 && c.hands && c.haptics && c.joints && c.wrist == WristMode::Auto, "config: defaults");
    c = parse_args({ "--rate=200", "--wrist=imu", "--datasets=hand,haptic", "--port=/dev/ttyACM0" });
    check(c.rate_hz == 200.0 && c.wrist == WristMode::Imu && c.hands && c.haptics && !c.joints && c.port == "/dev/ttyACM0",
          "config: values");
    check(parse_args({ "--rate=max" }).rate_hz == 0.0, "config: rate max is every frame");
    c = parse_args({ "--wrist=se3:vive_tracker_A,vive_tracker_B" });
    check(c.wrist == WristMode::Se3 && c.se3_left == "vive_tracker_A" && c.se3_right == "vive_tracker_B",
          "config: se3 wrist with both collections");
    check(!c.left_aim_given && !c.right_aim_given, "config: no aim offset given");
    check(parse_args({ "--right-aim-to-wrist=0,0,0,0,0,0,1" }).right_aim_given, "config: aim offset given");
    c = parse_args({ "--left-aim-to-wrist=0,0,0.1,0,0,0,2" });
    check(std::abs(c.left_aim_to_wrist.orientation.w - 1.0f) < 1e-6, "config: quaternion normalised");
    std::vector<std::string> unknown;
    parse_args({ "--plugin-root-id=gloves", "--nope=1" }, &unknown);
    check(unknown.size() == 1 && unknown[0] == "--nope=1", "config: unknown arguments returned");
    for (const char* bad : { "--rate=0", "--rate=abc", "--wrist=up", "--wrist=se3:onlyone", "--wrist=se3:,b",
                             "--datasets=", "--haptic-hold-ms=5", "--right-aim-to-wrist=1,2,3", "--glove-profiles=maybe" })
    {
        bool threw = false;
        try
        {
            parse_args({ bad });
        }
        catch (const std::runtime_error&)
        {
            threw = true;
        }
        check(threw, std::string("config: refuses ") + bad);
    }
    check(joint_names().size() == 20 && joint_names().front() == "thumb_cmc_flexion" &&
              joint_names().back() == "little_mcp_abduction",
          "config: joint names");
}

} // namespace

int main()
{
    test_mapping(fixtures::kRightBones, false);
    test_mapping(fixtures::kLeftBones, true);
    test_wrist_in_root(fixtures::kRightBones, false);
    test_wrist_in_root(fixtures::kLeftBones, true);
    test_tracker_offset_reply();
    test_imu();
    test_haptics();
    test_config();
    if (g_failures == 0)
    {
        std::printf("pandora_glove: all checks passed\n");
    }
    return g_failures == 0 ? 0 : 1;
}
