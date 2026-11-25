// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "pandora_hand_mapping.hpp"

#include <cmath>

namespace plugins
{
namespace pandora_glove
{

namespace
{

struct Vec
{
    double x, y, z;
};

struct Quat
{
    double w, x, y, z;
};

struct Pose
{
    Vec p;
    Quat q;
};

using Mat = std::array<std::array<double, 3>, 3>; // row-major

Quat mul(const Quat& a, const Quat& b)
{
    return { a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
             a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
}

Quat conj(const Quat& q)
{
    return { q.w, -q.x, -q.y, -q.z };
}

Quat normalized(const Quat& q)
{
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12)
    {
        return { 1.0, 0.0, 0.0, 0.0 };
    }
    return { q.w / n, q.x / n, q.y / n, q.z / n };
}

Vec rotate(const Quat& q, const Vec& v)
{
    const Quat r = mul(mul(q, Quat{ 0.0, v.x, v.y, v.z }), conj(q));
    return { r.x, r.y, r.z };
}

Pose compose(const Pose& a, const Pose& b)
{
    const Vec r = rotate(a.q, b.p);
    return { { a.p.x + r.x, a.p.y + r.y, a.p.z + r.z }, mul(a.q, b.q) };
}

Pose inverse(const Pose& a)
{
    const Quat qi = conj(a.q);
    const Vec p = rotate(qi, a.p);
    return { { -p.x, -p.y, -p.z }, qi };
}

Quat quat_from_matrix(const Mat& m)
{
    const double trace = m[0][0] + m[1][1] + m[2][2];
    Quat q{};
    if (trace > 0.0)
    {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q = { 0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s };
    }
    else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
    {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        q = { (m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s };
    }
    else if (m[1][1] > m[2][2])
    {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        q = { (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s };
    }
    else
    {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        q = { (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s };
    }
    return normalized(q);
}

// The rotation whose columns are where the OpenXR joint axes x, y, z lie in
// the OpenVR bone's own frame.
Quat basis(const Vec& x, const Vec& y, const Vec& z)
{
    return quat_from_matrix(Mat{ { { x.x, y.x, z.x }, { x.y, y.y, z.y }, { x.z, y.z, z.z } } });
}

constexpr Vec kPX{ 1, 0, 0 }, kNX{ -1, 0, 0 }, kPY{ 0, 1, 0 }, kNY{ 0, -1, 0 }, kPZ{ 0, 0, 1 }, kNZ{ 0, 0, -1 };

// Bone frames measured in libpandora's recorded skeletons: right finger bones
// run along local -X with the back of the hand at +Y, left ones along +X with
// it at -Y; the wrist bone runs along +Z toward the fingers with the back of
// the hand at -X (right) / +X (left). OpenXR wants -Z toward the fingertip and
// +Y out of the back of the hand.
Quat finger_basis(bool is_left)
{
    return is_left ? basis(kNZ, kNY, kNX) : basis(kNZ, kPY, kPX);
}

Quat wrist_basis(bool is_left)
{
    return is_left ? basis(kPY, kPX, kNZ) : basis(kNY, kNX, kNZ);
}

// OpenVR parent of each bone; the aux bones (26-30) are not mapped.
constexpr std::array<int, kPandoraBoneCount> kParent = { -1, 0,  1,  2,  3,  4, 1,  6,  7,  8,  9, 1, 11, 12, 13, 14,
                                                         1,  16, 17, 18, 19, 1, 21, 22, 23, 24, 0, 0, 0,  0,  0 };

constexpr double kPi = 3.14159265358979323846;
constexpr float kJointRadius = 0.01f;
constexpr float kTipRadius = 0.008f;

bool is_tip(int joint)
{
    return joint == XR_HAND_JOINT_THUMB_TIP_EXT || joint == XR_HAND_JOINT_INDEX_TIP_EXT ||
           joint == XR_HAND_JOINT_MIDDLE_TIP_EXT || joint == XR_HAND_JOINT_RING_TIP_EXT ||
           joint == XR_HAND_JOINT_LITTLE_TIP_EXT;
}

XrPosef to_xr(const Pose& p)
{
    const Quat q = normalized(p.q);
    return { { static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z), static_cast<float>(q.w) },
             { static_cast<float>(p.p.x), static_cast<float>(p.p.y), static_cast<float>(p.p.z) } };
}

Pose from_xr(const XrPosef& p)
{
    return { { p.position.x, p.position.y, p.position.z },
             { p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z } };
}

Mat mat_mul(const Mat& a, const Mat& b)
{
    Mat out{};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    return out;
}

Mat transpose(const Mat& a)
{
    Mat out{};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out[i][j] = a[j][i];
        }
    }
    return out;
}

Mat rx(double deg)
{
    const double r = deg * kPi / 180.0, c = std::cos(r), s = std::sin(r);
    return Mat{ { { 1, 0, 0 }, { 0, c, -s }, { 0, s, c } } };
}

Mat ry(double deg)
{
    const double r = deg * kPi / 180.0, c = std::cos(r), s = std::sin(r);
    return Mat{ { { c, 0, s }, { 0, 1, 0 }, { -s, 0, c } } };
}

Mat rz(double deg)
{
    const double r = deg * kPi / 180.0, c = std::cos(r), s = std::sin(r);
    return Mat{ { { c, -s, 0 }, { s, c, 0 }, { 0, 0, 1 } } };
}

// Fusion ZYX, body to earth: R = Rz(yaw) * Ry(pitch) * Rx(roll).
Mat euler(const double (&deg)[3])
{
    return mat_mul(mat_mul(rz(deg[2]), ry(deg[0])), rx(deg[1]));
}

} // namespace

HandJoints map_bones_to_openxr(const PandoraBones& bones, bool is_left)
{
    std::array<Pose, kPandoraBoneCount> root_space{};
    for (int i = 0; i < kPandoraBoneCount; ++i)
    {
        const double* b = bones[i];
        const Pose local{ { b[0], b[1], b[2] }, normalized(Quat{ b[3], b[4], b[5], b[6] }) };
        root_space[i] = kParent[i] < 0 ? local : compose(root_space[kParent[i]], local);
    }

    const Quat finger = finger_basis(is_left);
    const Pose wrist{ root_space[1].p, mul(root_space[1].q, wrist_basis(is_left)) };
    const Pose wrist_inv = inverse(wrist);

    HandJoints out{};
    for (int j = XR_HAND_JOINT_WRIST_EXT; j <= XR_HAND_JOINT_LITTLE_TIP_EXT; ++j)
    {
        // OpenXR joints 1..25 are OpenVR bones 1..25 in the same order.
        const Pose joint = j == XR_HAND_JOINT_WRIST_EXT ? wrist : Pose{ root_space[j].p, mul(root_space[j].q, finger) };
        out[j].pose = to_xr(compose(wrist_inv, joint));
        out[j].radius = is_tip(j) ? kTipRadius : kJointRadius;
        out[j].locationFlags = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    }

    // PALM: the centre of the middle metacarpal, oriented like it.
    const Vec& base = root_space[XR_HAND_JOINT_MIDDLE_METACARPAL_EXT].p;
    const Vec& knuckle = root_space[XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT].p;
    const Pose palm{ { (base.x + knuckle.x) * 0.5, (base.y + knuckle.y) * 0.5, (base.z + knuckle.z) * 0.5 },
                     mul(root_space[XR_HAND_JOINT_MIDDLE_METACARPAL_EXT].q, finger) };
    out[XR_HAND_JOINT_PALM_EXT].pose = to_xr(compose(wrist_inv, palm));
    out[XR_HAND_JOINT_PALM_EXT].radius = kJointRadius;
    out[XR_HAND_JOINT_PALM_EXT].locationFlags =
        XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    return out;
}

void compose_on_wrist(HandJoints& joints, const XrPosef& wrist, bool tracked)
{
    const Pose w = from_xr(wrist);
    for (auto& joint : joints)
    {
        if ((joint.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) == 0)
        {
            continue;
        }
        joint.pose = to_xr(compose(w, from_xr(joint.pose)));
        if (tracked)
        {
            joint.locationFlags |= XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
        }
    }
}

XrQuaternionf imu_rotation_since(const double (&wrist_deg)[3], const double (&start_deg)[3])
{
    // Earth (north-west-up) to OpenXR (+Y up, viewer at +Z), as Pandora-Web's src/hand/imu.ts.
    const Mat earth_to_xr = mat_mul(rx(-90.0), rz(180.0));
    const Mat r =
        mat_mul(mat_mul(earth_to_xr, mat_mul(euler(wrist_deg), transpose(euler(start_deg)))), transpose(earth_to_xr));
    const Quat q = quat_from_matrix(r);
    return { static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z), static_cast<float>(q.w) };
}

} // namespace pandora_glove
} // namespace plugins
