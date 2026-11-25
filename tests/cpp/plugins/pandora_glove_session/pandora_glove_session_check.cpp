// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Runs the real 01 Glove plugin (with libpandora reading a receiver, real or
// the stand-in) against a capturing plugin session instead of CloudXR, then
// checks what it pushed: hand joints, rates, wrist placement, joint angles,
// device status, and scripted haptic commands read through
// HapticCommandReaderTracker.
//
//   pandora_glove_session_check --seconds=10 --expect-rate=200 [plugin arguments...]

#include "pandora_glove_config.hpp"
#include "pandora_glove_plugin.hpp"

#include <deviceio_base/haptic_command_reader_tracker_base.hpp>
#include <flatbuffers/flatbuffers.h>
#include <schema/haptic_command_generated.h>
#include <schema/joint_state_generated.h>
#include <schema/plugin_device_status_generated.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace plugins::pandora_glove;
using Clock = std::chrono::steady_clock;

namespace
{

constexpr int kHapticMotors = 6;
constexpr float kHapticLevel = 0.6f;
constexpr double kHapticStart = 1.0;
constexpr double kHapticStep = 0.3;

Clock::time_point g_start;

double now_s()
{
    return std::chrono::duration<double>(Clock::now() - g_start).count();
}

struct HandPush
{
    double t;
    bool left;
    HandJoints joints;
};

struct JointSample
{
    double t;
    std::string device;
    std::vector<std::string> names;
    std::vector<float> positions;
};

struct StatusSample
{
    double t;
    std::map<std::string, std::string> states;
};

struct Captured
{
    std::vector<HandPush> pushes;
    std::vector<std::string> timeline;
    std::map<std::string, std::vector<JointSample>> joints;
    std::vector<StatusSample> status;
    std::map<bool, XrPosef> last_wrist_query;
    core::PluginSessionRequirements requirements;
    size_t haptic_reads = 0;
};

Captured g;

void event(const std::string& what)
{
    char buf[32];
    std::snprintf(buf, sizeof buf, "%6.2f s  ", now_s());
    g.timeline.push_back(buf + what);
}

const char* state_name(core::PluginDeviceState s)
{
    return core::EnumNamePluginDeviceState(s);
}

class HandChannel final : public core::IHandTrackingPushChannel
{
public:
    explicit HandChannel(bool left) : left_(left)
    {
        event(std::string(left ? "left" : "right") + " hand channel opened");
    }
    ~HandChannel() override
    {
        event(std::string(left_ ? "left" : "right") + " hand channel closed (inactive)");
    }
    void push(const XrHandJointLocationEXT* joints, int64_t) override
    {
        HandPush p{ now_s(), left_, {} };
        std::copy(joints, joints + XR_HAND_JOINT_COUNT_EXT, p.joints.begin());
        g.pushes.push_back(p);
    }

private:
    bool left_;
};

class SchemaChannel final : public core::ISchemaPushChannel
{
public:
    explicit SchemaChannel(std::string collection) : collection_(std::move(collection))
    {
    }
    void push_buffer(const uint8_t* buffer, size_t, int64_t, int64_t) override
    {
        if (collection_.find("device_status") != std::string::npos)
        {
            const auto* snap = flatbuffers::GetRoot<core::PluginDeviceStatusSnapshot>(buffer);
            StatusSample s{ now_s(), {} };
            if (snap->devices())
            {
                for (const auto* d : *snap->devices())
                {
                    std::string text = state_name(d->state());
                    if (d->reason() != core::PluginDeviceReason_NONE)
                    {
                        text += std::string("/") + core::EnumNamePluginDeviceReason(d->reason());
                    }
                    if (d->error() && d->error()->size())
                    {
                        text += " \"" + d->error()->str() + "\"";
                    }
                    s.states[d->path()->str()] = text;
                }
            }
            if (g.status.empty() || g.status.back().states != s.states)
            {
                std::string line = "status:";
                for (const auto& [path, text] : s.states)
                {
                    line += " " + path + "=" + text;
                }
                event(line);
            }
            g.status.push_back(s);
            return;
        }
        const auto* out = flatbuffers::GetRoot<core::JointStateOutput>(buffer);
        JointSample s{ now_s(), out->device_id() ? out->device_id()->str() : "", {}, {} };
        if (out->joints())
        {
            for (const auto* j : *out->joints())
            {
                s.names.push_back(j->name()->str());
                s.positions.push_back(j->position());
            }
        }
        g.joints[collection_].push_back(std::move(s));
    }

private:
    std::string collection_;
};

// A controller-like wrist that turns slowly about +Y, always tracked.
class FakeWrist final : public core::IWristTrackingSource
{
public:
    core::WristTrackingSample query(XrHandEXT hand, int64_t) override
    {
        const bool left = hand == XR_HAND_LEFT_EXT;
        const float a = static_cast<float>(0.25 * now_s());
        core::WristTrackingSample s;
        s.pose = { { 0.0f, std::sin(a), 0.0f, std::cos(a) }, { left ? -0.2f : 0.2f, 1.3f, -0.3f } };
        s.valid = true;
        s.tracked = true;
        g.last_wrist_query[left] = s.pose;
        return s;
    }
};

// Scripted HapticCommands: each motor in turn at kHapticLevel, then nothing.
class FakeHapticImpl final : public core::IHapticCommandReaderTrackerImpl
{
public:
    void update(int64_t) override
    {
    }
    const core::Serialized<core::HapticCommand>& get_data() const override
    {
        return get_data("left");
    }
    const core::Serialized<core::HapticCommand>& get_data(std::string_view endpoint) const override
    {
        ++g.haptic_reads;
        const int motor = static_cast<int>(std::floor((now_s() - kHapticStart) / kHapticStep));
        Slot& slot = endpoint == "left" ? left_ : right_;
        if (motor < 0 || motor >= kHapticMotors)
        {
            slot.command = {};
            slot.motor = -1;
            return slot.command;
        }
        if (motor != slot.motor)
        {
            flatbuffers::FlatBufferBuilder fbb(128);
            std::vector<float> values(kHapticMotors, 0.0f);
            values[motor] = kHapticLevel;
            fbb.Finish(core::CreateHapticCommandDirect(fbb, std::string(endpoint).c_str(), &values));
            slot.command = core::Serialized<core::HapticCommand>::adopt(fbb);
            slot.motor = motor;
        }
        return slot.command;
    }

private:
    struct Slot
    {
        core::Serialized<core::HapticCommand> command;
        int motor = -1;
    };
    mutable Slot left_;
    mutable Slot right_;
};

class FakePull final : public core::IPluginPullChannel
{
public:
    void update() override
    {
    }
    core::IWristTrackingSource& wrist_tracking_source() override
    {
        if (!g.requirements.wrist_tracking_pull)
        {
            throw std::logic_error("wrist tracking pull was not declared");
        }
        return wrist_;
    }
    const core::ITrackerImpl& get_tracker_impl(const core::ITracker&) const override
    {
        return haptics_;
    }

private:
    FakeWrist wrist_;
    FakeHapticImpl haptics_;
};

class FakeSession final : public core::IPluginSession
{
public:
    core::IPluginPullChannel& pull_channel() override
    {
        return pull_;
    }
    std::unique_ptr<core::ISchemaPushChannel> create_schema_push_channel(const core::SchemaPusherConfig& config) override
    {
        if (!g.requirements.schema_push)
        {
            throw std::logic_error("schema push was not declared");
        }
        return std::make_unique<SchemaChannel>(config.collection_id);
    }
    std::unique_ptr<core::IHandTrackingPushChannel> create_hand_tracking_push_channel(XrHandEXT hand) override
    {
        if (!g.requirements.hand_tracking_push)
        {
            throw std::logic_error("hand tracking push was not declared");
        }
        return std::make_unique<HandChannel>(hand == XR_HAND_LEFT_EXT);
    }

private:
    FakePull pull_;
};

// ── checks ───────────────────────────────────────────────────────────────

int g_failures = 0;

void fail(const std::string& what)
{
    std::printf("  FAIL: %s\n", what.c_str());
    ++g_failures;
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
V axis_of(const XrQuaternionf& q, int axis)
{
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    if (axis == 0)
        return { 1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y) };
    if (axis == 1)
        return { 2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x) };
    return { 2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y) };
}

std::vector<std::string> shape_problems(const HandJoints& j, bool left)
{
    std::vector<std::string> out;
    const int chains[5][5] = {
        { 2, 3, 4, 5, -1 }, { 6, 7, 8, 9, 10 }, { 11, 12, 13, 14, 15 }, { 16, 17, 18, 19, 20 }, { 21, 22, 23, 24, 25 }
    };
    for (const auto& chain : chains)
    {
        for (int k = 0; k + 1 < 5 && chain[k + 1] >= 0; ++k)
        {
            const V to_child = unit(sub(j[chain[k + 1]].pose.position, j[chain[k]].pose.position));
            if (-dot(axis_of(j[chain[k]].pose.orientation, 2), to_child) < 0.98)
            {
                out.push_back("joint " + std::to_string(chain[k]) + " -Z not toward its child");
            }
        }
    }
    const V thumb = unit(sub(j[7].pose.position, j[22].pose.position));
    const double x = dot(axis_of(j[1].pose.orientation, 0), thumb);
    if (left ? x < 0.7 : x > -0.7)
    {
        out.push_back("thumb on the wrong side");
    }
    // Wrist to middle fingertip along the finger: the same open or curled.
    double len = 0.0;
    for (int a : { 1, 11, 12, 13, 14 })
    {
        const V bone = sub(j[a + (a == 1 ? 10 : 1)].pose.position, j[a].pose.position);
        len += std::sqrt(dot(bone, bone));
    }
    if (len < 0.12 || len > 0.30)
    {
        out.push_back("hand length " + std::to_string(len));
    }
    for (const auto& joint : j)
    {
        if ((joint.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) == 0)
        {
            out.push_back("a joint is not valid");
            break;
        }
    }
    return out;
}

double percentile(std::vector<double> v, double p)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    return v[static_cast<size_t>(p * (v.size() - 1))];
}

void report_hand(bool left, const PandoraGloveConfig& config, double seconds, double expect_rate)
{
    std::vector<const HandPush*> pushes;
    for (const auto& p : g.pushes)
    {
        if (p.left == left)
            pushes.push_back(&p);
    }
    const char* name = left ? "left" : "right";
    std::printf("\n%s hand: %zu pushes\n", name, pushes.size());
    if (pushes.empty())
    {
        fail(std::string(name) + " hand never pushed");
        return;
    }
    // Rate over the steady part: from 1 s after the first push to the end.
    const double from = pushes.front()->t + 1.0;
    size_t steady = 0;
    std::vector<double> gaps;
    for (size_t i = 0; i < pushes.size(); ++i)
    {
        if (pushes[i]->t >= from)
            ++steady;
        if (i > 0)
            gaps.push_back(1000.0 * (pushes[i]->t - pushes[i - 1]->t));
    }
    const double rate = steady / std::max(1e-6, seconds - from);
    std::printf("  rate %.1f pushes/s; gap median %.2f ms, p95 %.2f ms, max %.1f ms\n", rate, percentile(gaps, 0.5),
                percentile(gaps, 0.95), percentile(gaps, 1.0));
    if (expect_rate > 0 && rate < 0.9 * expect_rate)
        fail(std::string(name) + " rate below 90% of " + std::to_string(expect_rate));
    if (expect_rate > 0 && rate > 1.1 * expect_rate)
        fail(std::string(name) + " rate above 110% of " + std::to_string(expect_rate));

    std::map<std::string, int> problems;
    for (size_t i = 0; i < pushes.size(); i += 10)
    {
        for (const auto& p : shape_problems(pushes[i]->joints, left))
            ++problems[p];
    }
    // Bones are rigid: each joint-to-next distance stays what it was in the first push.
    const auto bone_lengths = [](const HandJoints& j)
    {
        std::vector<double> out;
        for (int a = XR_HAND_JOINT_THUMB_METACARPAL_EXT; a < XR_HAND_JOINT_LITTLE_TIP_EXT; ++a)
        {
            if (a == XR_HAND_JOINT_THUMB_TIP_EXT || a == XR_HAND_JOINT_INDEX_TIP_EXT ||
                a == XR_HAND_JOINT_MIDDLE_TIP_EXT || a == XR_HAND_JOINT_RING_TIP_EXT)
                continue;
            const V bone = sub(j[a + 1].pose.position, j[a].pose.position);
            out.push_back(std::sqrt(dot(bone, bone)));
        }
        return out;
    };
    const auto first = bone_lengths(pushes.front()->joints);
    double stretch = 0.0;
    for (const auto* p : pushes)
    {
        const auto lengths = bone_lengths(p->joints);
        for (size_t i = 0; i < lengths.size(); ++i)
            stretch = std::max(stretch, std::abs(lengths[i] - first[i]));
    }
    std::printf("  bone lengths vary by at most %.4f mm across all pushes\n", stretch * 1000.0);
    if (stretch > 0.0005)
        ++problems["bones change length"];
    for (const auto& [p, n] : problems)
        fail(std::string(name) + ": " + p + " (" + std::to_string(n) + " samples)");
    if (problems.empty())
        std::printf("  ok: hand shape correct in every checked push\n");

    const auto& last = pushes.back()->joints;
    const auto& w = last[XR_HAND_JOINT_WRIST_EXT].pose;
    const bool tracked = (last[XR_HAND_JOINT_INDEX_TIP_EXT].locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0;
    std::printf("  last wrist (%+.3f, %+.3f, %+.3f) quat (%+.3f, %+.3f, %+.3f, %+.3f), %s\n", w.position.x,
                w.position.y, w.position.z, w.orientation.x, w.orientation.y, w.orientation.z, w.orientation.w,
                tracked ? "tracked" : "not tracked");
    if (config.wrist == WristMode::Fixed || config.wrist == WristMode::Imu)
    {
        const float x = left ? -0.12f : 0.12f;
        if (std::abs(w.position.x - x) > 1e-4f || std::abs(w.position.y - 1.0f) > 1e-4f ||
            std::abs(w.position.z + 0.35f) > 1e-4f)
            fail(std::string(name) + ": wrist not at the fixed test position");
        size_t turned = 0;
        for (const auto* p : pushes)
            turned += std::abs(p->joints[1].pose.orientation.w) < 0.999f;
        std::printf("  wrist turned in %zu of %zu pushes\n", turned, pushes.size());
        if (config.wrist == WristMode::Fixed && turned)
            fail(std::string(name) + ": fixed wrist rotated");
    }
    else
    {
        const XrPosef q = g.last_wrist_query[left];
        if (std::abs(w.position.x - q.position.x) > 1e-4f || std::abs(w.orientation.y - q.orientation.y) > 1e-3f)
            fail(std::string(name) + ": wrist not on the wrist source's pose");
        else
            std::printf("  ok: hand placed on the wrist source's pose\n");
    }
    if (!tracked)
        fail(std::string(name) + ": joints not marked tracked");
}

void report_joints()
{
    const auto& names = joint_names();
    for (const char* collection : { "pandora_glove_joints_left", "pandora_glove_joints_right" })
    {
        const auto& samples = g.joints[collection];
        std::printf("\n%s: %zu samples", collection, samples.size());
        if (samples.empty())
        {
            std::printf("\n");
            fail(std::string(collection) + " never pushed");
            continue;
        }
        float max_abs = 0;
        std::vector<float> lo(names.size(), 1e9f), hi(names.size(), -1e9f);
        bool names_ok = true;
        for (const auto& s : samples)
        {
            names_ok = names_ok && s.names == names;
            for (size_t i = 0; i < s.positions.size() && i < names.size(); ++i)
            {
                max_abs = std::max(max_abs, std::abs(s.positions[i]));
                lo[i] = std::min(lo[i], s.positions[i]);
                hi[i] = std::max(hi[i], s.positions[i]);
            }
        }
        std::printf(" from %s, largest |angle| %.2f rad\n", samples.back().device.c_str(), max_abs);
        std::printf("  index_mcp_flexion %.2f..%.2f rad, thumb_cmc_flexion %.2f..%.2f rad\n", lo[4], hi[4], lo[0], hi[0]);
        if (!names_ok)
            fail(std::string(collection) + ": joint names differ");
        else if (max_abs > 3.2f)
            fail(std::string(collection) + ": an angle beyond pi");
        else
            std::printf("  ok: 20 named joints in radians\n");
    }
}

} // namespace

int main(int argc, char** argv)
try
{
    double seconds = 10.0;
    double expect_rate = 0.0;
    std::vector<std::string> plugin_args;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a.rfind("--seconds=", 0) == 0)
            seconds = std::stod(a.substr(10));
        else if (a.rfind("--expect-rate=", 0) == 0)
            expect_rate = std::stod(a.substr(14));
        else
            plugin_args.push_back(a);
    }
    if (std::none_of(plugin_args.begin(), plugin_args.end(),
                     [](const std::string& a) { return a.rfind("--plugin-root-id=", 0) == 0; }))
    {
        plugin_args.push_back("--plugin-root-id=pandora_glove_check");
    }
    std::vector<std::string> unknown;
    const PandoraGloveConfig config = parse_args(plugin_args, &unknown);
    for (const auto& u : unknown)
        std::printf("unknown plugin argument %s\n", u.c_str());

    g_start = Clock::now();
    auto factory = [](const core::PluginSessionRequirements& req, std::vector<std::shared_ptr<core::ITracker>> trackers)
    {
        g.requirements = req;
        event("session: schema_push=" + std::to_string(req.schema_push) +
              " hand_push=" + std::to_string(req.hand_tracking_push) + " wrist_pull=" +
              std::to_string(req.wrist_tracking_pull.has_value()) + " trackers=" + std::to_string(trackers.size()));
        return std::unique_ptr<core::IPluginSession>(std::make_unique<FakeSession>());
    };
    {
        PandoraGlovePlugin plugin(config, factory);
        const auto period = plugin.tick_period();
        auto next = Clock::now();
        while (now_s() < seconds)
        {
            plugin.update();
            next += period;
            std::this_thread::sleep_until(next);
        }
    }

    std::printf("\n01 Glove plugin session check, %.0f s, rate %s, wrist mode %d\n", seconds,
                config.rate_hz > 0 ? std::to_string(static_cast<int>(config.rate_hz)).c_str() : "max",
                static_cast<int>(config.wrist));
    std::printf("\ntimeline:\n");
    for (const auto& line : g.timeline)
        std::printf("  %s\n", line.c_str());
    report_hand(true, config, seconds, expect_rate);
    report_hand(false, config, seconds, expect_rate);
    if (config.joints)
        report_joints();
    if (config.haptics)
    {
        std::printf("\nhaptics: %zu reads of the scripted commands (each motor %.1f s at %.1f from %.1f s, then none)\n",
                    g.haptic_reads, kHapticStep, kHapticLevel, kHapticStart);
        if (g.haptic_reads == 0)
            fail("haptic commands never read");
    }
    if (g.status.empty())
        fail("no device status published");
    std::printf("\nRESULT: %s (%d failures)\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
}
catch (const std::exception& e)
{
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
}
