// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "pandora_glove_plugin.hpp"

#include <flatbuffers/flatbuffers.h>
#include <oxr_utils/os_time.hpp>
#include <schema/joint_state_generated.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace plugins
{
namespace pandora_glove
{

namespace
{

constexpr const char* kJointsLeftCollection = "pandora_glove_joints_left";
constexpr const char* kJointsRightCollection = "pandora_glove_joints_right";
// Must agree with JointStateTracker's max_flatbuffer_size (src/core/deviceio_trackers/trackers.toml).
constexpr size_t kJointFlatbufferSize = 4096;
constexpr auto kReopenInterval = std::chrono::seconds(1);
constexpr auto kRateLogInterval = std::chrono::seconds(5);
constexpr auto kGloveTextInterval = std::chrono::milliseconds(500);
constexpr double kPi = 3.14159265358979323846;

// Test poses without a tracker, in the STAGE space: hands at a table in front
// of the origin, palms down, fingers forward (the identity OpenXR wrist).
constexpr XrPosef kFixedLeftWrist{ { 0.0f, 0.0f, 0.0f, 1.0f }, { -0.12f, 1.0f, -0.35f } };
constexpr XrPosef kFixedRightWrist{ { 0.0f, 0.0f, 0.0f, 1.0f }, { 0.12f, 1.0f, -0.35f } };

const char* side(bool is_left)
{
    return is_left ? "left" : "right";
}

std::optional<core::WristTrackingSourceConfig> wrist_pull(const PandoraGloveConfig& config)
{
    core::WristTrackingSourceConfig out;
    out.left_aim_to_wrist = config.left_aim_to_wrist;
    out.right_aim_to_wrist = config.right_aim_to_wrist;
    switch (config.wrist)
    {
    case WristMode::Auto:
        out.mode = core::WristTrackingSourceMode::Auto;
        return out;
    case WristMode::HandTracking:
        out.mode = core::WristTrackingSourceMode::HandTracking;
        return out;
    case WristMode::Controller:
        out.mode = core::WristTrackingSourceMode::Controller;
        return out;
    case WristMode::Fixed:
    case WristMode::Imu:
    case WristMode::Se3:
        return std::nullopt;
    }
    return std::nullopt;
}

// A string answer from libpandora (pandora_incompatible, pandora_unprofiled).
template <typename Fn>
std::string read_text(Fn fn)
{
    std::vector<char> buf(256);
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        size_t needed = 0;
        const pandora_status st = fn(buf.data(), buf.size(), &needed);
        if (st == PANDORA_OK)
        {
            return std::string(buf.data());
        }
        if (st != PANDORA_ERR_TOO_SMALL)
        {
            return {};
        }
        buf.resize(needed + 1);
    }
    return {};
}

} // namespace

PandoraGlovePlugin::PandoraGlovePlugin(const PandoraGloveConfig& config, const core::PluginSessionFactory& session_factory)
    : m_config(config)
{
    m_hands[0].is_left = true;
    m_hands[1].is_left = false;
    std::vector<std::shared_ptr<core::ITracker>> trackers;
    for (auto& hand : m_hands)
    {
        hand.haptics = HapticStream(m_config.haptic_hold);
        pandora_hand_placer* placer = nullptr;
        if (pandora_hand_placer_new(hand.is_left ? PANDORA_LEFT : PANDORA_RIGHT, -1.0, &placer) != PANDORA_OK)
        {
            throw std::runtime_error(std::string("PandoraGlovePlugin: ") + pandora_last_error());
        }
        hand.placer.reset(placer);
        if (m_config.wrist == WristMode::Se3)
        {
            hand.se3 = std::make_shared<core::Se3Tracker>(hand.is_left ? m_config.se3_left : m_config.se3_right);
            trackers.push_back(hand.se3);
        }
    }

    if (m_config.haptics)
    {
        // A command older than a few holds is the producer having gone away: stop the motors.
        m_haptic_reader = std::make_shared<core::HapticCommandReaderTracker>(
            m_config.haptic_collection, core::HapticCommandReaderTracker::DEFAULT_MAX_PAYLOAD_SIZE,
            std::chrono::nanoseconds(m_config.haptic_hold) * 4);
        trackers.push_back(m_haptic_reader);
    }

    core::PluginSessionRequirements requirements;
    requirements.schema_push = m_config.joints || m_config.plugin_root_id.has_value();
    requirements.hand_tracking_push = m_config.hands;
    requirements.wrist_tracking_pull = m_config.hands ? wrist_pull(m_config) : std::nullopt;
    m_session = session_factory(requirements, std::move(trackers));
    if (!m_session)
    {
        throw std::runtime_error("PandoraGlovePlugin: the plugin session factory returned no session");
    }

    if (m_config.plugin_root_id)
    {
        m_status = std::make_unique<plugin_utils::PluginDeviceStatusPublisher>(*m_session, *m_config.plugin_root_id);
    }
    if (m_config.joints)
    {
        for (auto& hand : m_hands)
        {
            hand.joint_channel = m_session->create_schema_push_channel(core::SchemaPusherConfig{
                .collection_id = hand.is_left ? kJointsLeftCollection : kJointsRightCollection,
                .max_flatbuffer_size = kJointFlatbufferSize,
                .tensor_identifier = "joint_state",
                .localized_name = hand.is_left ? "01 Glove joints left" : "01 Glove joints right" });
        }
    }

    m_logger->info("datasets: hand={} haptic={} joints={}; rate {}; libpandora ABI {}", m_config.hands ? "on" : "off",
                   m_config.haptics ? "on" : "off", m_config.joints ? "on" : "off",
                   m_config.rate_hz > 0.0 ? std::to_string(static_cast<int>(m_config.rate_hz)) + " Hz" : "every frame",
                   pandora_abi_version());
    if (pandora_abi_version() != PANDORA_ABI_VERSION || pandora_hand_frame_size() != sizeof(pandora_hand_frame) ||
        pandora_event_size() != sizeof(pandora_event))
    {
        throw std::runtime_error("PandoraGlovePlugin: libpandora does not match the pandora.h this plugin was built with");
    }
    m_rate_window_start = std::chrono::steady_clock::now();
    open_receiver_if_needed();
}

PandoraGlovePlugin::~PandoraGlovePlugin()
{
    for (auto& hand : m_hands)
    {
        if (m_rx != nullptr && !hand.device.empty())
        {
            pandora_stop_haptics(m_rx, hand.device.c_str());
        }
    }
    close_receiver("plugin stopping");
    // Push channels before the session that owns their transport.
    for (auto& hand : m_hands)
    {
        hand.hand_channel.reset();
        hand.joint_channel.reset();
    }
    m_status.reset();
    m_session.reset();
}

std::chrono::nanoseconds PandoraGlovePlugin::tick_period() const
{
    // Twice the push rate, so frames spaced by the glove's clock are not
    // skipped by jitter; every-frame mode polls at 1 kHz.
    const double hz = m_config.rate_hz > 0.0 ? std::min(1000.0, 2.0 * m_config.rate_hz) : 1000.0;
    return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
}

void PandoraGlovePlugin::open_receiver_if_needed()
{
    if (m_rx != nullptr)
    {
        if (pandora_is_running(m_rx))
        {
            return;
        }
        close_receiver("reading stopped");
    }
    const auto now = std::chrono::steady_clock::now();
    if (now < m_next_open_attempt)
    {
        return;
    }
    m_next_open_attempt = now + kReopenInterval;

    pandora_options options;
    pandora_options_init(&options);
    options.port = m_config.port.empty() ? nullptr : m_config.port.c_str();
    options.profiles_dir = m_config.profiles_dir.empty() ? nullptr : m_config.profiles_dir.c_str();
    options.read_glove_profiles = m_config.read_glove_profiles ? 1 : 0;
    pandora_receiver* rx = nullptr;
    if (pandora_receiver_open_with(&options, &rx) != PANDORA_OK)
    {
        const std::string error = std::string(pandora_last_error_code()) + ": " + pandora_last_error();
        if (error != m_receiver_error)
        {
            m_logger->warn("receiver not open ({}); retrying every second", error);
            m_receiver_error = error;
        }
        return;
    }
    if (pandora_subscribe_frames(rx, m_config.rate_hz) != PANDORA_OK)
    {
        m_logger->error("could not subscribe to frames: {}", pandora_last_error());
        pandora_receiver_close(rx);
        return;
    }
    m_rx = rx;
    m_receiver_error.clear();
    m_logger->info("receiver open{}{}", m_config.port.empty() ? "" : " on ", m_config.port);
}

void PandoraGlovePlugin::close_receiver(const char* why)
{
    if (m_rx == nullptr)
    {
        return;
    }
    m_logger->info("receiver closed: {}", why);
    pandora_receiver_close(m_rx);
    m_rx = nullptr;
    for (auto& hand : m_hands)
    {
        hand.hand_channel.reset();
        hand.has_frame = false;
        hand.last_seq = -1;
        // Read the offset again once the glove is back: its profile may have changed.
        hand.mount_device.clear();
    }
}

void PandoraGlovePlugin::drain_events()
{
    std::vector<char> text(4096);
    for (;;)
    {
        pandora_event event{};
        pandora_status st = pandora_next_event(m_rx, 0.0, &event, text.data(), text.size());
        if (st == PANDORA_ERR_TOO_SMALL)
        {
            text.resize(event.text_len + 1);
            st = pandora_next_event(m_rx, 0.0, &event, text.data(), text.size());
        }
        if (st != PANDORA_OK)
        {
            return;
        }
        if (event.kind == PANDORA_EVENT_STATUS)
        {
            m_logger->info("{}: {} ({})", event.device, event.code, text.data());
        }
        else if (event.kind == PANDORA_EVENT_ENDED)
        {
            m_logger->warn("receiver reading ended: {} ({})", event.code, text.data());
        }
        // PANDORA_EVENT_MESSAGE carries each frame as JSON; the plugin reads
        // pandora_latest_hand instead, so those are only taken off the queue.
    }
}

XrPosef PandoraGlovePlugin::wrist_pose(Hand& hand, const pandora_hand_frame& frame, int64_t now_ns, bool& tracked)
{
    tracked = false;
    switch (m_config.wrist)
    {
    case WristMode::Fixed:
        tracked = true;
        return hand.is_left ? kFixedLeftWrist : kFixedRightWrist;
    case WristMode::Imu:
    {
        XrPosef pose = hand.is_left ? kFixedLeftWrist : kFixedRightWrist;
        if (frame.has_wrist)
        {
            if (!hand.imu_started)
            {
                std::copy(std::begin(frame.wrist_deg), std::end(frame.wrist_deg), hand.imu_start);
                hand.imu_started = true;
            }
            pose.orientation = imu_rotation_since(frame.wrist_deg, hand.imu_start);
        }
        tracked = true;
        return pose;
    }
    case WristMode::Se3:
    {
        std::optional<XrPosef> tracker;
        const auto& data = hand.se3->get_data(m_session->pull_channel());
        const core::Se3TrackerPose* sample = data.get();
        if (sample != nullptr && sample->is_valid() && sample->pose() != nullptr)
        {
            const core::Pose& p = *sample->pose();
            tracker = XrPosef{ { p.orientation().x(), p.orientation().y(), p.orientation().z(), p.orientation().w() },
                               { p.position().x(), p.position().y(), p.position().z() } };
        }
        return placed_wrist(hand, frame, tracker, tracked);
    }
    default:
        break;
    }
    const core::WristTrackingSample sample = m_session->pull_channel().wrist_tracking_source().query(
        hand.is_left ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT, now_ns);
    if (uses_glove_offset(hand))
    {
        // The source answers the controller's aim pose itself (an identity offset):
        // the glove's stored offset and libpandora's placer take it from there.
        return placed_wrist(hand, frame, sample.tracked ? std::optional<XrPosef>(sample.pose) : std::nullopt, tracked);
    }
    if (sample.valid)
    {
        hand.last_wrist = sample.pose;
        tracked = sample.tracked;
    }
    return hand.last_wrist;
}

bool PandoraGlovePlugin::uses_glove_offset(const Hand& hand) const
{
    return m_config.wrist == WristMode::Controller &&
           !(hand.is_left ? m_config.left_aim_given : m_config.right_aim_given);
}

XrPosef PandoraGlovePlugin::placed_wrist(Hand& hand,
                                         const pandora_hand_frame& frame,
                                         const std::optional<XrPosef>& tracker,
                                         bool& tracked)
{
    const double t_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_start).count();
    const PandoraPose tracker_pose = tracker ? to_pandora_pose(*tracker) : PandoraPose{};
    PandoraPose root{};
    int32_t state = PANDORA_PLACEMENT_NONE;
    if (pandora_hand_placer_update(hand.placer.get(), t_s, tracker ? tracker_pose.data() : nullptr,
                                   frame.has_wrist ? frame.wrist_deg : nullptr, hand.mount.data(), root.data(),
                                   &state) != PANDORA_OK)
    {
        m_logger->warn("placing the {} hand failed: {}", side(hand.is_left), pandora_last_error());
        return hand.last_wrist;
    }
    if (state != PANDORA_PLACEMENT_NONE)
    {
        hand.last_wrist = compose_poses(from_pandora_pose(root), wrist_in_root(frame.bones, hand.is_left));
    }
    // Held through a loss, the joints stay valid but are not tracked.
    tracked = state == PANDORA_PLACEMENT_TRACKED;
    return hand.last_wrist;
}

void PandoraGlovePlugin::load_mount(Hand& hand)
{
    hand.mount_device = hand.device;
    hand.mount = PandoraPose{ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
    const std::string reply = read_text([this, &hand](char* out, size_t len, size_t* needed)
                                        { return pandora_tracker_offset(m_rx, hand.device.c_str(), out, len, needed); });
    const auto mount = parse_tracker_offset_reply(reply);
    if (!mount)
    {
        m_logger->warn(
            "{}: no tracker offset ({}); the hand sits on the tracker itself", hand.device, pandora_last_error());
        return;
    }
    hand.mount = mount->pose;
    m_logger->info("{}: {} tracker offset", hand.device, mount->stored ? "its stored" : "no stored, so a zero");
}

void PandoraGlovePlugin::push_hand(Hand& hand, const pandora_hand_frame& frame, int64_t now_ns)
{
    HandJoints joints = map_bones_to_openxr(frame.bones, hand.is_left);
    bool tracked = false;
    const XrPosef wrist = wrist_pose(hand, frame, now_ns, tracked);
    compose_on_wrist(joints, wrist, tracked);
    if (!hand.hand_channel)
    {
        hand.hand_channel =
            m_session->create_hand_tracking_push_channel(hand.is_left ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT);
        m_logger->info("{} hand active ({})", side(hand.is_left), hand.device);
    }
    hand.hand_channel->push(joints.data(), now_ns);
    ++hand.frames_pushed;
}

void PandoraGlovePlugin::push_joints(Hand& hand, const pandora_hand_frame& frame, int64_t now_ns)
{
    core::JointStateOutputT out;
    out.device_id = hand.device;
    out.has_velocity = false;
    out.has_effort = false;
    out.ee_pose_valid = false;
    const auto& names = joint_names();
    for (size_t i = 0; i < names.size(); ++i)
    {
        auto joint = std::make_shared<core::JointStateT>();
        joint->name = names[i];
        joint->position = static_cast<float>(frame.angles_deg[i / 4][i % 4] * kPi / 180.0);
        joint->valid = true;
        out.joints.push_back(std::move(joint));
    }
    flatbuffers::FlatBufferBuilder builder(kJointFlatbufferSize);
    builder.Finish(core::JointStateOutput::Pack(builder, &out));
    hand.joint_channel->push_buffer(builder.GetBufferPointer(), builder.GetSize(), now_ns, now_ns);
}

void PandoraGlovePlugin::update_hand(Hand& hand, int64_t now_ns)
{
    const auto now = std::chrono::steady_clock::now();
    pandora_hand_frame frame{};
    if (m_rx != nullptr && pandora_latest_hand(m_rx, hand.is_left ? PANDORA_LEFT : PANDORA_RIGHT, &frame) == PANDORA_OK &&
        frame.seq != hand.last_seq)
    {
        hand.last_seq = frame.seq;
        hand.last_new_frame = now;
        hand.has_frame = true;
        hand.device.assign(frame.device, strnlen(frame.device, sizeof(frame.device)));
        if (hand.device != hand.mount_device && (m_config.wrist == WristMode::Se3 || uses_glove_offset(hand)))
        {
            load_mount(hand);
        }
        if (m_config.hands)
        {
            push_hand(hand, frame, now_ns);
        }
        if (hand.joint_channel)
        {
            push_joints(hand, frame, now_ns);
        }
        return;
    }
    // No new frame: past the stale limit the hand goes inactive at the
    // receiver instead of freezing in place.
    if (hand.hand_channel && now - hand.last_new_frame > m_config.stale_after)
    {
        hand.hand_channel.reset();
        m_logger->info("{} hand inactive: no frame for {} ms", side(hand.is_left), m_config.stale_after.count());
    }
}

void PandoraGlovePlugin::play_haptics(Hand& hand)
{
    if (m_rx == nullptr || hand.device.empty())
    {
        return;
    }
    Amplitudes target{};
    const auto& sample = m_haptic_reader->get_data(m_session->pull_channel(), side(hand.is_left));
    const core::HapticCommand* command = sample.get();
    if (command != nullptr && command->values() != nullptr)
    {
        const auto amplitudes = amplitudes_from_powers(command->values()->data(), command->values()->size());
        if (amplitudes)
        {
            target = *amplitudes;
        }
        else
        {
            m_logger->warn("ignoring a {} HapticCommand with {} values: want 5 (thumb..pinky) or 6 (+back of hand)",
                           side(hand.is_left), command->values()->size());
        }
    }
    switch (hand.haptics.update(target, std::chrono::steady_clock::now()))
    {
    case HapticStream::Action::Play:
    {
        std::array<uint16_t, kMotorCount> durations{};
        durations.fill(static_cast<uint16_t>(hand.haptics.hold().count()));
        if (pandora_play_rtp(m_rx, hand.device.c_str(), hand.haptics.amplitudes().data(), durations.data()) != PANDORA_OK)
        {
            m_logger->warn("haptics to {} refused: {}", hand.device, pandora_last_error());
        }
        break;
    }
    case HapticStream::Action::Stop:
        pandora_stop_haptics(m_rx, hand.device.c_str());
        break;
    case HapticStream::Action::None:
        break;
    }
}

void PandoraGlovePlugin::publish_status()
{
    if (!m_status)
    {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= m_glove_text_next)
    {
        m_glove_text_next = now + kGloveTextInterval;
        m_unprofiled = m_rx ? read_text([this](char* out, size_t len, size_t* needed)
                                        { return pandora_unprofiled(m_rx, out, len, needed); }) :
                              std::string();
        m_incompatible = m_rx ? read_text([this](char* out, size_t len, size_t* needed)
                                          { return pandora_incompatible(m_rx, out, len, needed); }) :
                                std::string();
    }
    const std::string& unprofiled = m_unprofiled;
    const std::string& incompatible = m_incompatible;

    std::vector<plugin_utils::PluginDeviceStatusEntry> entries;
    for (const auto& hand : m_hands)
    {
        plugin_utils::PluginDeviceStatusEntry entry{ .path = hand.is_left ? "/hand/left" : "/hand/right" };
        const bool fresh = hand.has_frame && now - hand.last_new_frame <= m_config.stale_after;
        if (!m_config.hands)
        {
            entry.state = core::PluginDeviceState_DISABLED;
            entry.reason = core::PluginDeviceReason_DISABLED_BY_CONFIGURATION;
        }
        else if (m_rx == nullptr)
        {
            entry.state = core::PluginDeviceState_DISCONNECTED;
            entry.reason = core::PluginDeviceReason_NO_HARDWARE_SIGNAL;
            entry.error = m_receiver_error.empty() ? "no receiver" : m_receiver_error;
        }
        else if (fresh)
        {
            entry.state = core::PluginDeviceState_CONNECTED;
        }
        else if (hand.has_frame)
        {
            entry.state = core::PluginDeviceState_DEGRADED;
            entry.reason = core::PluginDeviceReason_NO_CURRENT_DATA;
        }
        else if (!incompatible.empty() && incompatible != "{}")
        {
            entry.state = core::PluginDeviceState_FAILED;
            entry.reason = core::PluginDeviceReason_DEVICE_ERROR;
            entry.error = "firmware this libpandora cannot read: " + incompatible;
        }
        else
        {
            entry.state = core::PluginDeviceState_DISCONNECTED;
            entry.reason = core::PluginDeviceReason_NO_HARDWARE_SIGNAL;
            if (!unprofiled.empty())
            {
                entry.error = "gloves with no calibration (run `pandora calibrate`): " + unprofiled;
            }
        }
        entries.push_back(std::move(entry));
    }
    try
    {
        m_status->publish_if_changed(entries, core::os_monotonic_now_ns());
        m_status_error_logged = false;
    }
    catch (const std::exception& e)
    {
        if (!m_status_error_logged)
        {
            m_logger->warn("device status publishing failed; tracking continues: {}", e.what());
            m_status_error_logged = true;
        }
    }
}

void PandoraGlovePlugin::log_rates()
{
    const auto now = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(now - m_rate_window_start).count();
    if (seconds < std::chrono::duration<double>(kRateLogInterval).count())
    {
        return;
    }
    const auto rate = [seconds](const Hand& hand)
    {
        return hand.device.empty() ?
                   std::string("none") :
                   hand.device + " " + std::to_string(static_cast<int>(hand.frames_pushed / seconds + 0.5)) + " Hz";
    };
    m_logger->info("pushed: left {}, right {}; receiver packets {}, dropped events {}", rate(m_hands[0]),
                   rate(m_hands[1]), m_rx ? pandora_packets(m_rx) : 0, m_rx ? pandora_dropped(m_rx) : 0);
    for (auto& hand : m_hands)
    {
        hand.frames_pushed = 0;
    }
    m_rate_window_start = now;
}

void PandoraGlovePlugin::update()
{
    m_session->pull_channel().update();
    open_receiver_if_needed();
    if (m_rx != nullptr)
    {
        drain_events();
    }
    const int64_t now_ns = core::os_monotonic_now_ns();
    for (auto& hand : m_hands)
    {
        update_hand(hand, now_ns);
        if (m_haptic_reader)
        {
            play_haptics(hand);
        }
    }
    publish_status();
    log_rates();
}

} // namespace pandora_glove
} // namespace plugins
