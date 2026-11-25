// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "pandora_glove_config.hpp"
#include "pandora_hand_mapping.hpp"
#include "pandora_haptics.hpp"

#include <deviceio_trackers/haptic_command_reader_tracker.hpp>
#include <deviceio_trackers/se3_tracker.hpp>
#include <log_bridge/logger.hpp>
#include <plugin_utils/plugin_device_status_publisher.hpp>
#include <pusherio/plugin_session.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

extern "C"
{
#include <pandora/pandora.h>
}

namespace plugins
{
namespace pandora_glove
{

// 01 Glove -> Isaac Teleop device plugin.
//
// Reads both gloves through libpandora (the receiver dongle, opened in this
// process), maps each glove's OpenVR skeleton to the 26 OpenXR joints and
// pushes them as hand tracking, places them on a wrist pose from
// plugin_utils (or a fixed/IMU pose for tests without a headset), pushes the
// 20 joint angles as JointStateOutput, and plays inbound HapticCommands on the
// glove's motors.
class PandoraGlovePlugin
{
public:
    PandoraGlovePlugin(const PandoraGloveConfig& config, const core::PluginSessionFactory& session_factory);
    ~PandoraGlovePlugin();

    PandoraGlovePlugin(const PandoraGlovePlugin&) = delete;
    PandoraGlovePlugin& operator=(const PandoraGlovePlugin&) = delete;

    //! One tick: pull, read the gloves, push, play haptics, publish status.
    void update();

    //! Seconds between ticks for the configured rate.
    std::chrono::nanoseconds tick_period() const;

private:
    struct Hand
    {
        bool is_left = false;
        std::string device; //!< the glove's id once a frame came
        int64_t last_seq = -1;
        std::chrono::steady_clock::time_point last_new_frame{};
        bool has_frame = false;
        bool imu_started = false;
        double imu_start[3] = { 0.0, 0.0, 0.0 };
        XrPosef last_wrist{ { 0.0f, 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f } };
        std::unique_ptr<core::IHandTrackingPushChannel> hand_channel;
        std::unique_ptr<core::ISchemaPushChannel> joint_channel;
        HapticStream haptics{ std::chrono::milliseconds(60) };
        uint64_t frames_pushed = 0;
        //! The tracker this glove is fixed to, in Se3 mode.
        std::shared_ptr<core::Se3Tracker> se3;
        //! libpandora's placement: the hand root on the tracker, held with the IMU when it is lost.
        std::unique_ptr<pandora_hand_placer, void (*)(pandora_hand_placer*)> placer{ nullptr, pandora_hand_placer_free };
        //! The glove's tracker offset (the root's pose in the tracker's frame) and whose it is.
        PandoraPose mount{ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
        std::string mount_device;
    };

    void open_receiver_if_needed();
    void close_receiver(const char* why);
    void drain_events();
    void update_hand(Hand& hand, int64_t now_ns);
    void push_hand(Hand& hand, const pandora_hand_frame& frame, int64_t now_ns);
    void push_joints(Hand& hand, const pandora_hand_frame& frame, int64_t now_ns);
    XrPosef wrist_pose(Hand& hand, const pandora_hand_frame& frame, int64_t now_ns, bool& tracked);
    //! The wrist from a tracker's pose (std::nullopt when not tracked this frame) through the glove's offset.
    XrPosef placed_wrist(Hand& hand, const pandora_hand_frame& frame, const std::optional<XrPosef>& tracker, bool& tracked);
    void load_mount(Hand& hand);
    bool uses_glove_offset(const Hand& hand) const;
    void play_haptics(Hand& hand);
    void publish_status();
    void log_rates();

    PandoraGloveConfig m_config;
    std::shared_ptr<spdlog::logger> m_logger =
        isaaccapture::Logger::get("isaaccapture.plugins.pandora_glove.PandoraGlovePlugin");

    pandora_receiver* m_rx = nullptr;
    std::string m_receiver_error;
    std::chrono::steady_clock::time_point m_next_open_attempt{};
    std::string m_incompatible;
    std::string m_unprofiled;
    std::chrono::steady_clock::time_point m_glove_text_next{};

    std::unique_ptr<core::IPluginSession> m_session;
    std::shared_ptr<core::HapticCommandReaderTracker> m_haptic_reader;
    std::unique_ptr<plugin_utils::PluginDeviceStatusPublisher> m_status;
    bool m_status_error_logged = false;

    std::array<Hand, 2> m_hands;
    std::chrono::steady_clock::time_point m_start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point m_rate_window_start{};
};

} // namespace pandora_glove
} // namespace plugins
