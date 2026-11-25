// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace plugins
{
namespace pandora_glove
{

// The glove's six LRA motors: thumb, index, middle, ring, pinky, back of hand.
inline constexpr std::size_t kMotorCount = 6;
using Amplitudes = std::array<uint8_t, kMotorCount>;

// HapticCommand values to drive levels (0 stop, 1..127). Five values are
// thumb..pinky (Isaac's FingerPowerVector); a sixth drives the back of the
// hand. Values are clamped to [0, 1]; a non-finite value or another count
// rejects the command.
std::optional<Amplitudes> amplitudes_from_powers(const float* values, std::size_t count);

// When to send one glove's streamed levels. The glove holds a level for the
// duration it was sent with and takes at most one frame per 5 ms.
class HapticStream
{
public:
    using Clock = std::chrono::steady_clock;

    enum class Action
    {
        None,
        Play, //!< send amplitudes() with hold() per motor
        Stop, //!< stop every motor
    };

    explicit HapticStream(std::chrono::milliseconds hold) : m_hold(hold)
    {
    }

    Action update(const Amplitudes& target, Clock::time_point now);

    const Amplitudes& amplitudes() const
    {
        return m_sent;
    }
    std::chrono::milliseconds hold() const
    {
        return m_hold;
    }

    static constexpr std::chrono::milliseconds kMinInterval{ 5 };

private:
    std::chrono::milliseconds m_hold;
    Amplitudes m_sent{};
    Clock::time_point m_sent_at{};
    bool m_active = false;
};

} // namespace pandora_glove
} // namespace plugins
