// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "pandora_haptics.hpp"

#include <algorithm>
#include <cmath>

namespace plugins
{
namespace pandora_glove
{

std::optional<Amplitudes> amplitudes_from_powers(const float* values, std::size_t count)
{
    if (values == nullptr || (count != kMotorCount - 1 && count != kMotorCount))
    {
        return std::nullopt;
    }
    Amplitudes out{};
    for (std::size_t i = 0; i < count; ++i)
    {
        if (!std::isfinite(values[i]))
        {
            return std::nullopt;
        }
        const float power = std::clamp(values[i], 0.0f, 1.0f);
        out[i] = static_cast<uint8_t>(std::lround(power * 127.0f));
    }
    return out;
}

HapticStream::Action HapticStream::update(const Amplitudes& target, Clock::time_point now)
{
    const bool silent = std::all_of(target.begin(), target.end(), [](uint8_t a) { return a == 0; });
    if (silent)
    {
        if (!m_active)
        {
            return Action::None;
        }
        m_active = false;
        m_sent = target;
        m_sent_at = now;
        return Action::Stop;
    }
    const auto since = now - m_sent_at;
    if (m_active && since < kMinInterval)
    {
        return Action::None;
    }
    // Refresh well before the hold runs out so a steady level has no gaps.
    if (m_active && target == m_sent && since < m_hold / 2)
    {
        return Action::None;
    }
    m_active = true;
    m_sent = target;
    m_sent_at = now;
    return Action::Play;
}

} // namespace pandora_glove
} // namespace plugins
