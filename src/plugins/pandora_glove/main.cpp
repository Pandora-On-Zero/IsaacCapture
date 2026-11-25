// SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "pandora_glove_config.hpp"
#include "pandora_glove_plugin.hpp"

#include <log_bridge/logger.hpp>
#include <plugin_utils/openxr_plugin_session.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <string>
#include <thread>
#include <vector>

using namespace plugins::pandora_glove;

namespace
{

static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "lock-free atomic bool is required for signal safety");
std::atomic<bool> g_stop_requested{ false };

void signal_handler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        g_stop_requested.store(true, std::memory_order_relaxed);
    }
}

} // namespace

int main(int argc, char** argv)
try
{
    auto logger = isaaccapture::Logger::get("isaaccapture.plugins.pandora_glove.main");
    std::vector<std::string> unknown;
    const PandoraGloveConfig config = parse_args(std::vector<std::string>(argv + 1, argv + argc), &unknown);
    for (const auto& arg : unknown)
    {
        logger->warn("ignoring unknown argument '{}'", arg);
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    logger->info("01 Glove plugin starting");
    PandoraGlovePlugin plugin(config, plugin_utils::make_openxr_plugin_session_factory("PandoraGlove"));
    logger->info("Plugin running. Press Ctrl+C to stop.");

    const auto period = plugin.tick_period();
    while (!g_stop_requested.load(std::memory_order_relaxed))
    {
        const auto start = std::chrono::steady_clock::now();
        plugin.update();
        std::this_thread::sleep_until(start + period);
    }
    return 0;
}
catch (const std::exception& e)
{
    isaaccapture::Logger::get("isaaccapture.plugins.pandora_glove.main")->error("{}: {}", argv[0], e.what());
    return 1;
}
catch (...)
{
    isaaccapture::Logger::get("isaaccapture.plugins.pandora_glove.main")->error("{}: unknown error", argv[0]);
    return 1;
}
