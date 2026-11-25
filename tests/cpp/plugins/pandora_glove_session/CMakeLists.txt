# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# The plugin with libpandora against a capturing session instead of CloudXR.
# run_session_checks.sh drives it with the stand-in receiver; it becomes a CTest
# when PANDORA_RECORDINGS_DIR names libpandora's golden recordings.
set(_PLUGIN_DIR ${CMAKE_SOURCE_DIR}/src/plugins/pandora_glove)
find_package(Threads REQUIRED)
add_executable(pandora_glove_session_check
    pandora_glove_session_check.cpp
    ${_PLUGIN_DIR}/pandora_glove_plugin.cpp
    ${_PLUGIN_DIR}/pandora_glove_config.cpp
    ${_PLUGIN_DIR}/pandora_hand_mapping.cpp
    ${_PLUGIN_DIR}/pandora_haptics.cpp
)
target_include_directories(pandora_glove_session_check PRIVATE ${_PLUGIN_DIR} ${PANDORA_C_INCLUDE_DIR})
target_link_libraries(pandora_glove_session_check PRIVATE
    Teleop::plugin_utils
    deviceio::deviceio_trackers
    pusherio::pusherio
    oxr::oxr_utils
    isaacteleop_schema
    log_bridge::log_bridge_core
    ${PANDORA_C_LIB}
    Threads::Threads
)
set_target_properties(pandora_glove_session_check PROPERTIES BUILD_RPATH "${PANDORA_C_LIBRARY_DIR}")

set(PANDORA_RECORDINGS_DIR "" CACHE PATH "libpandora golden recordings (shared/golden) for the session checks")
if(PANDORA_RECORDINGS_DIR)
    add_test(NAME pandora_glove_session
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/run_session_checks.sh
            $<TARGET_FILE:pandora_glove_session_check> ${PANDORA_RECORDINGS_DIR})
endif()
