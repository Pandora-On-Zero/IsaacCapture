# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# 01 Glove plugin. Needs libpandora, the 01 Glove's C library: an extracted
# pandora-c-X.Y.Z-<platform> release package (include/pandora/pandora.h and
# lib/libpandora.so), named with
#   cmake -B build -DBUILD_PLUGIN_PANDORA_GLOVE=ON -DPANDORA_C_DIR=/path/to/pandora-c
cmake_minimum_required(VERSION 3.20)

include(GNUInstallDirs)

set(PANDORA_C_DIR "" CACHE PATH "Extracted pandora-c package (include/ and lib/)")
find_path(PANDORA_C_INCLUDE_DIR pandora/pandora.h HINTS "${PANDORA_C_DIR}/include" NO_DEFAULT_PATH)
find_library(PANDORA_C_LIB NAMES pandora HINTS "${PANDORA_C_DIR}/lib" NO_DEFAULT_PATH)
if(NOT PANDORA_C_INCLUDE_DIR OR NOT PANDORA_C_LIB)
    message(FATAL_ERROR "Set PANDORA_C_DIR to an extracted pandora-c package (include/pandora/pandora.h, lib/libpandora.so)")
endif()
get_filename_component(PANDORA_C_LIBRARY_DIR "${PANDORA_C_LIB}" DIRECTORY)

add_executable(pandora_glove_plugin
    main.cpp
    pandora_glove_plugin.cpp
    pandora_glove_plugin.hpp
    pandora_glove_config.cpp
    pandora_glove_config.hpp
    pandora_hand_mapping.cpp
    pandora_hand_mapping.hpp
    pandora_haptics.cpp
    pandora_haptics.hpp
)

find_package(Threads REQUIRED)

target_include_directories(pandora_glove_plugin PRIVATE ${PANDORA_C_INCLUDE_DIR})
target_link_libraries(pandora_glove_plugin PRIVATE
    Teleop::plugin_utils
    deviceio::deviceio_trackers
    pusherio::pusherio
    oxr::oxr_utils
    OpenXR::openxr_loader
    isaacteleop_schema
    log_bridge::log_bridge_core
    ${PANDORA_C_LIB}
    ${CMAKE_DL_LIBS}
    Threads::Threads
)

set_target_properties(pandora_glove_plugin PROPERTIES
    BUILD_WITH_INSTALL_RPATH NO
    SKIP_BUILD_RPATH NO
    BUILD_RPATH "$ORIGIN;${PANDORA_C_LIBRARY_DIR}"
    INSTALL_RPATH "$ORIGIN"
)

install(TARGETS pandora_glove_plugin
    RUNTIME DESTINATION plugins/pandora_glove
    COMPONENT pandora_glove
)
install(FILES plugin.yaml README.md 70-pandora-receiver.rules
    DESTINATION plugins/pandora_glove
    COMPONENT pandora_glove
)
install(PROGRAMS install_udev_rules.sh
    DESTINATION plugins/pandora_glove
    COMPONENT pandora_glove
)
install(FILES "${PANDORA_C_LIB}"
    DESTINATION plugins/pandora_glove
    COMPONENT pandora_glove
)
# A package without its licence file still builds.
get_filename_component(PANDORA_C_ROOT_DIR "${PANDORA_C_LIBRARY_DIR}" DIRECTORY)
install(FILES "${PANDORA_C_ROOT_DIR}/LICENSE.md"
    DESTINATION plugins/pandora_glove
    COMPONENT pandora_glove
    RENAME LICENSE.libpandora.md
    OPTIONAL
)
