# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# 01 Glove plugin. Needs libpandora, the 01 Glove's C library. By default the
# published pandora-c-<PANDORA_C_VERSION>-linux-<arch> package is fetched from
# the core-rs-v<version> release of Pandora-On-Zero/Pandora-Release and its
# SHA-256 checked. To use a package already extracted instead:
#   cmake -B build -DBUILD_PLUGIN_PANDORA_GLOVE=ON -DPANDORA_C_DIR=/path/to/pandora-c
cmake_minimum_required(VERSION 3.20)

include(GNUInstallDirs)

set(PANDORA_C_VERSION "0.4.0" CACHE STRING "libpandora release fetched when PANDORA_C_DIR is empty")
set(PANDORA_C_DIR "" CACHE PATH "Extracted pandora-c package (include/ and lib/); empty fetches the published one")

# The published packages' SHA-256 (the release's SHA256SUMS).
set(_pandora_c_sha256_0.4.0_linux-x86_64 f9c152e944df796b86634a67af65f429060dcfcddae6870462592645ca5e734f)
set(_pandora_c_sha256_0.4.0_linux-aarch64 a40b2e0fb96a63ad6068424938834881c67216a67e78f01564122cfd4c7b30ad)

if(PANDORA_C_DIR)
    set(_pandora_c_root "${PANDORA_C_DIR}")
else()
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
        set(_pandora_c_platform linux-aarch64)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
        set(_pandora_c_platform linux-x86_64)
    else()
        message(FATAL_ERROR "No published libpandora for ${CMAKE_SYSTEM_PROCESSOR}; set PANDORA_C_DIR to a pandora-c package")
    endif()
    set(_pandora_c_name "pandora-c-${PANDORA_C_VERSION}-${_pandora_c_platform}")
    set(_pandora_c_root "${CMAKE_CURRENT_BINARY_DIR}/${_pandora_c_name}")
    if(NOT EXISTS "${_pandora_c_root}/include/pandora/pandora.h")
        set(_pandora_c_url "https://github.com/Pandora-On-Zero/Pandora-Release/releases/download/core-rs-v${PANDORA_C_VERSION}/${_pandora_c_name}.tar.gz")
        set(_pandora_c_archive "${CMAKE_CURRENT_BINARY_DIR}/${_pandora_c_name}.tar.gz")
        set(_pandora_c_sha256 "${_pandora_c_sha256_${PANDORA_C_VERSION}_${_pandora_c_platform}}")
        if(_pandora_c_sha256)
            set(_pandora_c_expected EXPECTED_HASH "SHA256=${_pandora_c_sha256}")
        else()
            message(WARNING "No SHA-256 known for ${_pandora_c_name}; fetching it unchecked")
            set(_pandora_c_expected "")
        endif()
        message(STATUS "Fetching libpandora: ${_pandora_c_url}")
        file(DOWNLOAD "${_pandora_c_url}" "${_pandora_c_archive}" ${_pandora_c_expected}
             TLS_VERIFY ON STATUS _pandora_c_status)
        list(GET _pandora_c_status 0 _pandora_c_code)
        if(NOT _pandora_c_code EQUAL 0)
            message(FATAL_ERROR "Could not fetch ${_pandora_c_url}: ${_pandora_c_status}. Set PANDORA_C_DIR to a pandora-c package instead.")
        endif()
        file(ARCHIVE_EXTRACT INPUT "${_pandora_c_archive}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
endif()

# Found again whenever the package changes (another PANDORA_C_DIR or version).
if(NOT "${_pandora_c_root}" STREQUAL "${PANDORA_C_FOUND_IN}")
    unset(PANDORA_C_INCLUDE_DIR CACHE)
    unset(PANDORA_C_LIB CACHE)
    set(PANDORA_C_FOUND_IN "${_pandora_c_root}" CACHE INTERNAL "The pandora-c package PANDORA_C_LIB was found in")
endif()
find_path(PANDORA_C_INCLUDE_DIR pandora/pandora.h HINTS "${_pandora_c_root}/include" NO_DEFAULT_PATH)
find_library(PANDORA_C_LIB NAMES pandora HINTS "${_pandora_c_root}/lib" NO_DEFAULT_PATH)
if(NOT PANDORA_C_INCLUDE_DIR OR NOT PANDORA_C_LIB)
    message(FATAL_ERROR "No libpandora in ${_pandora_c_root} (include/pandora/pandora.h, lib/libpandora.so)")
endif()
message(STATUS "libpandora: ${PANDORA_C_LIB}")
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
