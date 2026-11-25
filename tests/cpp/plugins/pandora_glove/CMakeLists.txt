# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# The plugin's pure parts; needs neither libpandora nor a runtime.
add_executable(test_pandora_glove
    test_pandora_glove.cpp
    ${CMAKE_SOURCE_DIR}/src/plugins/pandora_glove/pandora_glove_config.cpp
    ${CMAKE_SOURCE_DIR}/src/plugins/pandora_glove/pandora_hand_mapping.cpp
    ${CMAKE_SOURCE_DIR}/src/plugins/pandora_glove/pandora_haptics.cpp
)
target_include_directories(test_pandora_glove PRIVATE
    ${CMAKE_SOURCE_DIR}/src/plugins/pandora_glove
)
target_link_libraries(test_pandora_glove PRIVATE OpenXR::headers)
add_test(NAME pandora_glove_unit COMMAND test_pandora_glove)
