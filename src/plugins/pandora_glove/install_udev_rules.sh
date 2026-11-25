#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Install the 01 Glove receiver's udev rule so the plugin can open it without root.
set -euo pipefail

RULES="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/70-pandora-receiver.rules"
if [[ ! -f "$RULES" ]]; then
    echo "missing $RULES" >&2
    exit 1
fi
sudo install -m 0644 "$RULES" /etc/udev/rules.d/70-pandora-receiver.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Installed. Unplug the receiver and plug it back in."
