<!--
SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# 01 Glove test environment

Checks the [01 Glove plugin](../../src/plugins/pandora_glove/README.md) end to
end, with or without a glove:

| Module | What it does |
| --- | --- |
| `fake_receiver.py` | A stand-in receiver: replays recorded gloves (libpandora's `shared/golden/*.json` files from the Pandora repository) over a pseudo-terminal at `--rate` packets a second, writes each glove's profile, and logs every command the plugin sends back (`--command-log`) |
| `check_hands` | Reads both hands and the joint-angle collections from Isaac for `--seconds`, reports each hand's update rate and checks the joints form a correct OpenXR hand |
| `haptic_sweep` | Buzzes each motor in turn through Isaac's haptic-glove path |
| `run_test_env.sh` | Starts the stand-in receiver (or uses a real one), the CloudXR service, the plugin and the DeviceIO live view |

Build with `-DBUILD_PLUGIN_PANDORA_GLOVE=ON`, install the wheel into `.venv`
as in [Running an example](../README.md#running-an-example), then from the
repository root:

```bash
uv pip install -e ./examples/deviceio_live_view ./examples/pandora_glove --no-sources
examples/pandora_glove/run_test_env.sh --recordings /path/to/Pandora/shared/golden --rate 200
```

Open the live view's URL (<http://localhost:8080>) to see both hands. In
another terminal, with the venv active:

```bash
python -m isaaccapture_examples.pandora_glove.check_hands --seconds 10 --expect-rate 200
python -m isaaccapture_examples.pandora_glove.haptic_sweep
```

The sweep's levels show in `/tmp/pandora_glove/fake_receiver.log` and
`commands.jsonl`. With `--real` instead of `--recordings`, the plugin opens
the receiver plugged in and the sweep buzzes the real glove.

The stand-in receiver restamps the recorded packets on the glove clock at
`--rate`, so the recorded motion plays faster than it was made; the recordings
keep only every second to twentieth packet.
