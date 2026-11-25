.. SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
.. SPDX-License-Identifier: Apache-2.0

01 Glove
========

A Linux plugin for Pandora-On-Zero's 01 Glove, a magnetic-sensing data glove
with six haptic motors per hand. The plugin reads both gloves through their USB
receiver with libpandora, the glove's C library, and publishes each hand as
OpenXR hand tracking. It also publishes each glove's joint angles, and plays
Isaac Teleop's haptic commands on the glove's motors.

.. contents:: On this page
   :local:
   :depth: 2

Data flow
---------

.. code-block:: text

   01 Glove ──BLE──► receiver ──USB──► libpandora ─┐
                                    (calibration,  │
                                     31-bone hand) ├─► pandora_glove plugin ──► CloudXR OpenXR runtime
   Optical hand / controller ──────────────────────┘   (wrist-pose fusion)       hand joints, joint angles
                                                        ◄── HapticCommand ──────  HapticSink

The glove measures its hand's shape and the wrist's orientation, not where the
hand is. The plugin places the shape on a wrist pose from optical hand tracking,
or on a tracker fixed to the glove (a controller, or an SE3 tracker stream such
as a VIVE Ultimate Tracker's) through the tracker offset the glove stores. While
that tracker is lost, the hand stays at its last position, turned by the
glove's wrist IMU for up to a second, then holds still; its joints stay valid
but are not marked tracked.

Prerequisites
-------------

- **Linux**, x86_64 or aarch64, with the toolchain in
  :doc:`/development/build_from_source/index`.
- **libpandora**, the 01 Glove's C library. Configuring fetches the published
  ``pandora-c-0.4.0-linux-<arch>`` package from the ``core-rs-v0.4.0`` release
  of `Pandora-On-Zero/Pandora-Release
  <https://github.com/Pandora-On-Zero/Pandora-Release/releases>`_ and checks its
  SHA-256. Offline, extract a package and pass ``-DPANDORA_C_DIR``.
- **01 Glove(s)** paired with an **01 receiver** plugged into the plugin host.
- **Each glove calibrated**, with ``pandora calibrate`` (``pip install
  pandora-sdk``) or with the 01 Glove app or Pandora-Web, which store the
  calibration on the glove.
- **CloudXR runtime** running on the plugin host.

Installation
------------

.. code-block:: bash

   cmake -B build -DBUILD_PLUGINS=ON -DBUILD_PLUGIN_PANDORA_GLOVE=ON
   cmake --build build --target pandora_glove_plugin
   cmake --install build --component pandora_glove
   ./install/plugins/pandora_glove/install_udev_rules.sh

The udev rule lets the plugin open the receiver without root. Unplug the
receiver and plug it back in after installing it.

Running the plugin
------------------

.. code-block:: bash

   python -m isaaccapture.cloudxr.service start --accept-eula   # runs in the background
   source ~/.cloudxr/run/cloudxr.env
   ./install/plugins/pandora_glove/pandora_glove_plugin --rate=200

See :ref:`dedicated-cloudxr-runtime` and
:ref:`load-cloudxr-environment-variables` for the full service setup.
Applications can also launch the plugin through its ``plugin.yaml`` with
``TeleopSession``:

.. code-block:: python

   PluginConfig(plugin_name="pandora_glove_plugin", plugin_root_id="pandora_glove",
                plugin_args=["--rate=200"])

The plugin owns the receiver while it runs; close the 01 Glove app first. It
reopens the receiver by itself after it is unplugged.

.. list-table:: Arguments
   :widths: 30 15 55
   :header-rows: 1

   * - Argument
     - Default
     - Meaning
   * - ``--rate=HZ`` or ``--rate=max``
     - ``90``
     - Frames pushed per hand per second, 1 to 1000; ``max`` pushes every frame the glove sends
   * - ``--wrist=MODE``
     - ``auto``
     - ``auto``, ``hand_tracking``, ``controller`` (through the glove's tracker
       offset), ``se3:<left>,<right>`` (an SE3 tracker collection per glove, such as
       ``vive_tracker_<serial>``, through the glove's tracker offset); or, for tests
       without a headset, ``fixed`` (both hands at a table in front of the origin)
       and ``imu`` (fixed positions, turned by the glove's wrist IMU)
   * - ``--left-aim-to-wrist``, ``--right-aim-to-wrist``
     - the glove's offset
     - Controller aim pose to wrist as ``px,py,pz,qx,qy,qz,qw`` (metres, then
       quaternion), overriding the glove's stored offset in ``controller`` mode.
       The stored offsets are measured against SteamVR's poses, which can differ
       from OpenXR's aim pose for the same controller
   * - ``--port=PATH``
     - the receiver plugged in
     - The receiver's serial port
   * - ``--profiles-dir=DIR``
     - ``~/.pandora/profiles``
     - Where ``pandora calibrate`` wrote the gloves' profiles
   * - ``--glove-profiles=on|off``
     - ``on``
     - Use the calibration stored on a glove that has no profile file
   * - ``--datasets=hand,haptic,joints``
     - all three
     - What to run
   * - ``--haptic-collection=ID``
     - ``pandora_glove_haptic``
     - The haptic collection to read
   * - ``--haptic-hold-ms=MS``
     - ``60``
     - How long the glove holds each haptic level
   * - ``--stale-ms=MS``
     - ``200``
     - A hand with no new frame for this long goes inactive

Published data
--------------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Data
     - Contents
   * - Hand tracking, both hands
     - 26 OpenXR joints through ``plugin_utils::HandInjector``, read by
       ``HandTracker`` and every hand retargeter. Joints 1 to 25 are the glove's
       skeleton bones; the palm is the centre of the middle metacarpal.
   * - ``pandora_glove_joints_left``, ``pandora_glove_joints_right``
     - ``JointStateOutput`` (tensor ``joint_state``), one sample per glove frame:
       per finger, three flexions from the knuckle outward and the abduction at
       the base, in radians (``thumb_cmc_flexion`` … ``little_mcp_abduction``).
       Read them with ``JointStateSource``.
   * - ``<plugin root>/device_status``
     - ``/hand/left`` and ``/hand/right``: connected, degraded (no current
       frame), disconnected (no receiver, or no calibrated glove, named in the
       error) or failed (firmware libpandora cannot read).

Haptics
-------

The plugin reads ``HapticCommand`` on ``--haptic-collection`` for the endpoints
``left`` and ``right``. Each value is a power from 0 to 1: five values drive
thumb to pinky, and a sixth drives the back of the hand. Isaac Teleop's glove
adapter sends either:

.. code-block:: python

   from isaaccapture.haptic_devices.glove import haptic_glove_device

   device = haptic_glove_device("pandora_glove_haptic")                  # five fingers
   device = haptic_glove_device("pandora_glove_haptic", num_fingers=6)   # and the back of the hand

The motors are linear resonant actuators: amplitude is the only axis, with
20–40 ms spin-up and ring-down. A level stays on while commands keep arriving;
a command older than four holds is treated as silence and the motors stop.

Testing without hardware
------------------------

``examples/pandora_glove`` replays recorded gloves through a stand-in receiver
on a pseudo-terminal, so libpandora, the plugin and Isaac Teleop run as they
would with hardware. It also has a hand checker and a haptic sweep.
``tests/cpp/plugins/pandora_glove_session`` runs the plugin against a capturing
plugin session. It checks push rates, hand shape, wrist modes, haptics and
reconnection without a CloudXR runtime.

Troubleshooting
---------------

.. list-table::
   :widths: 40 60
   :header-rows: 1

   * - Symptom
     - Fix
   * - ``receiver not open (open_failed: Permission denied)``
     - Install the udev rule, then unplug and replug the receiver.
   * - ``receiver not open (no_receiver ...)``
     - No receiver is plugged in, or another program (the 01 Glove app) holds it.
       The plugin retries every second.
   * - A hand never appears
     - The glove is off, unpaired, or uncalibrated; the device status names
       gloves with no calibration.
   * - ``Failed to create OpenXR instance``
     - CloudXR is not running or ``cloudxr.env`` was not sourced. CloudXR needs
       a native NVIDIA Linux driver; it does not start under WSL2.
   * - Hands appear at the origin or the wrong place
     - The wrist source has no pose: connect the headset, or measure and pass the
       controller mount's ``--left-aim-to-wrist`` / ``--right-aim-to-wrist``.
