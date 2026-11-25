# SPDX-FileCopyrightText: Copyright (c) 2026 Pandora-On-Zero. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""A stand-in 01 Glove receiver: recorded glove packets over a pseudo-terminal.

The plugin opens the terminal as if it were the receiver's serial port, so
libpandora's serial reading, the glove pipeline and the plugin all run as
they would with hardware. Commands the plugin sends back (haptics) are
decoded and logged.

Recordings are libpandora's golden files (``shared/golden/*.json`` in the
Pandora repository): the glove's profile plus its packets. Each glove's
packets loop, restamped on the glove clock at ``--rate``.
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import os
import select
import struct
import threading
import time
import tty
from pathlib import Path

log = logging.getLogger("isaaccapture.examples.pandora_glove.fake_receiver")

SENTINEL = 0x00
DELIMITER = 0x0A
TRACKING = 0x01
HAPTIC_ROM = 0x10
HAPTIC_RTP = 0x11
STORE = 0x26
TIMESTAMP_AT = 17  # u32 little-endian milliseconds after the type byte and 16-byte id
MOTORS = ("thumb", "index", "middle", "ring", "pinky", "back")


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_at, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_at] = code
            code_at, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_at] = code
                code_at, code = len(out), 1
                out.append(0)
    out[code_at] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        if code == 0:
            return None
        i += 1
        end = i + code - 1
        if end > n:
            return None
        block = data[i:end]
        if 0 in block:
            return None
        out += block
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


_SWAP = bytes.maketrans(b"\x00\x0a", b"\x0a\x00")


def encode_frame(payload: bytes) -> bytes:
    """``[0x00] swap0A(COBS(payload || CRC16-BE)) [0x0A]``, as the receiver frames it."""
    body = cobs_encode(payload + struct.pack(">H", crc16_ccitt(payload)))
    return bytes([SENTINEL]) + body.translate(_SWAP) + bytes([DELIMITER])


def decode_frame(line: bytes) -> bytes | None:
    if line[:1] == bytes([SENTINEL]):
        line = line[1:]
    decoded = cobs_decode(line.translate(_SWAP))
    if decoded is None or len(decoded) < 3:
        return None
    payload, crc = decoded[:-2], struct.unpack(">H", decoded[-2:])[0]
    return payload if crc16_ccitt(payload) == crc else None


def device_id(packet: bytes) -> str:
    return packet[1:17].split(b"\x00", 1)[0].decode("utf-8", "replace")


class Glove:
    """One recorded glove: its packets, looped and restamped."""

    def __init__(self, recording: Path):
        data = json.loads(recording.read_text(encoding="utf-8"))
        self.packets = [base64.b64decode(p) for p in data["packets"]]
        self.packets = [p for p in self.packets if p and p[0] == TRACKING]
        if not self.packets:
            raise ValueError(f"{recording}: no tracking packets")
        self.id = device_id(self.packets[0])
        self.hand = data["hand"]
        self.profile = {"hand": data["hand"], **data["device"]}
        self.next = 0
        self.clock_ms = 1000

    def next_packet(self, period_ms: float) -> bytes:
        packet = bytearray(self.packets[self.next])
        self.next = (self.next + 1) % len(self.packets)
        self.clock_ms += period_ms
        struct.pack_into("<I", packet, TIMESTAMP_AT, int(self.clock_ms) & 0xFFFFFFFF)
        return bytes(packet)


class FakeReceiver:
    def __init__(
        self,
        gloves: list[Glove],
        rate_hz: float,
        command_log: Path | None,
        silences: list[tuple[int, float, float]] | None = None,
    ):
        self.gloves = gloves
        self.silences = silences or []
        self.period = 1.0 / rate_hz
        self.command_log = (
            command_log.open("a", encoding="utf-8") if command_log else None
        )
        self.master, self.slave = os.openpty()
        # Raw at once: with the default line discipline the terminal would echo
        # the packets written here back as if the plugin had sent them.
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        self.stop = threading.Event()
        self.sent = 0

    def write_loop(self) -> None:
        start = next_at = time.monotonic()
        while not self.stop.is_set():
            t = time.monotonic() - start
            frames = b"".join(
                encode_frame(g.next_packet(self.period * 1000.0))
                for i, g in enumerate(self.gloves)
                if not any(
                    i == which and begin <= t < end
                    for which, begin, end in self.silences
                )
            )
            if frames:
                os.write(self.master, frames)
            self.sent += 1
            next_at += self.period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()

    def read_loop(self) -> None:
        pending = b""
        while not self.stop.is_set():
            ready, _, _ = select.select([self.master], [], [], 0.2)
            if not ready:
                continue
            try:
                pending += os.read(self.master, 4096)
            except OSError:
                continue
            *lines, pending = pending.split(bytes([DELIMITER]))
            for line in lines:
                if line[:1] == bytes([SENTINEL]):
                    payload = decode_frame(line)
                    if payload is None:
                        log.warning("damaged command frame %s", line.hex())
                    else:
                        self.on_command(payload)
                elif line.strip():
                    log.info("text from the plugin: %r", line)

    def on_command(self, payload: bytes) -> None:
        kind, target = payload[0], device_id(payload)
        record: dict = {"t": round(time.time(), 4), "device": target}
        if kind == HAPTIC_RTP and len(payload) >= 17 + 6 + 12:
            amplitudes = list(payload[17:23])
            durations = list(struct.unpack_from("<6H", payload, 23))
            record |= {
                "type": "rtp",
                "amplitudes": amplitudes,
                "durations_ms": durations,
            }
            shown = " ".join(f"{m}={a}" for m, a in zip(MOTORS, amplitudes) if a)
            log.info(
                "haptics %s: %s (hold %d ms)", target, shown or "stop", max(durations)
            )
        elif kind == HAPTIC_ROM and len(payload) >= 23:
            record |= {"type": "rom", "effects": list(payload[17:23])}
            log.info("haptic effects %s: %s", target, list(payload[17:23]))
        elif kind == STORE:
            record |= {"type": "store"}
            log.debug("glove store request for %s (not answered)", target)
        else:
            record |= {"type": f"0x{kind:02x}", "bytes": payload.hex()}
            log.info("command 0x%02x for %s", kind, target)
        if self.command_log:
            self.command_log.write(json.dumps(record) + "\n")
            self.command_log.flush()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "recordings", nargs="+", type=Path, help="golden recording files, one per glove"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=200.0,
        help="packets per second per glove (default 200)",
    )
    parser.add_argument(
        "--link",
        type=Path,
        default=Path("/tmp/pandora-fake-rx"),
        help="symlink to the terminal",
    )
    parser.add_argument(
        "--profiles-dir",
        type=Path,
        default=Path("/tmp/pandora-fake-profiles"),
        help="where each glove's profile is written for the plugin's --profiles-dir",
    )
    parser.add_argument(
        "--command-log", type=Path, help="append each command received as a JSON line"
    )
    parser.add_argument(
        "--silence",
        action="append",
        default=[],
        metavar="GLOVE:FROM:TO",
        help="send nothing from glove GLOVE (0-based, in the order given) from FROM to TO seconds",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    gloves = [Glove(path) for path in args.recordings]
    args.profiles_dir.mkdir(parents=True, exist_ok=True)
    for glove in gloves:
        (args.profiles_dir / f"{glove.id}.json").write_text(
            json.dumps(glove.profile, indent=1) + "\n"
        )

    silences = []
    for spec in args.silence:
        which, begin, end = spec.split(":")
        silences.append((int(which), float(begin), float(end)))
    receiver = FakeReceiver(gloves, args.rate, args.command_log, silences)
    if args.link.is_symlink() or args.link.exists():
        args.link.unlink()
    args.link.symlink_to(receiver.port)
    threading.Thread(target=receiver.read_loop, daemon=True).start()
    writer = threading.Thread(target=receiver.write_loop, daemon=True)
    writer.start()
    names = ", ".join(f"{g.id} ({g.hand}, {len(g.packets)} packets)" for g in gloves)
    log.info(
        "fake receiver on %s (link %s) at %.0f Hz per glove: %s",
        receiver.port,
        args.link,
        args.rate,
        names,
    )
    log.info("profiles in %s", args.profiles_dir)
    try:
        last, last_sent = time.monotonic(), 0
        while writer.is_alive():
            time.sleep(5.0)
            now = time.monotonic()
            log.info(
                "sent %.0f packets/s per glove",
                (receiver.sent - last_sent) / (now - last),
            )
            last, last_sent = now, receiver.sent
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop.set()
        args.link.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
