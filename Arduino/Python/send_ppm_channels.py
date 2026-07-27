"""Encode and send channel commands to the Nano 33 IoT PPM bridge."""

import operator
import struct
from collections.abc import Sequence
from typing import Protocol


CHANNEL_COUNT = 8
MINIMUM_PULSE_US = 1000
MAXIMUM_PULSE_US = 2000
_PACKET = struct.Struct("<B8H")
_PACKET_HEADER = ord("P")


class _SerialConnection(Protocol):
    def write(self, data: bytes) -> int:
        ...


def encode_ppm_packet(channel_pulses_us: Sequence[int]) -> bytes:
    """Encode eight receiver-channel pulse widths into one serial packet."""

    pulses = tuple(operator.index(value) for value in channel_pulses_us)
    if len(pulses) != CHANNEL_COUNT:
        raise ValueError(f"Expected {CHANNEL_COUNT} channel pulse widths.")
    if any(
        pulse < MINIMUM_PULSE_US or pulse > MAXIMUM_PULSE_US
        for pulse in pulses
    ):
        raise ValueError(
            f"Channel pulse widths must be {MINIMUM_PULSE_US}-"
            f"{MAXIMUM_PULSE_US} microseconds."
        )
    return _PACKET.pack(_PACKET_HEADER, *pulses)


def send_ppm_channels(
    connection: _SerialConnection,
    channel_pulses_us: Sequence[int],
) -> None:
    """Write one complete channel command to an open serial connection."""

    connection.write(encode_ppm_packet(channel_pulses_us))
