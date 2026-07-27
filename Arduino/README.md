# Arduino Nano 33 IoT PPM Bridge

This optional interface converts eight numbered RC channel pulse widths sent
over USB into PPM intended for a compatible transmitter trainer input. It is
independent of Vicon and is not required for rigid-body tracking.

## Interface

| Property | Value |
|---|---|
| Board | [Arduino Nano 33 IoT](https://docs.arduino.cc/hardware/nano-33-iot) |
| Command input | USB virtual COM port |
| Host port setting | 1,000,000 baud |
| PPM output | D3 |
| Channels | 8, in the supplied order |
| Channel range | 1000-2000 microseconds |
| Default startup/timeout values | 1500 microseconds on every channel |
| PPM frame | 20 milliseconds |
| PPM mark | 300 microseconds |
| Command timeout | 250 milliseconds, then channels use their configured fallback values |

Commands take effect together at the start of a PPM frame. The firmware starts
at the configured fallback values and returns to them when valid commands stop
arriving.

## Files

- `Nano33IoTPpmBridge/Nano33IoTPpmBridge.ino`: serial-to-PPM firmware
- `Python/send_ppm_channels.py`: Python packet encoder and sender
- `MATLAB/encodePPMPacket.m`: MATLAB packet encoder
- `MATLAB/sendPPMChannels.m`: MATLAB sender

Both senders accept numbered channels directly. Aircraft surface names,
mixing, signs, trims, and receiver assignments belong in the calling
application.

## Packet

Each command is 17 bytes:

| Bytes | Value |
|---|---|
| 0 | `0x50` (`P`) |
| 1-2 | Channel 1, unsigned 16-bit little-endian microseconds |
| 3-4 | Channel 2 |
| 5-6 | Channel 3 |
| 7-8 | Channel 4 |
| 9-10 | Channel 5 |
| 11-12 | Channel 6 |
| 13-14 | Channel 7 |
| 15-16 | Channel 8 |

The all-1500-microsecond packet is:

```text
50 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05 dc 05
```

Only complete packets containing pulse widths in the stated range update the
output. Send commands at 50 Hz; at least one valid command must arrive within
every 250 milliseconds to prevent the timeout from applying the configured
fallback values.

The default 1500-microsecond values are not universally neutral or safe. Before
uploading, set each entry of `kFallbackPulseUs` in the sketch to an appropriate
1000-2000-microsecond fallback and verify its meaning for every assigned
transmitter and receiver channel.

## Upload

1. Install **Arduino SAMD Boards (32-bits ARM Cortex-M0+)** using
   [Boards Manager](https://support.arduino.cc/hc/en-us/articles/360016119519-Add-boards-to-Arduino-IDE).
2. Open `Nano33IoTPpmBridge/Nano33IoTPpmBridge.ino`.
3. Select **Arduino Nano 33 IoT** and its serial port.
4. Upload the sketch.

The sketch uses the board core only and requires no additional Arduino
libraries.

## Python

From `Arduino/Python`, install the serial dependency:

```powershell
python -m pip install -r requirements.txt
```

Open the serial connection once and reuse it:

```python
import time

from serial import Serial

from send_ppm_channels import send_ppm_channels


channel_pulses_us = (1500,) * 8

with Serial("COM3", 1_000_000) as connection:
    time.sleep(2.0)
    while True:
        send_ppm_channels(connection, channel_pulses_us)
        time.sleep(0.02)
```

Replace `COM3` with the port shown for the Nano 33 IoT.

Press `Ctrl+C` to stop. The firmware applies the configured fallback values
after the command timeout.

## MATLAB

From the repository root:

```matlab
addpath("Arduino/MATLAB")
serialConnection = serialport("COM3", 1e6);
pause(2)
channelPulsesMicroseconds = 1500*ones(1, 8);

while true
    sendPPMChannels(serialConnection, channelPulsesMicroseconds);
    pause(0.02)
end
```

Replace `COM3` with the port shown for the Nano 33 IoT.

Press `Ctrl+C` to stop, then run `clear serialConnection` to release the port.
The firmware applies the configured fallback values after the command timeout.

## Hardware Connection

The sketch drives D3 as a 3.3 V push-pull output with an idle-low PPM waveform
and positive marks. It does not define a transmitter connector or electrical
adapter.

Before connecting D3, verify the transmitter trainer port's signal pin,
reference connection, accepted voltage, and required polarity from the
transmitter documentation. Add level conversion or isolation when required.
Do not treat trainer-port connections as interchangeable between transmitter
models. The official
[Nano 33 IoT pinout](https://docs.arduino.cc/resources/pinouts/ABX00027-full-pinout.pdf)
identifies D3, ground, and the board's electrical limits.

## Bench Check

Before connecting a transmitter or aircraft:

1. Measure D3 relative to Arduino ground.
2. Confirm an idle-low waveform with 300-microsecond positive marks and a
   20-millisecond frame.
3. Send known channel values and confirm that only the requested numbered
   slots change between 1000 and 2000 microseconds.
4. Stop sending and confirm that every slot uses its configured fallback after
   the command timeout and the next PPM frame.

## Use with Vicon

Vicon tracking and command output are separate:

```text
Vicon Tracker -> Python or MATLAB client -> rigid-body states
Control application -> USB serial -> Nano 33 IoT -> compatible trainer PPM input
```

The control application decides whether and how a Vicon state produces a
channel command. This bridge does not provide an aircraft controller, arming
system, or complete flight-safety system.
