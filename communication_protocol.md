# Snake <-> Hamster Communication Protocol

Everything is **little-endian!**

Coordinates are specified in **cm** in the range 0-255 (this covers the entire arena length of 2.4m)

Bearings (absolute) are specified in the range 0-255 (256 divisions in a full circle), where **0 is north** (positive y axis) and clockwise is positive.

Correction angles (relative) are specified in the range 0-255 (256 divisions in a full circle), where **127 means 0 degrees** (no correction) and clockwise is positive. This allows corrections of between -180 and 180 degrees to be sent.

## Joystick Control Mode
_Drives the robot in real-time_

This is for testing purposes and not part of the main program, see robot_joystick.py

**UDP Port: 50003**

| Byte | Data         |
|------|--------------|
|    0 | message type |
|    1 | speed        |

### Byte 0: Message type
| DEC | BIN        | Message          |
|-----|------------|------------------|
|   0 | 0b00000000 | `STOP` (default) |
|   1 | 0b00000001 | `FORWARDS`       |
|   2 | 0b00000010 | `BACKWARDS`      |
|   3 | 0b00000011 | `LEFT`           |
|   4 | 0b00000100 | `RIGHT`          |

## Point-to-Point Control Mode
_Specifies a destination for the robot to drive towards and allows it to monitor its progress_

This is used in both manual and automatic control modes

### Destination Message

**UDP Port: 50001**

| Byte | Data          |
|------|---------------|
|    0 | current x     |
|    1 | current y     |
|    2 | bearing       |
|    3 | destination x |
|    4 | destination y |

Reply from robot is a single byte value of 1 (confirm) or 0 (error)

### Update Message

**UDP Port: 50002**

| Byte | Data             |
|------|------------------|
|    0 | angle correction |