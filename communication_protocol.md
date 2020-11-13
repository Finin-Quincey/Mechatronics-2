# Snake <-> Hamster Communication Protocol

Everything is **little-endian!**

## Joystick Control Mode
_Drives the robot in real-time_

This is for testing purposes and not part of the main program, see robot_joystick.py

### Byte 0: Message type
| DEC | BIN        | Message          |
|-----|------------|------------------|
|   0 | 0b00000000 | `STOP` (default) |
|   1 | 0b00000001 | `FORWARDS`       |
|   2 | 0b00000010 | `BACKWARDS`      |
|   3 | 0b00000011 | `LEFT`           |
|   4 | 0b00000100 | `RIGHT`          |

### Byte 1: Speed (as uint8)

## Point-to-Point Control Mode
_Specifies a destination for the robot to drive towards and allows it to monitor its progress_

This is used in both manual and automatic control modes

### Byte 0: Message type
| DEC | BIN        | Message          |
|-----|------------|------------------|
|   0 | 0b00000000 | `STOP` (default) |
|   1 | 0b00000001 | `DESTINATION`    |
|   2 | 0b00000010 | `UPDATE`         |

### Bytes 1-n: Data

`STOP`  
n = 1  
No additional data

`DESTINATION`  
n = 3
| Byte | Data         |
|------|--------------|
|    1 | x coordinate |
|    2 | y coordinate |

`UPDATE`  
n = 5
| Byte | Data          |
|------|---------------|
|    1 | x coordinate  |
|    2 | y coordinate  |
|    3 | bearing       |
|    4 | timestamp MSB |
|    5 | timestamp     |
|    6 | timestamp     |
|    7 | timestamp LSB |