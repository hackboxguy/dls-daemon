# dls-daemon

High-performance DLS/DLT packet capture daemon for automotive diagnostic logging.

## Overview

`dls-daemon` captures DLT (Diagnostic Log and Trace) packets from automotive ECUs over serial UART connections. It's designed for reliable, zero-loss capture from safety-critical automotive microcontrollers.

### Why dls-daemon?

Python-based parsers often drop packets due to GIL contention and buffering issues. In testing, Python showed ~35% packet variation and string corruption. `dls-daemon` achieves:

- **~10x reduction in packet loss** (3.7% vs ~35%)
- **Zero string corruption**
- **Consistent capture across power cycles**

## Features

- **Real-time serial capture** with SCHED_FIFO priority
- **Lock-free SPSC ring buffer** (1MB default)
- **Unix socket + optional TCP** for data streaming
- **Category-based filtering** (VERSION, BOOT, POWER, etc.)
- **Binary logging** for replay and analysis
- **Message counter gap detection**
- **Built-in FIBEX message definitions**
- **dls-ctl** command-line control utility

## Architecture

```
Serial Port ──► Ring Buffer ──► Parser ──► Dispatcher ──► Clients
    │              (1MB)          │            │
    │                             │            ├── Unix Socket
    │                             │            ├── TCP (optional)
    │                             │            └── Binary Logger
    │                             │
    └── RT Priority 50            └── FIBEX Enrichment
```

## Building

### Requirements

- CMake 3.14+
- C++14 compiler (GCC 5+ or Clang 3.4+)
- Linux (uses epoll, POSIX serial APIs)

### Build Steps

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Build dls-ctl Standalone

```bash
cd client
mkdir build && cd build
cmake ..
make
```

## Usage

### Basic Usage

```bash
# Capture from serial port
sudo ./dls-daemon -p /dev/ttyUSB2

# With verbose output
sudo ./dls-daemon -p /dev/ttyUSB2 -v
```

### With TCP and Logging

```bash
# Enable TCP streaming on port 3490 and log to file
sudo ./dls-daemon -p /dev/ttyUSB2 -T 3490 -l capture.bin
```

### Run as Daemon

```bash
sudo ./dls-daemon -p /dev/ttyUSB2 -d -P /var/run/dls-daemon.pid
```

### Replay a Capture

```bash
./dls-daemon --replay capture.bin --replay-speed 2.0
```

## Command Line Options

```
Serial Port:
  -p, --port PATH           Serial port (default: /dev/ttyUSB2)
  -b, --baud RATE           Baud rate (default: 115200)

Network:
  -s, --data-socket PATH    Unix socket (default: /tmp/dls-data.sock)
  -T, --data-tcp-port PORT  Enable TCP data streaming
  -C, --control-port PORT   Control port (default: 3491)

Configuration:
  -c, --config FILE         FIBEX message config JSON
  -B, --buffer-size KB      Ring buffer size (default: 1024)

Logging:
  -l, --log-file FILE       Binary packet logging
  -v, --verbose             Verbose output

Daemon:
  -d, --daemon              Run as background daemon
  -P, --pid-file FILE       PID file path

Replay:
  -r, --replay FILE         Replay from binary log
  -S, --replay-speed N      Speed factor (0 = fast as possible)

Gap Tracking:
  -G, --gap-threshold N     Min gap to report (0 = disable)
```

## Connecting Clients

### Python Test Client

```bash
# Subscribe to all packets
python3 scripts/test_client.py

# Filter by category
python3 scripts/test_client.py -c VERSION BOOT

# Filter by message IDs
python3 scripts/test_client.py -m 1201 1203 9330
```

### Using dls-ctl

```bash
# Show daemon status
./dls-ctl stats

# Show recent mcnt gaps
./dls-ctl gaps 10

# Enable logging
./dls-ctl log enable /tmp/capture.bin

# List connected clients
./dls-ctl clients
```

### Custom Client (JSON Protocol)

Connect to Unix socket `/tmp/dls-data.sock` or TCP port 3490:

```json
// Subscribe to categories
{"cmd": "subscribe", "categories": ["VERSION", "BOOT"]}

// Subscribe to specific message IDs
{"cmd": "subscribe", "msg_ids": [1201, 1203]}

// Subscribe to all packets
{"cmd": "subscribe"}
```

Packets are streamed as JSON:

```json
{
  "type": "packet",
  "seq": 1234,
  "ts": 1702800000123456,
  "mcnt": 42,
  "msg_id": 1201,
  "msg_id_hex": "0x000004b1",
  "category": "VERSION",
  "name": "APPA_SW_VERSION",
  "payload_hex": "070032353439354100",
  "payload_ascii": "25495A"
}
```

## Built-in Message Categories

| Category | Description | Priority |
|----------|-------------|----------|
| VERSION | SW version strings | High |
| BOOT | Boot sequence events | High |
| POWER | Power mode changes | Medium |
| THERMAL | Temperature events | High |
| VOLTAGE | Battery/voltage status | Medium |
| SFI | Safety heartbeat | Medium |
| SAFETY | Safety test results | High |
| ERROR | Error conditions | High |

## Performance

Tested with RH850 MCU at 115200 baud over 27 power cycles:

| Metric | Result |
|--------|--------|
| Packet rate | ~430 pkt/sec |
| Buffer overflows | 0 |
| Parse errors | 0 |
| Packet loss | 3.7% (1 in 27 cycles) |
| String corruption | 0% |

## Troubleshooting

### Permission Denied on Serial Port

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### High mcnt Gap Count

If packets lack ECU ID/APID/CTID headers, disable gap tracking:

```bash
./dls-daemon -p /dev/ttyUSB2 -G 0
```

### Daemon Won't Shutdown

The daemon handles SIGINT/SIGTERM. If unresponsive:

```bash
kill -9 $(cat /var/run/dls-daemon.pid)
```

## License

MIT License

