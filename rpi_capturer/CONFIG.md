# Configuration Guide

## Overview

All configuration is done via `config.json` file. Copy `config.example.json` to `config.json` and edit it.

```bash
cp config.example.json config.json
nano config.json
```

## Configuration Sections

### 1. Camera Settings

```json
"camera": {
  "width": 1920,      // Camera resolution width (pixels)
  "height": 1080,     // Camera resolution height (pixels)
  "fps": 30           // Frame rate (frames per second)
}
```

**Typical values:**
- 1920x1080 @ 30fps (Full HD)
- 1280x720 @ 60fps (HD, high frame rate)
- 3840x2160 @ 30fps (4K, if supported)

---

### 2. Autofocus Settings

```json
"autofocus": {
  "enabled": true,    // Enable/disable autofocus
  "mode": 2,          // Autofocus mode
  "speed": 1,         // Autofocus speed
  "range": 2          // Autofocus range
}
```

**Mode values:**
- `0` = Auto (camera decides when to focus)
- `1` = Manual (fixed focus position)
- `2` = Continuous (constantly adjusting)

**Speed values:**
- `0` = Normal (slower, more accurate)
- `1` = Fast (faster, may be less accurate)

**Range values:**
- `0` = Normal (general purpose)
- `1` = Macro (close-up objects)
- `2` = Full (entire range)

**Note:** Only works with Camera Module 3 (or modules with autofocus support)

---

### 3. Motion Detection

```json
"motion_detection": {
  "enabled": true,      // Enable/disable motion detection
  "frame_skip": 7,      // Process every N-th frame
  "tail_duration": 0    // Continue recording after motion stops (seconds)
}
```

**frame_skip:**
- Higher values = less CPU usage, but slower motion detection
- `1` = process every frame (high CPU)
- `7` = process every 7th frame (recommended)
- `15` = process every 15th frame (low CPU)

**tail_duration:**
- How many seconds to continue recording after motion stops
- `0` = stop immediately
- `5` = continue for 5 more seconds

---

### 4. MJPEG / JPEG Settings

```json
"mjpeg": {
  "enabled": true,              // Enable MJPEG encoding
  "output_enabled": true,       // Enable MJPEG output
  "encode_interval_ms": 300,    // Minimum time between encodes (milliseconds)
  "burst_photo_count": 5        // Photos to collect before sending
}
```

**encode_interval_ms:**
- Throttle JPEG encoding to reduce CPU usage
- `300` = encode at most every 300ms (~3 fps)
- Lower = more photos, higher CPU usage

**burst_photo_count:**
- Number of photos to collect before sending to Telegram as media group
- `5` = collect 5 photos, then send as album
- `1` = send each photo individually (not recommended)

---

### 5. H.264 Hardware Encoding

```json
"h264": {
  "enabled": true,          // Enable H.264 encoding
  "output_enabled": true,   // Enable H.264 output
  "gop_size": 60,           // Keyframe interval (GOP size)
  "bitrate": 5000000,       // Target bitrate (bits per second)
  "cbr": false,             // Constant bitrate mode
  "sps_pps_repeat": true    // Repeat SPS/PPS headers
}
```

**gop_size (Group of Pictures):**
- Number of frames between keyframes (I-frames)
- `30` = 1 keyframe per second @ 30fps (good for seeking)
- `60` = 1 keyframe every 2 seconds (better compression)
- Lower = larger file, easier to seek; Higher = smaller file, harder to seek

**bitrate:**
- Target bitrate in bits per second
- `5000000` = 5 Mbps (high quality)
- `2000000` = 2 Mbps (medium quality)
- `10000000` = 10 Mbps (very high quality)

**cbr (Constant Bitrate):**
- `false` = VBR (Variable Bitrate) - better quality, variable size
- `true` = CBR (Constant Bitrate) - consistent size, may lose quality in complex scenes

**sps_pps_repeat:**
- Repeat SPS/PPS headers with each keyframe
- `true` = better for streaming (clients can join mid-stream)
- `false` = smaller file size

---

### 6. Telegram Bot

```json
"telegram": {
  "enabled": true,              // Enable Telegram integration
  "bot_token": "123:ABC...",    // Bot token from @BotFather
  "chat_id": "123456789",       // Your chat ID
  "max_queue_size": 5           // Maximum queued messages
}
```

**How to get bot_token:**
1. Talk to [@BotFather](https://t.me/BotFather) on Telegram
2. Send `/newbot` and follow instructions
3. Copy the token (format: `123456789:ABCdefGHIjklMNOpqrsTUVwxyz`)

**How to get chat_id:**
1. Talk to [@userinfobot](https://t.me/userinfobot) on Telegram
2. It will reply with your user ID (format: `123456789`)

**max_queue_size:**
- Maximum number of pending messages in queue
- If exceeded, oldest messages are dropped
- `5` = keep up to 5 pending photo bursts

---

### 7. TCP Streaming

```json
"tcp": {
  "format": "h264",           // Stream format: "h264" or "mjpeg"
  "broadcast": {
    "enabled": true,          // Enable TCP server (broadcast mode)
    "port": 8554,             // TCP port to listen on
    "max_clients": 5          // Maximum simultaneous clients
  },
  "client": {
    "enabled": false,         // Enable TCP client mode
    "remote_ip": "10.0.0.2",  // Remote server IP address
    "remote_port": 9999,      // Remote server port
    "reconnect_interval_sec": 1  // Reconnection interval (seconds)
  }
}
```

**format:**
- `"h264"` = stream H.264 video (efficient, requires decoder)
- `"mjpeg"` = stream MJPEG frames (compatible, higher bandwidth)

**broadcast mode (server):**
- Acts as TCP server, multiple clients can connect
- Useful for local network streaming
- Example: `ffplay tcp://10.0.0.1:8554` (replace with Pi's IP)

**client mode:**
- Connects to remote TCP server
- Automatically reconnects if connection lost
- Useful for sending stream to remote server

**reconnect_interval_sec:**
- How often to retry connection if disconnected
- `1` = retry every second (aggressive)
- `5` = retry every 5 seconds (conservative)

---

## Example Use Cases

### 1. Motion-Activated Telegram Camera

```json
{
  "camera": {"width": 1920, "height": 1080, "fps": 30},
  "motion_detection": {"enabled": true, "frame_skip": 7},
  "mjpeg": {"enabled": true, "burst_photo_count": 5},
  "telegram": {"enabled": true, "bot_token": "...", "chat_id": "..."},
  "tcp": {"broadcast": {"enabled": false}},
  "h264": {"enabled": false}
}
```

### 2. High-Quality H.264 Streaming Server

```json
{
  "camera": {"width": 1920, "height": 1080, "fps": 30},
  "h264": {"enabled": true, "bitrate": 10000000, "gop_size": 30},
  "tcp": {"format": "h264", "broadcast": {"enabled": true, "port": 8554}},
  "motion_detection": {"enabled": false},
  "telegram": {"enabled": false}
}
```

### 3. Low-Bandwidth MJPEG Stream

```json
{
  "camera": {"width": 1280, "height": 720, "fps": 15},
  "mjpeg": {"enabled": true, "encode_interval_ms": 500},
  "tcp": {"format": "mjpeg", "broadcast": {"enabled": true, "port": 8554}},
  "h264": {"enabled": false},
  "telegram": {"enabled": false}
}
```

---

## Testing Your Configuration

After editing `config.json`, run the application:

```bash
cd build2
./rpi_capturer ../config.json
```

Or simply (uses `config.json` by default):

```bash
./rpi_capturer
```

The application will print your configuration on startup and show which features are enabled.
