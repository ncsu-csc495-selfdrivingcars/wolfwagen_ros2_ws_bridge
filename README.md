# ROS2 WebSocket Bridge — WolfWagen Monitor

A lightweight tool to stream ROS2 ZED camera data (pose + RGB image) to any
browser over WebSocket. No ROS needed on the viewing machine.

```
┌─────────────────────┐         WebSocket          ┌─────────────────────┐
│    Robot (ROS2)      │ ────────────────────────── │   Laptop (Browser)  │
│                      │   pose: JSON  ~15 Hz       │                     │
│  ZED Node            │   image: binary JPEG ~10Hz │   Web UI Dashboard  │
│    ↓                 │                            │   - Camera feed     │
│  ros2_ws_bridge.py   │ ◄──── ping/config ──────── │   - Pose display    │
│    (ws://0.0.0.0:    │                            │   - 3D trajectory   │
│         8765)        │                            │                     │
└─────────────────────┘                            └─────────────────────┘
```

## Files

```
ros2_ws_bridge/
├── ros2_ws_bridge.py      # ROS2 node + WebSocket server (runs on robot)
├── web_ui/
│   └── index.html         # Browser dashboard (runs on laptop)
├── requirements.txt       # Python dependencies
└── README.md              # This file
```

## Setup

### WolfWagen side

```bash
# 1. Install Python deps
pip install -r requirements.txt

# 2. Run the bridge (Make sure that the ZED ROS wrapper is running)
python3 ros2_ws_bridge.py
```

The bridge will:
- Subscribe to `/zed/zed_node/pose` and `/zed/zed_node/rgb/color/rect/image`
- Start a WebSocket server on port **8765**
- Start an HTTP server on port **8080** serving the web UI

### Laptop side

**Option A — Use the built-in HTTP server (easiest)**

Open a browser and go to:
```
http://ROBOT_IP:8080
```

**Option B — Open the HTML file directly**

Copy `web_ui/index.html` to your laptop and open it in any modern browser.
Then type the robot's IP and port (e.g. `192.168.1.100:8765`) in the
connection bar and click Connect.

## Configuration

All settings are controlled via environment variables:

| Variable        | Default                                         | Description                       |
|-----------------|------------------------------------------------|-----------------------------------|
| `WS_HOST`       | `0.0.0.0`                                      | WebSocket bind address            |
| `WS_PORT`       | `8765`                                          | WebSocket port                    |
| `IMAGE_WIDTH`   | `640`                                           | Downscaled image width            |
| `IMAGE_HEIGHT`  | `360`                                           | Downscaled image height           |
| `IMAGE_QUALITY` | `70`                                            | JPEG quality (1-100)              |
| `IMAGE_FPS`     | `10`                                            | Image broadcast rate (Hz)         |
| `POSE_FPS`      | `15`                                            | Pose broadcast rate (Hz)          |
| `POSE_TOPIC`    | `/zed/zed_node/pose`                            | ROS2 pose topic                   |
| `IMAGE_TOPIC`   | `/zed/zed_node/rgb/color/rect/image`            | ROS2 image topic                  |
| `SERVE_UI`      | `true`                                          | Serve web UI via HTTP             |
| `UI_PORT`       | `8080`                                          | HTTP port for web UI              |

Example with custom settings:
```bash
IMAGE_FPS=5 IMAGE_QUALITY=50 WS_PORT=9090 python3 ros2_ws_bridge.py
```

## Web UI Features

- **Camera Feed** — live JPEG stream rendered on Canvas
- **Pose Display** — position (x, y, z) and orientation as Roll, Pitch, Yaw (degrees, converted from quaternion using ZYX intrinsic convention matching ROS tf2 / REP-103)
- **3D Trajectory** — real-time path visualization with 3 view modes:
  - Top-down (XY plane)
  - Side view (XZ plane)
  - Rotating 3D perspective
- **Connection status** — auto-reconnect on disconnect
- **FPS counters** — separate pose and image rates

## Protocol

### Robot → Browser

**Pose** (JSON text frame):
```json
{
  "type": "pose",
  "ts": 1711234567.123,
  "frame": "map",
  "position": {"x": 1.23, "y": -0.45, "z": 0.02},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.1, "w": 0.99}
}
```
Note: Orientation is transmitted as a quaternion over the wire. The Web UI
converts it to Euler angles (Roll, Pitch, Yaw in degrees) client-side using
the ZYX intrinsic convention (same as ROS tf2).

**Image** (binary frame):
Raw JPEG bytes. No JSON wrapping, no base64 — minimal overhead.

### Browser → Robot

```json
{"type": "ping"}              // → responds with {"type": "pong", "ts": ...}
{"type": "get_stats"}         // → responds with message counts
```

## Bandwidth Estimates

| Setting                | Image bandwidth | Pose bandwidth |
|------------------------|----------------|----------------|
| 640×360 Q70 @ 10 Hz   | ~400 KB/s      | ~6 KB/s        |
| 640×360 Q50 @ 5 Hz    | ~125 KB/s      | ~6 KB/s        |
| 320×180 Q50 @ 5 Hz    | ~40 KB/s       | ~6 KB/s        |

## Extending

**Add joystick control (laptop → robot):**
The WebSocket is bidirectional. Send a JSON message from the browser:
```js
ws.send(JSON.stringify({type: 'cmd_vel', linear: 0.5, angular: 0.1}));
```
Handle it in `_handle_client_msg()` in the bridge node.

**Add more topics:**
Follow the same pattern — add a subscriber + a broadcast coroutine.

## Troubleshooting

1. **No image / no pose?** — Check that ZED node is running and publishing:
   ```bash
   ros2 topic list | grep zed
   ros2 topic hz /zed/zed_node/pose
   ```

2. **Can't connect from laptop?** — Check firewall allows ports 8765 and 8080:
   ```bash
   sudo ufw allow 8765
   sudo ufw allow 8080
   ```

3. **High latency?** — Reduce image quality/resolution/fps via env vars.

4. **WiFi too slow?** — Try `IMAGE_WIDTH=320 IMAGE_HEIGHT=180 IMAGE_QUALITY=40 IMAGE_FPS=3`

