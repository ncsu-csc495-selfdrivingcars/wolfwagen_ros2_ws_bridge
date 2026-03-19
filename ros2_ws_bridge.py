#!/usr/bin/env python3
"""
ROS2 WebSocket Bridge Node
===========================
Subscribes to ZED camera topics (pose + image) and broadcasts them
over WebSocket to any connected client (browser, script, etc.)

Usage:
  1. Run:  python3 ros2_ws_bridge.py
  2. Open the web UI on your laptop at  http://ROBOT_IP:8765

Dependencies:
  pip install websockets opencv-python-headless numpy
  (rclpy, cv_bridge, sensor_msgs, geometry_msgs come from ROS2)

Configuration via environment variables:
  WS_PORT          WebSocket port (default: 8765)
  WS_HOST          Bind address (default: 0.0.0.0)
  IMAGE_WIDTH      Downscale width (default: 640)
  IMAGE_HEIGHT     Downscale height (default: 360)
  IMAGE_QUALITY    JPEG quality 1-100 (default: 70)
  IMAGE_FPS        Target image broadcast rate (default: 10)
  POSE_FPS         Target pose broadcast rate (default: 15)
  POSE_TOPIC       Pose topic (default: /zed/zed_node/pose)
  IMAGE_TOPIC      Image topic (default: /zed/zed_node/rgb/color/rect/image)
  SERVE_UI         Serve built-in web UI on HTTP (default: true)
  UI_PORT          HTTP port for web UI (default: 8080)
"""

import os
import sys
import json
import time
import signal
import base64
import asyncio
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

try:
    import websockets
    from websockets.server import serve as ws_serve
except ImportError:
    print("ERROR: 'websockets' package not found. Install it with:")
    print("  pip install websockets")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
WS_HOST = os.environ.get("WS_HOST", "0.0.0.0")
WS_PORT = int(os.environ.get("WS_PORT", "8765"))
IMAGE_WIDTH = int(os.environ.get("IMAGE_WIDTH", "640"))
IMAGE_HEIGHT = int(os.environ.get("IMAGE_HEIGHT", "360"))
IMAGE_QUALITY = int(os.environ.get("IMAGE_QUALITY", "70"))
IMAGE_FPS = float(os.environ.get("IMAGE_FPS", "10"))
POSE_FPS = float(os.environ.get("POSE_FPS", "15"))
POSE_TOPIC = os.environ.get("POSE_TOPIC", "/zed/zed_node/pose")
IMAGE_TOPIC = os.environ.get(
    "IMAGE_TOPIC", "/zed/zed_node/rgb/color/rect/image"
)
SERVE_UI = os.environ.get("SERVE_UI", "true").lower() in ("true", "1", "yes")
UI_PORT = int(os.environ.get("UI_PORT", "8080"))


# ---------------------------------------------------------------------------
# ROS2 Subscriber Node
# ---------------------------------------------------------------------------
class ZedBridgeNode(Node):
    def __init__(self):
        super().__init__("ws_bridge")

        self.bridge = CvBridge()
        self.latest_pose = None
        self.latest_image_bytes = None  # JPEG bytes
        self.pose_count = 0
        self.image_count = 0
        self._lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(PoseStamped, POSE_TOPIC, self._pose_cb, qos)
        self.create_subscription(Image, IMAGE_TOPIC, self._image_cb, qos)

        self.get_logger().info(f"Subscribed to pose:  {POSE_TOPIC}")
        self.get_logger().info(f"Subscribed to image: {IMAGE_TOPIC}")

    # -- Callbacks ----------------------------------------------------------
    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose_dict = {
            "type": "pose",
            "ts": stamp,
            "frame": msg.header.frame_id,
            "position": {"x": p.x, "y": p.y, "z": p.z},
            "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w},
        }
        with self._lock:
            self.latest_pose = pose_dict
            self.pose_count += 1

    def _image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        cv_img = cv2.resize(
            cv_img, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA
        )
        ok, jpg = cv2.imencode(
            ".jpg", cv_img, [cv2.IMWRITE_JPEG_QUALITY, IMAGE_QUALITY]
        )
        if ok:
            with self._lock:
                self.latest_image_bytes = jpg.tobytes()
                self.image_count += 1

    # -- Thread-safe getters ------------------------------------------------
    def get_pose(self):
        with self._lock:
            return self.latest_pose

    def get_image_bytes(self):
        with self._lock:
            return self.latest_image_bytes

    def get_stats(self):
        with self._lock:
            return {
                "pose_msgs_received": self.pose_count,
                "image_msgs_received": self.image_count,
            }


# ---------------------------------------------------------------------------
# WebSocket Server
# ---------------------------------------------------------------------------
class WsBroadcaster:
    def __init__(self, ros_node: ZedBridgeNode):
        self.ros_node = ros_node
        self.clients: set = set()
        self._running = True

    async def handler(self, websocket):
        """Handle a new WebSocket connection."""
        self.clients.add(websocket)
        remote = websocket.remote_address
        print(f"[WS] Client connected: {remote}  (total: {len(self.clients)})")
        try:
            async for raw in websocket:
                # Handle incoming messages from client
                try:
                    msg = json.loads(raw)
                    await self._handle_client_msg(websocket, msg)
                except json.JSONDecodeError:
                    pass
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            print(f"[WS] Client disconnected: {remote}  (total: {len(self.clients)})")

    async def _handle_client_msg(self, ws, msg):
        """Process messages from a client (e.g. config changes)."""
        if msg.get("type") == "ping":
            await ws.send(json.dumps({"type": "pong", "ts": time.time()}))
        elif msg.get("type") == "get_stats":
            stats = self.ros_node.get_stats()
            stats["type"] = "stats"
            stats["clients"] = len(self.clients)
            await ws.send(json.dumps(stats))

    async def broadcast_pose(self):
        """Broadcast pose data at POSE_FPS."""
        interval = 1.0 / POSE_FPS
        last_pose = None
        while self._running:
            pose = self.ros_node.get_pose()
            if pose is not None and pose is not last_pose and self.clients:
                last_pose = pose
                data = json.dumps(pose)
                await self._send_all_text(data)
            await asyncio.sleep(interval)

    async def broadcast_image(self):
        """Broadcast image data at IMAGE_FPS using binary frames."""
        interval = 1.0 / IMAGE_FPS
        last_bytes = None
        while self._running:
            img_bytes = self.ros_node.get_image_bytes()
            if img_bytes is not None and img_bytes is not last_bytes and self.clients:
                last_bytes = img_bytes
                # Send as binary WebSocket frame (no base64 overhead)
                await self._send_all_binary(img_bytes)
            await asyncio.sleep(interval)

    async def _send_all_text(self, data: str):
        if not self.clients:
            return
        stale = set()
        for ws in list(self.clients):
            try:
                await ws.send(data)
            except websockets.ConnectionClosed:
                stale.add(ws)
        self.clients -= stale

    async def _send_all_binary(self, data: bytes):
        if not self.clients:
            return
        stale = set()
        for ws in list(self.clients):
            try:
                await ws.send(data)
            except websockets.ConnectionClosed:
                stale.add(ws)
        self.clients -= stale

    def stop(self):
        self._running = False


# ---------------------------------------------------------------------------
# Optional: serve the web UI via a tiny HTTP server (background thread)
# ---------------------------------------------------------------------------
def start_ui_server(port: int, directory: str):
    """Start a simple HTTP server to serve the web UI files."""

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=directory, **kwargs)

        def log_message(self, format, *args):
            pass  # silence logs

    server = HTTPServer(("0.0.0.0", port), Handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    return server


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
async def async_main(ros_node: ZedBridgeNode):
    broadcaster = WsBroadcaster(ros_node)

    async with ws_serve(broadcaster.handler, WS_HOST, WS_PORT):
        print(f"[WS] WebSocket server listening on ws://{WS_HOST}:{WS_PORT}")
        await asyncio.gather(
            broadcaster.broadcast_pose(),
            broadcaster.broadcast_image(),
        )


def main():
    rclpy.init()
    ros_node = ZedBridgeNode()

    # Spin ROS2 in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    # Optionally serve web UI
    ui_dir = Path(__file__).parent / "web_ui"
    if SERVE_UI and ui_dir.exists():
        start_ui_server(UI_PORT, str(ui_dir))
        print(f"[UI] Web UI serving at http://0.0.0.0:{UI_PORT}")
    elif SERVE_UI:
        print(f"[UI] web_ui/ directory not found at {ui_dir}, skipping HTTP server")

    print("=" * 60)
    print("  ROS2 → WebSocket Bridge")
    print(f"  Pose topic:  {POSE_TOPIC}  @ {POSE_FPS} Hz")
    print(f"  Image topic: {IMAGE_TOPIC} @ {IMAGE_FPS} Hz")
    print(f"  Image size:  {IMAGE_WIDTH}x{IMAGE_HEIGHT}  JPEG Q={IMAGE_QUALITY}")
    print(f"  WebSocket:   ws://0.0.0.0:{WS_PORT}")
    if SERVE_UI:
        print(f"  Web UI:      http://0.0.0.0:{UI_PORT}")
    print("=" * 60)

    # Handle Ctrl+C
    def shutdown(*_):
        print("\nShutting down...")
        ros_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    asyncio.run(async_main(ros_node))


if __name__ == "__main__":
    main()
