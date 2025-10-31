"""Simple USB/V4L camera capture node for the Psyched eye module."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image

_LOGGER = get_logger("psyched_usb_camera")


@dataclass(slots=True)
class CameraConfig:
    device: str = "/dev/video0"
    width: int = 640
    height: int = 480
    fps: float = 30.0
    encoding: str = "bgr8"
    frame_id: str = "usb_camera"
    image_topic: str = "/eye/usb/image_raw"
    camera_info_topic: str = "/eye/usb/camera_info"


class UsbCameraNode(Node):
    """Publish frames from a USB/V4L camera using OpenCV."""

    _REOPEN_DELAY = 2.0  # seconds
    _MAX_FAILED_FRAMES = 5

    def __init__(self) -> None:
        super().__init__("psyched_usb_camera")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._config = CameraConfig(
            device=self.declare_parameter("device", "/dev/video0").get_parameter_value().string_value,
            width=int(self.declare_parameter("width", 640).get_parameter_value().integer_value or 640),
            height=int(self.declare_parameter("height", 480).get_parameter_value().integer_value or 480),
            fps=float(self.declare_parameter("fps", 30.0).get_parameter_value().double_value or 30.0),
            encoding=self.declare_parameter("encoding", "bgr8").get_parameter_value().string_value or "bgr8",
            frame_id=self.declare_parameter("frame_id", "usb_camera").get_parameter_value().string_value
            or "usb_camera",
            image_topic=self.declare_parameter("image_topic", "/eye/usb/image_raw").get_parameter_value().string_value
            or "/eye/usb/image_raw",
            camera_info_topic=self.declare_parameter("camera_info_topic", "/eye/usb/camera_info")
            .get_parameter_value()
            .string_value
            or "/eye/usb/camera_info",
        )

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._image_pub = self.create_publisher(Image, self._config.image_topic, qos)
        self._camera_info_pub = self.create_publisher(CameraInfo, self._config.camera_info_topic, qos)

        # Keep the capture handle around the node so we can release/reopen safely.
        self._capture: Optional[cv2.VideoCapture] = None
        self._last_failed_read = 0
        self._next_reopen_time = time.monotonic()
        self._timer = self.create_timer(self._timer_period, self._capture_frame)

        self.get_logger().info(
            "USB camera node initialised (device=%s, resolution=%dx%d @ %.1f FPS, encoding=%s)",
            self._config.device,
            self._config.width,
            self._config.height,
            self._config.fps,
            self._config.encoding,
        )

    @property
    def _timer_period(self) -> float:
        fps = max(self._config.fps, 0.5)
        return 1.0 / fps

    def _open_capture(self) -> None:
        with self._lock:
            if self._capture is not None and self._capture.isOpened():
                return

            _LOGGER.info("Opening camera device %s", self._config.device)
            cap = cv2.VideoCapture(self._config.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                _LOGGER.error("Failed to open camera device %s", self._config.device)
                self._capture = None
                self._next_reopen_time = time.monotonic() + self._REOPEN_DELAY
                return

            # Attempt to apply configuration hints.
            if self._config.width > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._config.width))
            if self._config.height > 0:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._config.height))
            if self._config.fps > 0:
                cap.set(cv2.CAP_PROP_FPS, float(self._config.fps))

            self._capture = cap
            self._last_failed_read = 0
            self._next_reopen_time = time.monotonic()
            _LOGGER.info("Camera device %s ready", self._config.device)

    def _capture_frame(self) -> None:
        if self._capture is None or not self._capture.isOpened():
            if time.monotonic() >= self._next_reopen_time:
                self._open_capture()
            return

        with self._lock:
            ret, frame = self._capture.read()

        if not ret or frame is None:
            self._last_failed_read += 1
            if self._last_failed_read == 1:
                self.get_logger().warning("Failed to read frame from %s", self._config.device)
            if self._last_failed_read >= self._MAX_FAILED_FRAMES:
                self.get_logger().error("Camera read failures exceeded threshold; restarting capture")
                self._release_capture()
            return

        self._last_failed_read = 0
        stamp = self.get_clock().now().to_msg()
        try:
            image_msg = self._bridge.cv2_to_imgmsg(frame, encoding=self._config.encoding)
        except Exception as exc:  # pragma: no cover - defensive conversion path
            self.get_logger().error("cv_bridge failed to convert frame: %s", exc)
            return

        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self._config.frame_id
        self._image_pub.publish(image_msg)

        info_msg = self._build_camera_info(stamp, frame.shape[1], frame.shape[0])
        self._camera_info_pub.publish(info_msg)

    def _build_camera_info(self, stamp, width: int, height: int) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._config.frame_id
        info.width = int(width)
        info.height = int(height)
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5

        cx = width / 2.0
        cy = height / 2.0
        # Assume square pixels with focal length equal to width for lack of calibration.
        fx = width
        fy = width

        info.k = [0.0] * 9
        info.k[0] = fx
        info.k[2] = cx
        info.k[4] = fy
        info.k[5] = cy
        info.k[8] = 1.0

        info.r = [0.0] * 9
        info.r[0] = info.r[4] = info.r[8] = 1.0

        info.p = [0.0] * 12
        info.p[0] = fx
        info.p[2] = cx
        info.p[5] = fy
        info.p[6] = cy
        info.p[10] = 1.0

        return info

    def _release_capture(self) -> None:
        with self._lock:
            if self._capture is not None:
                try:
                    self._capture.release()
                except Exception:  # pragma: no cover - cleanup best effort
                    pass
            self._capture = None
        self._next_reopen_time = time.monotonic() + self._REOPEN_DELAY

    def destroy_node(self) -> bool:
        self._release_capture()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - manual shutdown
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - script entry
    main()
