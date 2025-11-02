"""Route selected image streams to the faces module."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image


@dataclass(slots=True)
class SourceTopics:
    image_topic: str
    camera_info_topic: str
    enabled: bool = True


class FacesRouterNode(Node):
    """Republish camera images from the eye module to the faces detector."""

    def __init__(self) -> None:
        super().__init__("psyched_faces_router")

        self._sources: Dict[str, SourceTopics] = {
            "kinect": SourceTopics(
                image_topic=self._declare_string("kinect_image_topic", "/camera/color/image_raw"),
                camera_info_topic=self._declare_string("kinect_camera_info_topic", "/camera/color/camera_info"),
                enabled=self.declare_parameter("kinect_enabled", True).get_parameter_value().bool_value,
            ),
            "usb": SourceTopics(
                image_topic=self._declare_string("usb_image_topic", "/eye/usb/image_raw"),
                camera_info_topic=self._declare_string("usb_camera_info_topic", "/eye/usb/camera_info"),
                enabled=self.declare_parameter("usb_enabled", False).get_parameter_value().bool_value,
            ),
        }

        self._faces_source = self._declare_string("faces_source", "kinect").lower()
        self._fallback_source = self._declare_string("fallback_source", "kinect").lower()

        image_topic = self._declare_string("output_image_topic", "/faces/camera/image_raw")
        camera_info_topic = self._declare_string("output_camera_info_topic", "/faces/camera/camera_info")

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._image_pub = self.create_publisher(Image, image_topic, qos) if image_topic else None
        self._info_pub = self.create_publisher(CameraInfo, camera_info_topic, qos) if camera_info_topic else None

        self._active_source: Optional[str] = None
        self._image_subscription = None
        self._info_subscription = None

        self.add_on_set_parameters_callback(self._handle_parameter_update)
        self._switch_source(self._resolve_source(self._faces_source))

        sources_state = {name: src.enabled for name, src in self._sources.items()}
        image_target = image_topic or "<disabled>"
        info_target = camera_info_topic or "<disabled>"
        self.get_logger().info(
            f"Faces router initialised (sources={sources_state}, output image={image_target}, camera_info={info_target})"
        )

    # Parameter helpers -------------------------------------------------
    def _declare_string(self, name: str, default: str) -> str:
        value = self.declare_parameter(name, default).get_parameter_value().string_value
        return value or default

    def _handle_parameter_update(self, params) -> SetParametersResult:
        updated_source = None
        updated_enabled = {}

        for param in params:
            if param.name == "faces_source" and isinstance(param.value, str):
                updated_source = param.value.lower()
            elif param.name in ("kinect_enabled", "usb_enabled"):
                updated_enabled[param.name.split("_")[0]] = bool(param.value)

        if updated_enabled:
            for name, enabled in updated_enabled.items():
                src = self._sources.get(name)
                if src:
                    src.enabled = enabled

        if updated_source is not None:
            target = self._resolve_source(updated_source)
            self._switch_source(target)
            self._faces_source = updated_source

        return SetParametersResult(successful=True)

    # Routing -----------------------------------------------------------
    def _resolve_source(self, desired: Optional[str]) -> Optional[str]:
        if not desired:
            return None

        desired = desired.lower()
        if desired in ("none", "off"):
            return None
        if desired == "auto":
            for candidate in ("usb", "kinect"):
                src = self._sources.get(candidate)
                if src and src.enabled:
                    return candidate
            return None

        src = self._sources.get(desired)
        if src and src.enabled:
            return desired

        fallback = self._fallback_source
        if fallback and fallback not in ("none", "off", desired):
            return self._resolve_source(fallback)

        self.get_logger().warning(f"Requested faces source '{desired}' is unavailable; routing disabled.")
        return None

    def _switch_source(self, new_source: Optional[str]) -> None:
        if new_source == self._active_source:
            return

        self._destroy_subscriptions()
        self._active_source = new_source

        if new_source is None:
            self.get_logger().info("Faces routing disabled (no active source).")
            return

        config = self._sources.get(new_source)
        if config is None or not config.enabled:
            self.get_logger().warning(f"Source '{new_source}' is not enabled; routing disabled.")
            self._active_source = None
            return

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._image_subscription = self.create_subscription(
            Image, config.image_topic, self._make_image_callback(new_source), qos
        )
        if config.camera_info_topic and self._info_pub is not None:
            self._info_subscription = self.create_subscription(
                CameraInfo, config.camera_info_topic, self._make_info_callback(new_source), qos
            )

        camera_info_target = config.camera_info_topic or "<none>"
        self.get_logger().info(
            f"Faces routing source set to '{new_source}' (image={config.image_topic}, camera_info={camera_info_target})"
        )

    def _destroy_subscriptions(self) -> None:
        if self._image_subscription is not None:
            try:
                self.destroy_subscription(self._image_subscription)
            except Exception:  # pragma: no cover - defensive cleanup
                pass
            self._image_subscription = None
        if self._info_subscription is not None:
            try:
                self.destroy_subscription(self._info_subscription)
            except Exception:  # pragma: no cover - defensive cleanup
                pass
            self._info_subscription = None

    def _make_image_callback(self, source: str):
        def _callback(msg: Image) -> None:
            if self._image_pub is None or self._active_source != source:
                return
            self._image_pub.publish(msg)

        return _callback

    def _make_info_callback(self, source: str):
        def _callback(msg: CameraInfo) -> None:
            if self._info_pub is None or self._active_source != source:
                return
            self._info_pub.publish(msg)

        return _callback

    def destroy_node(self) -> bool:
        self._destroy_subscriptions()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FacesRouterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - manual shutdown
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - script entry point
    main()
