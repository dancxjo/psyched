"""ROS 2 node that relays scan overlays to a vision LLM for annotations."""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:  # pragma: no cover - requires ROS 2 runtime
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
        SensorDataQoS,
    )
except ImportError:  # pragma: no cover - exercised outside ROS 2
    QoSDurabilityPolicy = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    QoSReliabilityPolicy = None  # type: ignore[assignment]

    def SensorDataQoS():  # type: ignore[misc]
        return 10


def _best_effort_qos(*, depth: int = 10):
    if (
        QoSProfile is None
        or QoSHistoryPolicy is None
        or QoSReliabilityPolicy is None
        or QoSDurabilityPolicy is None
    ):
        return depth
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

from psyched_nav.vision_prompt import VisionLLMClient, build_prompt


class VisionPromptNode(Node):
    """Wrap the prompt helpers in a ROS 2 node for experimentation."""

    def __init__(self):
        super().__init__('vision_prompt')
        self.bridge = CvBridge()
        self.llm = VisionLLMClient()
        self.prompt = build_prompt()
        self.create_subscription(Image, 'scan_overlay', self.image_callback, SensorDataQoS())
        self.ann_pub = self.create_publisher(String, 'vision_annotation', _best_effort_qos(depth=5))

    def image_callback(self, msg):
        """Generate a textual annotation for the incoming overlay image."""

        rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        annotation = self.llm.annotate(rgb_img)
        ann_msg = String()
        ann_msg.data = annotation
        self.ann_pub.publish(ann_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPromptNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
