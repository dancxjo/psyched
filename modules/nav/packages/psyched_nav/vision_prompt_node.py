"""ROS 2 node that relays scan overlays to a vision LLM for annotations."""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from psyched_nav.vision_prompt import VisionLLMClient, build_prompt


class VisionPromptNode(Node):
    """Wrap the prompt helpers in a ROS 2 node for experimentation."""

    def __init__(self):
        super().__init__('vision_prompt')
        self.bridge = CvBridge()
        self.llm = VisionLLMClient()
        self.prompt = build_prompt()
        self.create_subscription(Image, 'scan_overlay', self.image_callback, 10)
        self.ann_pub = self.create_publisher(String, 'vision_annotation', 10)

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
