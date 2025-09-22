"""
vision_prompt_node.py
ROS2 node: subscribes to scan_overlay, sends to vision LLM, publishes annotation.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from nav.vision_prompt import build_prompt, VisionLLMClient

class VisionPromptNode(Node):
    def __init__(self):
        super().__init__('vision_prompt')
        self.bridge = CvBridge()
        self.llm = VisionLLMClient()
        self.prompt = build_prompt()
        self.create_subscription(Image, 'scan_overlay', self.image_callback, 10)
        self.ann_pub = self.create_publisher(String, 'vision_annotation', 10)

    def image_callback(self, msg):
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
