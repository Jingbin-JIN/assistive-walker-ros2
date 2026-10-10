import time

import rclpy
from rclpy.node import Node

from interfaces_pkg.msg import DetectionArray
from std_msgs.msg import Bool, String


class FusionNode(Node):
    """
    English version + one-shot (no repeat speech).
    """

    def __init__(self):
        super().__init__('fusion_node')

        self.sub_detection = self.create_subscription(
            DetectionArray,
            'detections',
            self.camera_callback,
            10
        )

        self.sub_lidar = self.create_subscription(
            Bool,
            'lidar_obstacle_info',
            self.lidar_callback,
            10
        )

        self.pub_fusion = self.create_publisher(
            String,
            'fusion_result',
            10
        )

        # States
        self.current_object = None
        self.lidar_obstacle = False

        self.last_announced_object = None  # prevent repeat
        self.object_visible = False        # track disappearance

        self.get_logger().info("FusionNode started (one-shot mode).")

    def camera_callback(self, msg: DetectionArray):
        if len(msg.detections) == 0:
            # object disappeared → allow future announcements
            self.current_object = None
            self.object_visible = False
            return

        # object exists
        self.current_object = msg.detections[0].class_name.lower()
        self.object_visible = True
        self.try_announce()

    def lidar_callback(self, msg: Bool):
        self.lidar_obstacle = msg.data
        self.try_announce()

    def try_announce(self):
        # must have object + lidar
        if not self.lidar_obstacle:
            return

        if not self.object_visible:
            return

        # already announced this object
        if self.current_object == self.last_announced_object:
            return

        # announce
        sentence = f"There is a {self.current_object} ahead."
        msg = String()
        msg.data = sentence
        self.pub_fusion.publish(msg)

        self.get_logger().info(f"[Fusion] {sentence}")

        # update
        self.last_announced_object = self.current_object


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
