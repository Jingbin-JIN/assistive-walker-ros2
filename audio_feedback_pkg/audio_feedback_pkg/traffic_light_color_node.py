import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class TrafficLightColorNode(Node):
    def __init__(self):
        super().__init__("traffic_light_color_node")

        # subscribe to image
        self.image_sub = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10,
        )

        # publish detected color
        self.color_pub = self.create_publisher(
            String,
            "yolov8_traffic_light_info",
            10,
        )

        self.bridge = CvBridge()
        self.get_logger().info("TrafficLightColorNode started.")

    def image_callback(self, msg: Image):
        # Image → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg)  # encoding 자동
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        color = self.detect_color(frame)

        color_msg = String()
        color_msg.data = color
        self.color_pub.publish(color_msg)

        self.get_logger().info(f"Traffic light color: {color}")

    def detect_color(self, img):
        """
        이미지에서 신호등이 있는 대략적인 영역 전체를 보고
        Green / Red / Yellow / None 판단
        """

        h, w, _ = img.shape

        # 상단 중앙 쪽만 잘라서 사용 (신호등이 보통 위쪽에 있으니까)
        x1 = int(w * 0.25)
        x2 = int(w * 0.75)
        y1 = int(h * 0.0)
        y2 = int(h * 0.5)
        roi = img[y1:y2, x1:x2]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # H-Mobility 코드에서 쓰던 범위를 참고한 HSV 범위
        hsv_ranges = {
            "red1": (np.array([0, 100, 95]), np.array([10, 255, 255])),
            "red2": (np.array([160, 100, 95]), np.array([179, 255, 255])),
            "yellow": (np.array([20, 100, 95]), np.array([30, 255, 255])),
            "green": (np.array([40, 100, 95]), np.array([90, 255, 255])),
        }

        # 각 색상별 마스크 면적 계산
        masks_area = {}

        # red는 두 구간(red1, red2)을 합쳐서 계산
        red_mask1 = cv2.inRange(hsv, hsv_ranges["red1"][0], hsv_ranges["red1"][1])
        red_mask2 = cv2.inRange(hsv, hsv_ranges["red2"][0], hsv_ranges["red2"][1])
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        masks_area["Red"] = int(cv2.countNonZero(red_mask))

        yellow_mask = cv2.inRange(hsv, hsv_ranges["yellow"][0], hsv_ranges["yellow"][1])
        masks_area["Yellow"] = int(cv2.countNonZero(yellow_mask))

        green_mask = cv2.inRange(hsv, hsv_ranges["green"][0], hsv_ranges["green"][1])
        masks_area["Green"] = int(cv2.countNonZero(green_mask))

        # 가장 면적이 큰 색상 선택
        best_color = "None"
        best_area = 0
        for c, area in masks_area.items():
            if area > best_area:
                best_area = area
                best_color = c

        # 너무 작으면 없는 것으로 처리
        if best_area < 100:  # 필요하면 조절
            return "None"

        return best_color


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightColorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
