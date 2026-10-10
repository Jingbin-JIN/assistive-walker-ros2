import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from interfaces_pkg.msg import DetectionArray, Detection, BoundingBox2D


class TestScenarioNode(Node):
    """
    카메라 + 라이다를 대신하는 시나리오 노드.

    시간 흐름 예:
      0 ~ 2초:   아무것도 없음 (장애물 없음)
      2 ~ 4초:   의자 보이지만 아직 멀어서 obstacle=False
      4 ~ 8초:   의자 + obstacle=True (앞으로 접근해서 가까워진 구간)
      8 ~ 10초:  의자 + obstacle=False (뒤로 물러난 구간)
    """

    def __init__(self):
        super().__init__("test_scenario_node")

        # publishers
        self.det_pub = self.create_publisher(DetectionArray, "detections", 10)
        self.lidar_pub = self.create_publisher(Bool, "lidar_obstacle_info", 10)

        # 0.5초마다 상태 업데이트
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.step = 0
        self.get_logger().info("TestScenarioNode started (chair approach & retreat).")

    def timer_callback(self):
        # 현재 시간 단계 (0.5초 단위)
        t = self.step * self.timer_period

        # 기본값
        obstacle = False
        send_detection = False

        # 시나리오 정의
        if 0.0 <= t < 2.0:
            # 아무것도 없음
            send_detection = False
            obstacle = False
        elif 2.0 <= t < 4.0:
            # 의자 보이지만 아직 멀어서 obstacle=False
            send_detection = True
            obstacle = False
        elif 4.0 <= t < 8.0:
            # 의자 + obstacle=True (가까워짐)
            send_detection = True
            obstacle = True
        elif 8.0 <= t < 10.0:
            # 다시 멀어짐: 의자 보이지만 obstacle=False
            send_detection = True
            obstacle = False
        else:
            # 10초 이후: 루프를 다시 시작
            self.step = 0
            return

        # DetectionArray 구성
        det_array = DetectionArray()
        if send_detection:
            det = Detection()
            det.class_name = "chair"
            det.score = 0.95

            # 대충 화면 중앙에 있는 bbox (실제 값은 의미 없음, fusion_node는 class_name만 사용)
            bbox = BoundingBox2D()
            bbox.center.position.x = 320.0
            bbox.center.position.y = 240.0
            bbox.size.x = 100.0
            bbox.size.y = 150.0
            det.bbox = bbox

            det_array.detections.append(det)

        # lidar Bool
        lidar_msg = Bool()
        lidar_msg.data = obstacle

        # publish
        self.det_pub.publish(det_array)
        self.lidar_pub.publish(lidar_msg)

        self.get_logger().info(
            f"[Scenario] t={t:.1f}s, detect={send_detection}, obstacle={obstacle}"
        )

        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = TestScenarioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
