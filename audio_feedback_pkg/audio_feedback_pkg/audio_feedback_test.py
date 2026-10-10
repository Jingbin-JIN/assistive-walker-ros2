import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import time


class AudioFeedbackTestNode(Node):
    """
    audio_feedback_node 를 테스트하기 위한 시나리오 노드.

    0~3초: 아무 객체 없음 (distance=1.5m)
    3~6초: chair, distance=0.8m   → ZONE1 (삐삐)
    6~9초: chair, distance=0.25m  → ZONE2 (삐——)
    9~12초: 객체 없음, distance=1.5m → 조용
    12~15초: door, distance=0.6m  → ZONE1 (삐삐)
    이후 시간 반복
    """

    def __init__(self):
        super().__init__('audio_feedback_test')

        self.pub_obj = self.create_publisher(String, 'fusion_result', 10)
        self.pub_dist = self.create_publisher(Float32, 'lidar_distance', 10)

        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.timer_cb)  # 20Hz 정도

        self.get_logger().info("AudioFeedbackTestNode started.")

    def timer_cb(self):
        t = time.time() - self.start_time

        obj = String()
        dist = Float32()

        # 0~3초: 아무 객체 없음
        if t < 3:
            obj.data = ""
            dist.data = 1.5

        # 3~6초: 의자 등장 (0.8m)
        elif t < 6:
            obj.data = "chair"
            dist.data = 0.8

        # 6~9초: 더 가까워짐 (0.25m)
        elif t < 9:
            obj.data = "chair"
            dist.data = 0.25

        # 9~12초: 멀어짐 (다시 안전)
        elif t < 12:
            obj.data = ""
            dist.data = 1.5

        # 12~15초: 문 등장 (0.6m)
        elif t < 15:
            obj.data = "door"
            dist.data = 0.6

        # 15초 이후: 다시 처음으로 반복
        else:
            self.start_time = time.time()
            return

        self.pub_obj.publish(obj)
        self.pub_dist.publish(dist)


def main(args=None):
    rclpy.init(args=args)
    node = AudioFeedbackTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
