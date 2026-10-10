import math
import time
import os
import tempfile
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import LaserScan


# Piper 설정 (audio_feedback_node와 같은 경로 규칙)
# Piper TTS binary and voice; override with the PIPER_BIN / PIPER_MODEL / PIPER_CONFIG environment variables
PIPER_BIN = os.environ.get("PIPER_BIN", os.path.expanduser("~/piper/piper/piper"))
PIPER_MODEL = os.environ.get("PIPER_MODEL", os.path.expanduser("~/piper/en_US-amy-low.onnx"))
PIPER_CONFIG = os.environ.get("PIPER_CONFIG", os.path.expanduser("~/piper/en_US-amy-low.onnx.json"))


class LidarDistanceSpeakerNode(Node):
    def __init__(self):
        super().__init__("lidar_distance_speaker_node")

        # 파라미터 선언 (필요시 launch 파일에서 바꿔 쓸 수 있음)
        self.declare_parameter("sub_topic", "lidar_processed")
        self.declare_parameter("front_angle_min_deg", -30.0)  # 앞쪽 왼쪽 -30도
        self.declare_parameter("front_angle_max_deg", 30.0)   # 앞쪽 오른쪽 +30도
        self.declare_parameter("min_range", 0.2)              # 최소 거리 [m]
        self.declare_parameter("max_range", 5.0)              # 최대 거리 [m]
        self.declare_parameter("distance_change_threshold", 0.3)  # 이전 발화 대비 거리 변화 임계값 [m]
        self.declare_parameter("min_interval", 2.0)           # 최소 발화 간격 [s]

        self.sub_topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        self.front_angle_min_deg = self.get_parameter("front_angle_min_deg").get_parameter_value().double_value
        self.front_angle_max_deg = self.get_parameter("front_angle_max_deg").get_parameter_value().double_value
        self.min_range = self.get_parameter("min_range").get_parameter_value().double_value
        self.max_range = self.get_parameter("max_range").get_parameter_value().double_value
        self.distance_change_threshold = self.get_parameter("distance_change_threshold").get_parameter_value().double_value
        self.min_interval = self.get_parameter("min_interval").get_parameter_value().double_value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            self.sub_topic,
            self.lidar_callback,
            self.qos_profile,
        )

        # 마지막으로 말한 거리/시간 저장
        self.last_announced_distance = None
        self.last_speech_time = 0.0

        self.get_logger().info(
            f"LidarDistanceSpeakerNode started. "
            f"Subscribing to [{self.sub_topic}], front FOV [{self.front_angle_min_deg}, {self.front_angle_max_deg}] deg."
        )

    # ---------------------------
    #  LaserScan 콜백
    # ---------------------------
    def lidar_callback(self, msg: LaserScan):
        # 앞쪽 특정 각도 범위 내에서 최소 거리 계산
        min_dist = self.get_front_min_distance(msg)

        now = time.time()

        # 말할지 결정
        if min_dist is None:
            # 장애물이 없는데 이전에 어떤 거리값을 말한 상태였다면
            # 필요하면 "No obstacle within X meters" 같은 문장을 넣을 수 있음.
            return

        # 최소 발화 간격 체크
        if now - self.last_speech_time < self.min_interval:
            return

        # 이전에 말한 거리와 비교해서 변화량이 작으면 생략
        if self.last_announced_distance is not None:
            diff = abs(min_dist - self.last_announced_distance)
            if diff < self.distance_change_threshold:
                return

        # 여기까지 왔으면 말하기
        distance_str = f"{min_dist:.1f}"
        sentence = f"Obstacle at {distance_str} meters ahead."

        self.get_logger().info(f"[Piper Speak] {sentence}")
        self.speak(sentence)

        self.last_announced_distance = min_dist
        self.last_speech_time = now

    # ---------------------------
    #  앞쪽 최소 거리 계산
    # ---------------------------
    def get_front_min_distance(self, msg: LaserScan):
        """
        LaserScan에서 설정된 각도 범위 안의 최소 거리 계산.
        유효 범위: [min_range, max_range]
        """
        angle_min = msg.angle_min   # rad
        angle_max = msg.angle_max   # rad
        angle_inc = msg.angle_increment  # rad

        # 각도 범위를 rad로 변환
        front_min_rad = math.radians(self.front_angle_min_deg)
        front_max_rad = math.radians(self.front_angle_max_deg)

        ranges = msg.ranges
        n = len(ranges)

        min_dist = None

        for i in range(n):
            angle = angle_min + i * angle_inc

            # 앞쪽 범위에 해당하지 않으면 패스
            if angle < front_min_rad or angle > front_max_rad:
                continue

            r = ranges[i]

            # NaN / inf / 0 등 무효값 필터링
            if math.isinf(r) or math.isnan(r) or r <= 0.0:
                continue

            # 거리 범위 밖이면 패스
            if r < self.min_range or r > self.max_range:
                continue

            if (min_dist is None) or (r < min_dist):
                min_dist = r

        return min_dist

    # ---------------------------
    #  Piper TTS
    # ---------------------------
    def speak(self, text: str):
        if not os.path.isfile(PIPER_BIN):
            self.get_logger().error(f"Piper binary not found: {PIPER_BIN}")
            return
        if not os.path.isfile(PIPER_MODEL):
            self.get_logger().error(f"Piper model not found: {PIPER_MODEL}")
            return
        if not os.path.isfile(PIPER_CONFIG):
            self.get_logger().error(f"Piper config not found: {PIPER_CONFIG}")
            return

        try:
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as fp:
                wav_path = fp.name

            proc = subprocess.Popen(
                [
                    PIPER_BIN,
                    "--model", PIPER_MODEL,
                    "--config", PIPER_CONFIG,
                    "--output_file", wav_path,
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            proc.communicate(input=text.encode("utf-8"))

            subprocess.Popen(["aplay", wav_path]).wait()

        except Exception as e:
            self.get_logger().error(f"Piper TTS error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarDistanceSpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
