import math
import os
import tempfile
import subprocess

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from interfaces_pkg.msg import DetectionArray


# Piper 경로
# Piper TTS binary and voice; override with the PIPER_BIN / PIPER_MODEL / PIPER_CONFIG environment variables
PIPER_BIN = os.environ.get("PIPER_BIN", os.path.expanduser("~/piper/piper/piper"))
PIPER_MODEL = os.environ.get("PIPER_MODEL", os.path.expanduser("~/piper/en_US-amy-low.onnx"))
PIPER_CONFIG = os.environ.get("PIPER_CONFIG", os.path.expanduser("~/piper/en_US-amy-low.onnx.json"))


class ObjectFusionNode(Node):
    """
    YOLOv8 detections + Lidar LaserScan 융합 노드

    - YOLO: /detections (DetectionArray)
        · 각 Detection에 class_name, bbox.center.position.x 존재
    - Lidar: /lidar_processed (LaserScan)
        · angle_min ~ angle_max, ranges[]

    동작:
      1) 카메라에서 감지된 객체의 bbox 중심 x좌표를 각도로 변환
      2) 해당 각도 방향의 라이다 거리값을 가져옴
      3) 특정 거리 임계값 이하로 가까워지면 Piper로 음성 안내
         예) "There is a chair 0.8 meters ahead."
    """

    def __init__(self):
        super().__init__("object_fusion_node")

        # ---- 파라미터 선언 ----
        self.declare_parameter("camera_topic", "detections")
        self.declare_parameter("lidar_topic", "lidar_processed")
        self.declare_parameter("image_width", 640.0)         # 카메라 입력 해상도 가로
        self.declare_parameter("camera_fov_deg", 60.0)       # 카메라 수평 화각 (deg)
        self.declare_parameter("distance_threshold", 1.5)    # 이 거리 이내면 말하기 [m]
        self.declare_parameter("min_interval", 2.0)          # 최소 발화 간격 [s]

        # ---- 파라미터 값 로드 ----
        self.camera_topic = self.get_parameter("camera_topic").value
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.image_width = self.get_parameter("image_width").value
        self.camera_fov_deg = self.get_parameter("camera_fov_deg").value
        self.distance_threshold = self.get_parameter("distance_threshold").value
        self.min_interval = self.get_parameter("min_interval").value

        # ---- Subscriber 설정 ----
        self.sub_cam = self.create_subscription(
            DetectionArray,
            self.camera_topic,
            self.camera_callback,
            10
        )

        self.sub_lidar = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.lidar_callback,
            10
        )

        # ---- 상태 변수 ----
        # 현재 프레임에서 감지된 객체 리스트: [(class_name, angle_deg), ...]
        self.current_objects = []
        # 가장 최근에 받은 라이다 스캔
        self.latest_scan = None
        # 마지막으로 말한 시각 (seconds)
        self.last_speech_time = 0.0

        self.get_logger().info(
            f"ObjectFusionNode started. "
            f"camera_topic={self.camera_topic}, lidar_topic={self.lidar_topic}"
        )

    # -------------------------
    #  YOLO Detection 콜백
    # -------------------------
    def camera_callback(self, msg: DetectionArray):
        """
        YOLO 노드에서 오는 DetectionArray를 받아
        각 객체의 class_name + bbox 중심 x좌표를 기반으로 각도를 계산해서 저장
        """
        objects = []

        for det in msg.detections:
            class_name = det.class_name

            # bbox 중심 x 좌표
            cx = det.bbox.center.position.x

            # x → angle 변환
            angle_deg = self.convert_x_to_angle_deg(cx)
            objects.append((class_name, angle_deg))

        self.current_objects = objects

    def convert_x_to_angle_deg(self, cx: float) -> float:
        """
        영상 가로 방향 픽셀 좌표(cx)를 카메라 기준 각도(deg)로 변환.
        - 영상 중앙(cx = image_width/2) → 0 deg (정면)
        - 왼쪽/오른쪽으로 갈수록 ±camera_fov_deg/2
        """
        if self.image_width <= 0:
            return 0.0

        # [0, image_width] → [-0.5, 0.5]
        norm = (cx / self.image_width) - 0.5
        # [-0.5, 0.5] → [-FOV/2, FOV/2]
        angle_deg = norm * self.camera_fov_deg
        return angle_deg

    # -------------------------
    #  Lidar 콜백
    # -------------------------
    def lidar_callback(self, scan: LaserScan):
        """
        최신 LaserScan 메시지를 저장하고,
        카메라에서 들어온 객체 리스트와 융합하여
        말할지 여부를 판단
        """
        self.latest_scan = scan
        self.check_fusion()

    # 특정 각도에서 라이다 거리값 가져오기
    def get_distance_at_angle(self, scan: LaserScan, angle_deg: float):
        """
        LaserScan에서 주어진 각도(deg)에 해당하는 거리값을 가져온다.
        - LaserScan.angle_min / angle_max / angle_increment를 이용
        - angle_deg가 음수이면 360도를 더해서 0~360 범위로 맞춰줌
        """
        if scan.angle_increment == 0.0:
            return None

        # 라이다는 [0, 2π] 기준이라고 가정 (lidar_publisher_node 설정과 일치)
        # 카메라에서 온 각도는 [-FOV/2, FOV/2] 범위 → 음수면 360 더해서 0~360으로 맞춤
        if angle_deg < 0.0:
            angle_deg += 360.0

        angle_rad = math.radians(angle_deg)

        # angle_min ~ angle_max 범위를 벗어나면 무시
        if angle_rad < scan.angle_min or angle_rad > scan.angle_max:
            return None

        index = int((angle_rad - scan.angle_min) / scan.angle_increment)
        if index < 0 or index >= len(scan.ranges):
            return None

        dist = scan.ranges[index]

        # NaN / inf / 비정상 값 제거
        if not (0.05 < dist < 30.0):
            return None

        return dist

    # -------------------------
    #  Fusion 로직
    # -------------------------
    def check_fusion(self):
        """
        - current_objects: 카메라에서 감지한 객체들
        - latest_scan: 가장 최근 라이다 스캔

        각 객체에 대해 해당 각도 방향 라이다 거리값을 조회하고,
        일정 거리 이내이면 Piper로 음성 안내.
        """
        if self.latest_scan is None:
            return
        if not self.current_objects:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        # 최소 발화 간격 체크
        if now_sec - self.last_speech_time < self.min_interval:
            return

        for (class_name, angle_deg) in self.current_objects:
            dist = self.get_distance_at_angle(self.latest_scan, angle_deg)
            if dist is None:
                continue

            if dist < self.distance_threshold:
                # 문장 생성 & 발화
                sentence = f"There is a {class_name} {dist:.1f} meters ahead."
                self.get_logger().info(
                    f"[Fusion] class={class_name}, angle={angle_deg:.1f} deg, dist={dist:.2f} m"
                )
                self.speak(sentence)
                self.last_speech_time = now_sec
                break

    # -------------------------
    #  Piper TTS
    # -------------------------
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
    node = ObjectFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
