import time
import subprocess
import os

import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32


class AudioFeedbackNode(Node):
    """
    - /fusion_result: 객체 인식 결과 (예: "chair", "door" 등)
    - /lidar_distance: 라이다 최소 거리 [m]

    거리 경고음 규칙:
      d > 1.0 m         → 무음
      0.3 < d ≤ 1.0 m   → 삐— + 객체 이름(영어 TTS)
      d ≤ 0.3 m         → 삐———— (긴 경고음만, 음성 없음)
    """

    def __init__(self):
        super().__init__('audio_feedback_node')

        # Subscribers
        self.sub_fusion = self.create_subscription(
            String, 'fusion_result', self.fusion_callback, 10
        )
        self.sub_distance = self.create_subscription(
            Float32, 'lidar_distance', self.distance_callback, 10
        )

        # 상태 저장
        self.current_object = None      # "chair", "door" 같은 문자열 기대
        self.current_distance = None
        self.last_zone = None
        self.last_beep_time = 0.0
        self.min_interval = 1.0         # 최소 1초 간격으로만 반응

        # Piper 경로 (환경에 맞게 필요하면 수정)
        self.piper_bin = os.path.expanduser('~/piper/piper/piper')
        self.piper_model = os.path.expanduser('~/piper/en_US-amy-low.onnx')
        self.piper_config = os.path.expanduser('~/piper/en_US-amy-low.onnx.json')
        self.piper_tmp_wav = '/tmp/piper_tts.wav'

        self.get_logger().info("AudioFeedbackNode (beep + object TTS) started.")

    # =============================
    # Beep sound functions
    # =============================
    def beep(self, duration=0.2, freq=900):
        """짧은 beep (기본 0.2초)"""
        t = np.linspace(0, duration, int(44100 * duration), False)
        tone = np.sin(2 * np.pi * freq * t) * 0.3
        sd.play(tone, samplerate=44100)
        sd.wait()

    def beep_double(self):
        """짧은 beep 두 번 (현재 사용 X, 필요하면 사용)"""
        self.beep(0.175, 900)
        time.sleep(0.175)
        self.beep(0.175, 900)

    def beep_long(self):
        """긴 경고음 (약 1.5초)"""
        t = np.linspace(0, 1.5, int(44100 * 1.25), False)
        tone = np.sin(2 * np.pi * 900 * t) * 0.3
        sd.play(tone, samplerate=44100)
        sd.wait()

    # =============================
    # TTS (영어, Piper)
    # =============================
    def speak_object_name_en(self, obj_name: str):
        """
        Piper로 객체 이름 영어로 읽어주기.
        예: "chair", "desk", "door" 등.
        """
        text = obj_name.strip()
        if not text:
            return

        # Piper 바이너리/모델 존재 체크 (없으면 조용히 로그만 찍고 리턴)
        if not os.path.isfile(self.piper_bin):
            self.get_logger().warn(f"Piper binary not found: {self.piper_bin}")
            return
        if not os.path.isfile(self.piper_model) or not os.path.isfile(self.piper_config):
            self.get_logger().warn("Piper model/config not found.")
            return

        try:
            # 영어 문장으로 만들어서 조금 더 자연스럽게
            sentence = f"{text}."

            cmd = [
                self.piper_bin,
                '--model', self.piper_model,
                '--config', self.piper_config,
                '--output_file', self.piper_tmp_wav
            ]

            # Piper에 텍스트 전달 → WAV 생성
            proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            stdout, stderr = proc.communicate(sentence)
            if proc.returncode != 0:
                self.get_logger().error(f"Piper error: {stderr}")
                return

            # 생성된 WAV 재생
            if os.path.isfile(self.piper_tmp_wav):
                play_proc = subprocess.Popen(
                    ['aplay', self.piper_tmp_wav],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                play_proc.wait()
            else:
                self.get_logger().warn("Piper output wav not found.")

        except Exception as e:
            self.get_logger().error(f"TTS exception: {e}")

    # =============================
    # Callbacks
    # =============================

    def fusion_callback(self, msg: String):
        # 여기서는 fusion_result가 "chair", "door" 같은 class name 이라고 가정
        text = msg.data.strip()
        self.current_object = text if text else None
        self.try_beep()

    def distance_callback(self, msg: Float32):
        self.current_distance = float(msg.data)
        self.try_beep()

    # =============================
    # Logic
    # =============================

    def get_zone(self, d: float):
        """거리 기반 zone 계산"""
        if d > 1.0:
            return 0  # 안전
        elif d > 0.3:
            return 1  # 주의 (삐— + 객체 이름)
        else:
            return 2  # 매우 근접 (삐—만)

    def try_beep(self):
        # 거리나 객체 정보가 없으면 아무것도 안 함
        if self.current_distance is None or self.current_object is None:
            return

        zone = self.get_zone(self.current_distance)
        now = time.time()

        # zone 0: 안전 구간이면 상태만 리셋하고 종료
        if zone == 0:
            self.last_zone = 0
            return

        # 최소 간격 유지 (너무 자주 울리지 않게)
        if now - self.last_beep_time < self.min_interval:
            return

        # 같은 zone에 계속 있으면 반복 안 함
        if zone == self.last_zone:
            return

        # ===== 실제 동작 =====
        if zone == 1:
            # 0.3 ~ 1.0m: 삐— + 객체 이름
            self.get_logger().info(
                f"[Beep + TTS] ZONE 1 (0.3~1.0m) object={self.current_object}, dist={self.current_distance:.2f}"
            )
            self.beep_long()
            # 객체 이름을 영어로 읽어줌
            self.speak_object_name_en(self.current_object)

        elif zone == 2:
            # 0.3m 이하: 삐—만 (말은 하지 않음)
            self.get_logger().info(
                f"[Beep only] ZONE 2 (<0.3m) object={self.current_object}, dist={self.current_distance:.2f}"
            )
            self.beep_long()

        self.last_zone = zone
        self.last_beep_time = now


def main(args=None):
    rclpy.init(args=args)
    node = AudioFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
