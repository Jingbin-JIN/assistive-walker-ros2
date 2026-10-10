import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import tempfile
import os


# Piper TTS binary and voice; override with the PIPER_BIN / PIPER_MODEL / PIPER_CONFIG environment variables
PIPER_BIN = os.environ.get("PIPER_BIN", os.path.expanduser("~/piper/piper/piper"))
PIPER_MODEL = os.environ.get("PIPER_MODEL", os.path.expanduser("~/piper/en_US-amy-low.onnx"))
PIPER_CONFIG = os.environ.get("PIPER_CONFIG", os.path.expanduser("~/piper/en_US-amy-low.onnx.json"))


class FusionSpeakerNode(Node):
    def __init__(self):
        super().__init__("fusion_speaker_node")

        self.subscription = self.create_subscription(
            String,
            "fusion_result",
            self.fusion_callback,
            10,
        )

        self.last_spoken_text = ""
        self.min_interval = 2.0  # 최소 발화 간격
        self.last_time = 0.0

        self.get_logger().info("FusionSpeakerNode started (Piper TTS Ready).")

    def fusion_callback(self, msg: String):
        text = msg.data
        now = self.get_clock().now().nanoseconds / 1e9

        # 너무 자주 말하지 않기
        if text == self.last_spoken_text and (now - self.last_time) < self.min_interval:
            return

        self.last_spoken_text = text
        self.last_time = now

        self.get_logger().info(f"[Speak] {text}")
        self.speak(text)

    def speak(self, text):
        if not os.path.isfile(PIPER_BIN):
            self.get_logger().error("Piper binary not found.")
            return

        try:
            # 임시 wav 생성
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as fp:
                wav_path = fp.name

            # Piper 실행
            process = subprocess.Popen(
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
            process.communicate(input=text.encode())

            # 재생
            subprocess.Popen(["aplay", wav_path]).wait()

        except Exception as e:
            self.get_logger().error(f"Piper error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FusionSpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
