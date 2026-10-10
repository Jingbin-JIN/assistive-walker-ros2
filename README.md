# assistive-walker-ros2

ROS 2 software of an **AI-based smart walker for visually impaired users**, built in Capstone Design at SungKyunKwan University (September–December 2025) by a ten-person team from electrical, mechanical, civil, physics and materials engineering. A webcam and a low-cost 2D LiDAR on a standard walker frame detect indoor obstacles — doors, chairs, tables, people — ahead of the user and warn by speech and beeps before a collision. Inference ran on a laptop carried on the walker (ROS 2 Humble).

I was one of the team members; this repository holds the software as it was in the team workspace at the end of the project, kept here for reference. The parts the team wrote are separated from the course framework we built on (see *Provenance*).

## How it works

```
webcam ─► image_publisher_node ─► yolov8_node ─► /detections (DetectionArray) ─┐
          (640×480 frames,         (custom YOLOv8 weights,                      ├─► fusion_node ─► /fusion_result ─► fusion_speaker_node (Piper TTS)
           fixed-rate timer)        door / chair / table / person)              │     AND rule, one announcement per class
                                                                                 │
RPLidar ─► lidar_publisher_node ─► lidar_processor_node ─► lidar_obstacle_detector_node ─► /lidar_obstacle_info (Bool)
           /lidar_raw               rotate / flip            forward sector 0–30°, 0.5–2.0 m, 5 consecutive scans
```

- **Camera.** `image_publisher_node` reads the webcam, resizes to 640×480 and publishes `image_raw` on a timer. `yolov8_node` (from the yolov8_ros project) runs a YOLOv8 model — the team's own weights `best.pt`, trained on a self-collected indoor dataset labelled door / chair / table / person (weights not included) — and publishes `DetectionArray` messages.
- **LiDAR.** `lidar_publisher_node` reads an RPLidar over serial, `lidar_processor_node` rotates/flips the scan into the walker's frame, and `lidar_obstacle_detector_node` reports `True` when something is inside the forward sector (0–30°) between 0.5 m and 2.0 m for five consecutive scans (`StabilityDetector`).
- **Fusion and feedback (team-written).** `fusion_node` combines the two channels with an AND rule: when an object is visible *and* the LiDAR confirms an obstacle, it publishes a sentence for the detected class ("There is a chair ahead.") and does not repeat it until a different class is detected. `fusion_speaker_node` speaks that sentence with Piper TTS. Two further variants were written during development: `object_fusion_node` maps each detection's bounding-box centre to a bearing and reads the LiDAR range in that direction itself ("There is a chair 0.8 meters ahead."), and `lidar_distance_speaker_node` announces the nearest obstacle in a ±30° sector. `audio_feedback_node` adds distance zones (beep + object name within 0.3–1.0 m, long beep only below 0.3 m) and takes distances from a `/lidar_distance` topic that only `audio_feedback_test` publishes, so it runs against the test scenario rather than the live pipeline. `traffic_light_color_node` (an HSV colour test on the camera image) is a side experiment.
- Planned but not in the delivered prototype: directional haptic feedback (Arduino and vibration motors) and automatic braking — the walker warned by speech.

## Provenance

| Package / file | Origin |
|---|---|
| `audio_feedback_pkg/` (5 nodes), `fusion_pkg/` (3 nodes) | **written by the team** for the walker |
| `camera_perception_pkg/image_publisher_node.py` | course framework file, **modified by the team** (640×480 camera pipeline on a fixed-rate timer) |
| `camera_perception_pkg/yolov8_node.py` | yolov8_ros by Miguel Ángel González Santamarta (GPL-3), as shipped with the framework; the team pointed it at its own weights and CPU inference |
| `lidar_perception_pkg/lidar_obstacle_detector_node.py` | framework file, with the walker's sector / range / stability parameters (0–30°, 0.5–2.0 m, five scans) |
| rest of `camera_perception_pkg/`, `lidar_perception_pkg/`, `interfaces_pkg/` (yolov8_ros messages) | course framework, as received |

The framework is the ROS 2 workspace of the SungKyunKwan University textbook *ROS 2 기반 자율주행 차량 설계 및 구현* ("Design and implementation of a ROS 2-based autonomous vehicle"), by Hyeong-Keun Hong and colleagues (GPL-3); it was given to the class for an autonomous mini-car, and the walker reused its camera and LiDAR packages. Its lane-following, motion-planning, serial-motor and Arduino packages are not part of the walker and are not included. Two of the framework's helper libraries were distributed as compiled Python 3.10 modules (`*/lib/*.cpython-310.pyc`) loaded by `lib/__init__.py`; they are kept so that the packages run as received, with the readable `.py` versions next to them.

## Running

ROS 2 Humble. The workspace must sit at `~/<workspace>/src` because the framework's `lib/__init__.py` locates its compiled modules from that layout. From the team's run sheet:

```bash
sudo apt install python3-sounddevice python3-numpy
pip install ultralytics opencv-python pyserial              # framework install.sh
# Piper TTS binary and the en_US-amy-low voice under ~/piper (or set PIPER_BIN / PIPER_MODEL / PIPER_CONFIG)

cd ~/ros2_ws
colcon build --symlink-install --packages-select interfaces_pkg camera_perception_pkg lidar_perception_pkg fusion_pkg audio_feedback_pkg
source install/local_setup.bash

ros2 run camera_perception_pkg image_publisher_node        # camera  (ls /dev/video*)
ros2 run camera_perception_pkg yolov8_node                 # detection, expects best.pt in the working directory
ros2 run lidar_perception_pkg lidar_publisher_node         # LiDAR   (ls /dev/ttyUSB*)
ros2 run lidar_perception_pkg lidar_processor_node
ros2 run lidar_perception_pkg lidar_obstacle_detector_node
ros2 run fusion_pkg fusion_node
ros2 run fusion_pkg fusion_speaker_node                    # or: ros2 run audio_feedback_pkg object_fusion_node
```

`fusion_pkg/test_scenario_node` and `audio_feedback_pkg/audio_feedback_test` replay scripted detections and distances so the fusion and feedback nodes can be tried without hardware.

## Notes on this copy

Prepared in October 2026 from the team workspace. Changes: a teammate's home directory in the Piper TTS paths of three nodes replaced by a `~/piper/...` default that environment variables can override; placeholder maintainer / description / license fields in the team packages' manifests filled in; a leftover `CMakeLists.txt` from the package's earlier name removed from `audio_feedback_pkg`; the framework author's machine-name placeholder in two manifests replaced by his name, and their empty description/license fields filled (GPL-3, as the framework's other packages declare). Datasets, videos, model weights and compiled caches other than the two framework libraries are not included. Node logic is unchanged.

## License

GPL-3, because the framework and the yolov8_ros code it ships are GPL-3 (see [LICENSE](LICENSE)); the team's own packages are released under the same license.
