# RDKX5_AGRICULTURALCONTROL
This project is based on the RDK X5 embedded platform and designs an intelligent fruit and vegetable harvesting robot. The system uses the RDK X5 as the main control unit and is equipped with a visual recognition module to complete fruit target detection, localization, and coordinate calculation. It then controls the actuator to achieve automatic and precise harvesting.

The project innovatively adopts a lightweight vision algorithm, which significantly improves real-time performance and response speed while maintaining recognition accuracy and reducing hardware power consumption. The overall system features a compact structure and stable operation, and is capable of continuous, efficient, and reliable automated harvesting in field environments. It can effectively reduce manual labor intensity, improve fruit and vegetable harvesting efficiency, and has strong practical value and broad application prospects.
1. Project Positioning

This project is an apple vision perception system running on the **RDK X5 edge AI platform**. It uses the GS130WI stereo camera to acquire rectified images and depth maps, runs a YOLOv8 model on the RDK X5 BPU to detect red and green apples, and estimates the 3D position of the apple in the camera coordinate system.

The current project focuses on:

```text
Visual detection → Depth estimation → 3D localization → Target locking / live distance demo → Web visualization → PICK trigger output
```

The project also reserves an interface for STM32 or robot-arm controllers. When a red apple is detected and locked, RDK X5 can send a UART command:

```text
PICK,x,y,z
```

The STM32 side can simply check whether the received string starts with `PICK`, and then execute a predefined action sequence.

> Note: This open-source version mainly demonstrates the vision frontend and target-decision output. It does not claim a fully closed-loop robot-arm picking system as a completed feature.

---

## 2. Highlights

### 2.1 Edge deployment on RDK X5

Image input, AI inference, depth estimation, and target-coordinate output are all executed on RDK X5 without cloud computing or an external GPU.

### 2.2 YOLOv8 BPU inference

The apple detection model is based on YOLOv8 and converted to an RDK X5 BPU `.bin` model. The model uses a `640×640` input size and `NV12` input format.

### 2.3 Red/green apple recognition

The model supports two classes:

```text
green_apple
red_apple
```

`red_apple` can be treated as the picking target, while `green_apple` can be treated as an unripe target.

### 2.4 Stereo-depth-based 3D localization

The system subscribes to:

```text
/StereoNetNode/rectified_image
/StereoNetNode/stereonet_depth
/StereoNetNode/camera_info
```

It extracts valid depth values around the center of the detection box and computes the target 3D position using camera intrinsics:

```text
camera_xyz_m = [X, Y, Z]
```

### 2.5 Two demo modes

1. **Live distance mode**  
   No target locking. The Z distance changes continuously when the apple moves closer to or farther away from the camera.

2. **Stable target locking mode**  
   The target is locked after multi-frame stability verification and published through `/apple_target_json`.

### 2.6 Web visualization

The system publishes the annotated image to `/apple_detect_raw`, and `hobot_codec_republish` converts it to `/image_jpeg` for web display.

---

## 3. System Pipeline

```text
GS130WI stereo camera
        ↓
RDK X5 camera and depth nodes
        ↓
/StereoNetNode/rectified_image
/StereoNetNode/stereonet_depth
        ↓
YOLOv8 BPU model inference
        ↓
Class, confidence, bounding box
        ↓
Depth extraction around the target
        ↓
3D coordinate calculation using camera intrinsics
        ↓
camera_xyz_m = [X, Y, Z]
        ↓
Live distance demo or stable target locking
        ↓
/apple_live_json or /apple_target_json
        ↓
Web visualization / Terminal display / STM32 PICK signal
```

---

## 4. Hardware Environment

| Module | Description |
|---|---|
| Main platform | RDK X5 |
| Camera | GS130WI stereo depth camera |
| AI acceleration | RDK X5 BPU |
| Optional controller | STM32F103C8T6 |
| Camera interface | MIPI camera interface |
| Communication | ROS2 topics, UART |
| Display | TogetherROS Web Display / hobot_codec JPEG streaming |

The GS130WI camera is connected through the MIPI camera interface of RDK X5. This is not a USB depth-camera or PCIe-camera solution.

---

## 5. Software Environment

| Software / Component | Description |
|---|---|
| OS | RDK OS / Ubuntu 22.04 |
| ROS | TogetherROS Humble |
| Language | Python 3 |
| Inference API | `hobot_dnn.pyeasy_dnn` |
| Image processing | OpenCV |
| ROS2 messages | `sensor_msgs`, `std_msgs` |
| Web streaming | `hobot_codec` |

Before running any ROS2 script:

```bash
source /opt/tros/humble/setup.bash
```

---

## 6. Repository Structure

```text
rdk_apple_vision_open_source/
├── README.md
├── README_ZH.md
├── README_EN.md
├── COMPETITION_DEMO.md
├── LICENSE_TEMPLATE.txt
├── .gitignore
├── docs/
│   └── TOPICS.md
└── scripts/
    ├── apple_lock_target_ros.py
    ├── apple_live_distance_ros.py
    ├── send_pick_to_stm32.py
    ├── vision_demo_panel.py
    ├── start_camera.sh
    ├── start_lock_apple.sh
    ├── start_live_distance.sh
    ├── start_detect_web.sh
    ├── start_send_pick.sh
    └── start_vision_demo.sh
```

---

## 7. Core Scripts

### 7.1 `apple_lock_target_ros.py`

Function: apple detection, depth-based localization, and multi-frame stable target locking.

Main pipeline:

```text
Subscribe to rectified image
Subscribe to depth map
Load YOLOv8 BPU model
Detect red_apple / green_apple
Extract depth around target center
Compute camera_xyz_m
Verify multi-frame stability
Lock target
Publish /apple_target_json
Publish annotated image /apple_detect_raw
```

Suitable for:

```text
Formal demo, stable target locking, picking-decision output, STM32 PICK trigger
```

---

### 7.2 `apple_live_distance_ros.py`

Function: live distance demo without locking or freezing.

Features:

```text
Real-time apple detection
Real-time bounding box display
Real-time X / Y / Z output
Continuous Z-distance changes
Publish /apple_live_json
Publish /apple_detect_raw
```

Expected behavior:

```text
Apple moves closer  → Z decreases
Apple moves farther → Z increases
```

---

### 7.3 `send_pick_to_stm32.py`

Function: send a UART PICK trigger after a red apple is locked.

Example output:

```text
PICK,-0.380,-0.153,0.551
```

The STM32 program can ignore the coordinates at the beginning stage and only detect the `PICK` prefix.

---

### 7.4 `vision_demo_panel.py`

Function: terminal-based demo panel.

It displays:

```text
Target class
Maturity state
Confidence
X/Y/Z coordinates
Target lock state
Picking suggestion
```

---

## 8. ROS2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/StereoNetNode/rectified_image` | `sensor_msgs/Image` | Input | Rectified camera image |
| `/StereoNetNode/stereonet_depth` | `sensor_msgs/Image` | Input | Stereo depth map |
| `/StereoNetNode/camera_info` | `sensor_msgs/CameraInfo` | Input | Camera intrinsics |
| `/apple_detect_raw` | `sensor_msgs/Image` | Output | Annotated detection image |
| `/apple_target_json` | `std_msgs/String` | Output | Locked target JSON |
| `/apple_live_json` | `std_msgs/String` | Output | Live distance JSON |
| `/image_jpeg` | JPEG stream | Output | Web display image |

---

## 9. Quick Start

Assume the project is located at:

```bash
/home/sunrise/apple_project
```

Add execution permission:

```bash
cd /home/sunrise/apple_project
chmod +x *.sh
chmod +x *.py
```

---

## 10. Demo 1: Live Distance Demo

Terminal 1:

```bash
cd /home/sunrise/apple_project
./start_camera.sh
```

Terminal 2:

```bash
cd /home/sunrise/apple_project
./start_live_distance.sh
```

Terminal 3:

```bash
cd /home/sunrise/apple_project
./start_detect_web.sh
```

The web view will show the detection box, class, confidence, and X/Y/Z coordinates.

---

## 11. Demo 2: Stable Target Locking

Terminal 1:

```bash
cd /home/sunrise/apple_project
./start_camera.sh
```

Terminal 2:

```bash
cd /home/sunrise/apple_project
./start_lock_apple.sh
```

Terminal 3:

```bash
cd /home/sunrise/apple_project
./start_detect_web.sh
```

View locked target output:

```bash
source /opt/tros/humble/setup.bash
ros2 topic echo /apple_target_json
```

Example:

```json
{
  "ready_for_arm": true,
  "locked": true,
  "name": "red_apple",
  "score": 0.733,
  "camera_xyz_m": [-0.380, -0.153, 0.551]
}
```

---

## 12. Demo 3: PICK Signal to STM32

Terminal 1:

```bash
cd /home/sunrise/apple_project
./start_camera.sh
```

Terminal 2:

```bash
cd /home/sunrise/apple_project
./start_lock_apple.sh
```

Terminal 3:

```bash
cd /home/sunrise/apple_project
./start_send_pick.sh
```

When a red apple is locked, RDK X5 sends:

```text
PICK,x,y,z
```

Recommended STM32 logic:

```text
1. Initialize the robot arm
2. Wait for UART input
3. If the string starts with PICK
4. Execute the predefined motion sequence
5. Return to the initial position
```

---

## 13. Web Streaming

The visualization pipeline is:

```text
/apple_detect_raw  ->  hobot_codec_republish  ->  /image_jpeg
```

Command:

```bash
ros2 run hobot_codec hobot_codec_republish --ros-args \
-p channel:=1 \
-p in_mode:=ros \
-p out_mode:=ros \
-p sub_topic:=/apple_detect_raw \
-p pub_topic:=/image_jpeg \
-p in_format:=bgr8 \
-p out_format:=jpeg \
-p enc_qp:=10.0 \
-p jpg_quality:=50.0 \
-p input_framerate:=10 \
-p output_framerate:=-1 \
-p dump_output:=false
```

Important distinction:

```text
/StereoNetNode/stereonet_visual is mainly for depth visualization.
/apple_detect_raw -> /image_jpeg is for apple detection visualization.
```

---

## 14. Model File

The model file is not included:

```text
best_bayese_640x640_nv12.bin
```

Please place your converted BPU model at:

```text
/home/sunrise/apple_project/best_bayese_640x640_nv12.bin
```

Model requirements:

| Item | Requirement |
|---|---|
| Model type | YOLOv8 object detection |
| Deployment format | RDK X5 BPU `.bin` |
| Input size | 640×640 |
| Input format | NV12 |
| Classes | `green_apple`, `red_apple` |

---

## 15. Competition Demo Suggestion

Recommended demo sequence:

```text
1. Start camera and depth nodes
2. Start live distance mode
3. Move the apple closer/farther to show Z changes
4. Start web streaming to show annotated detection image
5. Switch to stable locking mode
6. Show red_apple locked
7. Show /apple_target_json with X/Y/Z coordinates
8. Optionally show PICK command sent to STM32
```

Suggested explanation:

```text
This system performs apple recognition and 3D localization on RDK X5. The GS130WI stereo camera provides rectified images and depth maps, while the YOLOv8 BPU model detects red and green apples. The system estimates the target position in the camera coordinate system and visualizes the result through web streaming. After a stable red apple is locked, RDK X5 can send a PICK command to STM32 for later robot-arm integration.
```

---

## 16. Troubleshooting

### 16.1 Web streaming receives no image

Check whether `/apple_detect_raw` has a publisher:

```bash
source /opt/tros/humble/setup.bash
ros2 topic info /apple_detect_raw
```

If `Publisher count` is 0, start the apple detection node first.

---

### 16.2 Only depth view is visible, no detection box

You are probably viewing the depth visualization page. The detection visualization uses:

```text
/apple_detect_raw -> /image_jpeg
```

---

### 16.3 Many calibration parameters are printed

This is normal when the camera and depth nodes start. Check whether the image topic has a valid frequency:

```bash
ros2 topic hz /StereoNetNode/rectified_image
```

---

### 16.4 STM32 does not respond to PICK

Check wiring and UART settings:

```text
RDK TX  -> STM32 RX
RDK GND -> STM32 GND
Baudrate: 9600
Format: 8N1
```

---

## 17. Current Scope and Future Work

Current focus:

```text
Apple detection
Stereo depth estimation
3D localization
Target locking
Web visualization
PICK UART trigger
```

Future work:

```text
Real robot-arm control
Eye-to-Hand calibration
Multi-target picking sequence planning
Outdoor lighting robustness
Long-term orchard testing
Integrated web dashboard
```
