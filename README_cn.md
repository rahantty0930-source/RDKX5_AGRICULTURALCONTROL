基于RDKX5平台的智能视觉采摘机器人设计与实现
本作品基于RDKX5嵌入式平台，设计并实现一款智能果蔬采摘机器人。系统以RDKX5为主控核心，搭载视觉识别模块，完成果实目标检测、定位与坐标解算，通过控制执行机构实现自动精准采摘。作品创新采用轻量化视觉算法，在保证识别准确率的前提下，大幅提升系统实时性与响应速度，降低硬件功耗。整机结构紧凑、运行稳定，可在田间环境下完成连续、高效、可靠的自动化采摘作业，有效减轻人工劳动强度，提高果蔬采收效率，具备良好的实用价值与推广前景。
## 1. 项目定位

本项目是一个运行在 **RDK X5 边缘计算平台** 上的苹果视觉感知系统。系统通过 GS130WI 双目相机获取矫正图像和深度图，使用部署在 RDK X5 BPU 上的 YOLOv8 量化模型识别红苹果与绿苹果，并结合深度图计算苹果在相机坐标系下的三维坐标。

本项目当前重点完成的是：

```text
视觉识别 → 深度测距 → 三维定位 → 目标锁定/实时距离演示 → 网页打标显示 → PICK 控制信号输出
```

项目保留了与 STM32 或机械臂控制器通信的接口。实际演示中，RDK X5 可以在识别并锁定红苹果后，通过串口发送：

```text
PICK,x,y,z
```

STM32 端只需要判断字符串是否以 `PICK` 开头，即可执行预设动作。这样既能体现“视觉识别触发执行机构”的系统逻辑，又能保证比赛现场演示的稳定性。

> 说明：本开源版本重点展示视觉前端与目标决策，不把机械臂真实闭环采摘作为已完成核心功能描述。

---

## 2. 项目亮点

### 2.1 RDK X5 端侧部署

系统在 RDK X5 上完成图像接入、模型推理、深度测距和目标坐标输出，不依赖云端或外部 GPU。

### 2.2 YOLOv8 BPU 量化推理

苹果检测模型采用 YOLOv8，并转换为 RDK X5 BPU 可运行的 `.bin` 模型。模型输入尺寸为 `640×640`，输入格式为 `NV12`。

### 2.3 红/绿苹果成熟度区分

模型支持两类目标：

```text
green_apple
red_apple
```

其中 `red_apple` 可作为采摘目标，`green_apple` 可作为未成熟目标进行忽略或提示。

### 2.4 双目深度辅助三维定位

系统订阅 GS130WI / StereoNet 输出的矫正图像和深度图：

```text
/StereoNetNode/rectified_image
/StereoNetNode/stereonet_depth
/StereoNetNode/camera_info
```

根据检测框中心区域的有效深度值，结合相机内参计算苹果在相机坐标系下的三维坐标：

```text
camera_xyz_m = [X, Y, Z]
```

### 2.5 两种比赛演示模式

本项目提供两种实用演示模式：

1. **实时距离演示模式**  
   不锁定目标，苹果靠近或远离相机时，画面上的 `Z` 距离实时变化，适合展示双目测距能力。

2. **多帧稳定锁定模式**  
   对连续多帧识别结果进行稳定性判断，满足条件后锁定目标并发布 `/apple_target_json`，适合展示采摘决策逻辑。

### 2.6 网页打标推流

系统发布 `/apple_detect_raw` 打标图像，再通过 `hobot_codec_republish` 转换为 `/image_jpeg`，用于网页端显示苹果识别框、类别、置信度和距离信息。

---

## 3. 系统整体流程

```text
GS130WI 双目相机
        ↓
RDK X5 相机与深度节点
        ↓
/StereoNetNode/rectified_image
/StereoNetNode/stereonet_depth
        ↓
YOLOv8 BPU 量化模型推理
        ↓
苹果检测框、类别、置信度
        ↓
检测框中心区域深度提取
        ↓
相机内参解算三维坐标
        ↓
camera_xyz_m = [X, Y, Z]
        ↓
实时距离演示 或 多帧稳定目标锁定
        ↓
/apple_live_json 或 /apple_target_json
        ↓
网页打标显示 / 终端展示 / STM32 PICK 信号
```

---

## 4. 硬件环境

| 模块 | 说明 |
|---|---|
| 主控平台 | RDK X5 |
| 视觉传感器 | GS130WI 双目深度相机 |
| AI 加速 | RDK X5 BPU |
| 可选执行控制器 | STM32F103C8T6 |
| 相机接入方式 | MIPI 相机接口 |
| 通信方式 | ROS2 Topic、UART 串口 |
| 网页显示 | TogetherROS Web Display / hobot_codec JPEG 推流 |

注意：本项目使用 GS130WI 双目相机的矫正图像与深度图，不是 USB 深度相机方案，也不是 PCIe 相机方案。

---

## 5. 软件环境

| 软件/组件 | 说明 |
|---|---|
| 操作系统 | RDK OS / Ubuntu 22.04 |
| ROS 环境 | TogetherROS Humble |
| 编程语言 | Python 3 |
| AI 推理接口 | `hobot_dnn.pyeasy_dnn` |
| 图像处理 | OpenCV |
| ROS2 消息 | `sensor_msgs`、`std_msgs` |
| 网页推流 | `hobot_codec` |

每次运行前需要加载 ROS2 环境：

```bash
source /opt/tros/humble/setup.bash
```

---

## 6. 开源目录结构

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

## 7. 核心脚本说明

### 7.1 `apple_lock_target_ros.py`

功能：苹果识别、深度定位、多帧稳定目标锁定。

主要流程：

```text
订阅矫正图像
订阅深度图
加载 YOLOv8 BPU 模型
识别 red_apple / green_apple
提取检测框中心深度
计算 camera_xyz_m
连续多帧稳定判断
锁定目标
发布 /apple_target_json
发布 /apple_detect_raw 打标图像
```

适合用于：

```text
正式识别演示、目标锁定、采摘决策输出、STM32 PICK 触发
```

---

### 7.2 `apple_live_distance_ros.py`

功能：实时距离演示，不锁定、不冻结。

该脚本适合比赛现场展示“苹果拉近/拉远时距离变化”。

特点：

```text
实时识别苹果
实时显示检测框
实时显示 X / Y / Z
Z 距离随苹果靠近或远离连续变化
发布 /apple_live_json
发布 /apple_detect_raw 打标图像
```

演示效果：

```text
苹果靠近相机 → Z 值减小
苹果远离相机 → Z 值增大
```

---

### 7.3 `send_pick_to_stm32.py`

功能：锁定红苹果后，通过串口发送 PICK 触发信号。

发送格式：

```text
PICK,-0.380,-0.153,0.551
```

含义：

```text
PICK,苹果X坐标,苹果Y坐标,苹果Z坐标
```

STM32 端可以先不解析坐标，只判断是否收到 `PICK`，收到后执行预设动作。

适合用于说明：

```text
RDK X5 完成视觉识别与目标定位
STM32 接收采摘触发信号
机械臂执行部分可后续扩展
```

---

### 7.4 `vision_demo_panel.py`

功能：终端演示面板。

显示内容：

```text
识别目标
成熟度判断
置信度
X/Y/Z 坐标
目标锁定状态
是否建议采摘
```

适合在不使用网页时，直接在终端向评委展示视觉决策结果。

---

### 7.5 启动脚本

| 脚本 | 功能 |
|---|---|
| `start_camera.sh` | 启动 GS130WI 与 StereoNet 深度节点 |
| `start_lock_apple.sh` | 启动多帧稳定锁定识别 |
| `start_live_distance.sh` | 启动实时距离演示 |
| `start_detect_web.sh` | 将 `/apple_detect_raw` 转为 `/image_jpeg` 网页推流 |
| `start_send_pick.sh` | 启动 RDK → STM32 PICK 串口发送 |
| `start_vision_demo.sh` | 启动终端演示面板 |

---

## 8. ROS2 话题说明

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/StereoNetNode/rectified_image` | `sensor_msgs/Image` | 输入 | 矫正后的相机图像 |
| `/StereoNetNode/stereonet_depth` | `sensor_msgs/Image` | 输入 | 双目深度图 |
| `/StereoNetNode/camera_info` | `sensor_msgs/CameraInfo` | 输入 | 相机内参 |
| `/apple_detect_raw` | `sensor_msgs/Image` | 输出 | 带检测框、类别和距离文字的打标图像 |
| `/apple_target_json` | `std_msgs/String` | 输出 | 锁定模式下的目标 JSON |
| `/apple_live_json` | `std_msgs/String` | 输出 | 实时距离模式下的目标 JSON |
| `/image_jpeg` | JPEG 图像流 | 输出 | Web 页面显示用图像 |

---

## 9. 快速启动

假设代码位于：

```bash
/home/sunrise/apple_project
```

给脚本增加执行权限：

```bash
cd /home/sunrise/apple_project
chmod +x *.sh
chmod +x *.py
```

---

## 10. 演示方式一：实时距离演示

该模式最适合比赛现场展示双目测距效果。

终端 1：启动相机和深度节点

```bash
cd /home/sunrise/apple_project
./start_camera.sh
```

终端 2：启动实时距离识别

```bash
cd /home/sunrise/apple_project
./start_live_distance.sh
```

终端 3：启动网页打标推流

```bash
cd /home/sunrise/apple_project
./start_detect_web.sh
```

网页端查看 `/image_jpeg` 图像。画面中会显示苹果检测框、类别、置信度以及 `X/Y/Z` 坐标。

---

## 11. 演示方式二：目标锁定演示

该模式适合展示“稳定目标锁定”和“采摘决策输出”。

终端 1：

```bash
cd /home/sunrise/apple_project
./start_camera.sh
```

终端 2：

```bash
cd /home/sunrise/apple_project
./start_lock_apple.sh
```

终端 3：

```bash
cd /home/sunrise/apple_project
./start_detect_web.sh
```

查看锁定结果：

```bash
source /opt/tros/humble/setup.bash
ros2 topic echo /apple_target_json
```

典型输出示例：

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

## 12. 演示方式三：发送 PICK 给 STM32

终端 1：

```bash
cd /home/sunrise/apple_project
./start_camera.sh
```

终端 2：

```bash
cd /home/sunrise/apple_project
./start_lock_apple.sh
```

终端 3：

```bash
cd /home/sunrise/apple_project
./start_send_pick.sh
```

当系统识别到红苹果并完成目标锁定后，RDK X5 会通过串口发送：

```text
PICK,x,y,z
```

例如：

```text
PICK,-0.380,-0.153,0.551
```

建议 STM32 端逻辑：

```text
1. 上电后机械臂进入初始化状态
2. 等待串口数据
3. 如果字符串以 PICK 开头
4. 执行预设动作
5. 返回初始化位置
```

---

## 13. 网页推流说明

本项目用于识别结果显示的推流链路是：

```text
/apple_detect_raw  ->  hobot_codec_republish  ->  /image_jpeg
```

推流命令：

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

注意区分两个页面：

```text
/StereoNetNode/stereonet_visual：主要用于查看深度图和距离网格
/apple_detect_raw -> /image_jpeg：用于查看苹果识别框和打标结果
```

如果出现：

```text
Hobot_Codec has not received image for more than 5 seconds
```

通常说明 `/apple_detect_raw` 还没有发布。请先启动相机和识别节点，再启动网页推流。

---

## 14. 模型文件说明

本仓库不包含模型文件：

```text
best_bayese_640x640_nv12.bin
```

原因：

1. 模型文件体积较大；
2. 数据集可能涉及版权；
3. 不同场景下建议重新训练与量化模型。

使用时请将模型文件放到 RDK X5 项目目录，例如：

```text
/home/sunrise/apple_project/best_bayese_640x640_nv12.bin
```

模型要求：

| 项目 | 要求 |
|---|---|
| 模型类型 | YOLOv8 目标检测 |
| 部署格式 | RDK X5 BPU `.bin` |
| 输入尺寸 | 640×640 |
| 输入格式 | NV12 |
| 类别 | `green_apple`、`red_apple` |

---

## 15. 比赛展示建议

推荐现场演示顺序：

```text
1. 启动相机和深度节点
2. 启动实时距离演示
3. 拉近/拉远苹果，展示 Z 距离变化
4. 启动网页推流，展示打标画面
5. 切换到锁定模式
6. 展示 red_apple 被锁定
7. 展示 /apple_target_json 中的 X/Y/Z 坐标
8. 可选：展示 RDK 向 STM32 发送 PICK 信号
```

推荐讲解词：

```text
本系统基于 RDK X5 实现端侧苹果识别与三维定位。GS130WI 双目相机提供矫正图像和深度图，YOLOv8 BPU 量化模型完成红苹果与绿苹果识别。系统根据检测框中心区域深度值计算目标在相机坐标系下的 X、Y、Z 坐标，并可通过网页推流显示检测框与距离信息。识别到稳定红苹果后，系统可以向 STM32 发送 PICK 指令，为后续机械臂执行提供接口。
```

---

## 16. 常见问题

### 16.1 网页推流报错没有收到图像

检查 `/apple_detect_raw` 是否存在：

```bash
source /opt/tros/humble/setup.bash
ros2 topic info /apple_detect_raw
```

如果 `Publisher count` 为 0，请先启动识别脚本。

---

### 16.2 只能看到深度图，看不到苹果识别框

这是因为打开的是深度显示页面。苹果识别打标图需要使用：

```text
/apple_detect_raw -> /image_jpeg
```

不要把深度页面和识别打标页面混淆。

---

### 16.3 相机终端输出很多标定参数

这是正常现象。启动相机时会输出内参、外参、基线、矫正参数等信息。只要以下话题有频率即可：

```bash
ros2 topic hz /StereoNetNode/rectified_image
```

---

### 16.4 RDK 发送 PICK 后 STM32 没反应

检查串口设备、波特率和接线：

```text
RDK TX  -> STM32 RX
RDK GND -> STM32 GND
Baudrate: 9600
Format: 8N1
```

STM32 程序只需要判断字符串是否以 `PICK` 开头。

---

## 17. 当前边界与后续工作

当前开源重点：

```text
苹果识别
双目深度测距
三维定位
目标锁定
网页打标显示
PICK 串口触发信号
```

后续可扩展：

```text
真实机械臂控制
Eye-to-Hand 标定
多目标采摘顺序规划
复杂光照场景数据增强
果园长期实测
完整 Web 控制面板
```
