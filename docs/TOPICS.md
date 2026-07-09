# ROS2 话题说明

## 输入话题

```text
/StereoNetNode/rectified_image
```

矫正后的相机图像，通常为 `nv12`，用于苹果检测。

```text
/StereoNetNode/stereonet_depth
```

双目深度图，通常为 `mono16`，用于计算目标距离和三维坐标。

```text
/StereoNetNode/camera_info
```

相机内参，用于由像素坐标和深度值解算相机坐标系下的 `X/Y/Z`。

## 输出话题

```text
/apple_detect_raw
```

带检测框、类别、置信度、三维坐标文字的 BGR 图像。网页推流时应订阅该话题。

```text
/image_jpeg
```

由 `hobot_codec_republish` 将 `/apple_detect_raw` 转码生成，用于 TogetherROS Web Display 显示识别打标画面。

```text
/apple_target_json
```

锁定模式输出的目标 JSON，示例：

```json
{
  "ready_for_arm": true,
  "locked": true,
  "mode": "multi_frame_target_lock",
  "name": "red_apple",
  "score": 0.733,
  "camera_xyz_m": [-0.38, -0.153, 0.551],
  "xyz_std_m": [0.051, 0.025, 0.065],
  "note": "target_locked_do_not_move_camera_or_apple"
}
```

```text
/apple_live_json
```

实时距离模式输出的目标 JSON，不锁定，适合展示拉近/拉远时距离变化。

## 网页推流注意

深度页面和识别打标页面要分清楚：

- 深度页面：显示 StereoNet 深度伪彩色和距离网格。
- 识别打标页面：应使用 `/apple_detect_raw -> /image_jpeg`。

推流命令核心参数：

```bash
-p sub_topic:=/apple_detect_raw
-p pub_topic:=/image_jpeg
-p in_format:=bgr8
-p out_format:=jpeg
```
