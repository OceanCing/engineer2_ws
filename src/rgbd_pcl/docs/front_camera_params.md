# `front_camera.launch` 参数说明

本文对应文件：`src/rgbd_pcl/launch/front_camera.launch`。

> 说明：该 launch 中大部分 `<arg>` 会通过同名 `<param>` 传入 `orbbec_camera/OBCameraNodelet`。布尔参数通常为开关；`-1` 常表示“不设置/使用设备默认”；`0` 在分辨率/FPS 场景中通常表示“自动选择配置”。

## 1) 启动与 Nodelet 管理参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `external_manager` | `false` | 是否使用外部已存在的 nodelet manager。`false` 时本 launch 内创建 manager。 |
| `manager` | `orbbec_camera_manager` | nodelet manager 名称；相机 nodelet 会加载到该 manager。 |
| `required` | `false` | 节点是否设为 required（退出时影响 roslaunch 生命周期）。 |
| `output` | `screen` | 节点日志输出方式（`screen` 或 `log`）。 |
| `respawn` | `false` | 节点异常退出后是否自动拉起。 |

## 2) 基础相机参数（Basic camera）

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `camera_name` | `camera` | 相机实例名/命名空间关键前缀。 |
| `connection_delay` | `100` | 打开设备前延时（通常是 ms 级）。 |
| `log_level` | `none` | 驱动日志级别。 |
| `publish_tf` | `true` | 是否发布相机相关 TF。 |
| `tf_publish_rate` | `0.0` | TF 发布频率；`0.0` 常表示按内部策略。 |
| `enable_frame_sync` | `true` | 是否启用帧同步。 |
| `time_domain` | `global` | 时间域来源（设备/系统/全局策略，依驱动支持）。 |
| `uvc_backend` | `libuvc` | UVC 后端（如 `libuvc` / `v4l2`）。 |
| `device_preset` | `Default` | 设备预设模式（主要针对特定系列）。 |
| `diagnostics_frequency` | `1.0` | diagnostics 发布频率。 |
| `enable_laser` | `true` | 是否开启激光发射（深度主动光）。 |
| `sync_mode` | `standalone` | 多机/同步模式。 |
| `disparity_to_depth_mode` | `HW` | 视差转深度模式（硬件/软件/关闭，依驱动定义）。 |
| `ir_info_uri` | `` | IR 相机标定 URI。空表示不额外加载。 |
| `color_info_uri` | `` | RGB 相机标定 URI。空表示不额外加载。 |

## 3) 彩色相机参数（Color）

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `color_width` | `0` | 彩色流宽度。`0` 常表示自动匹配。 |
| `color_height` | `0` | 彩色流高度。`0` 常表示自动匹配。 |
| `color_fps` | `0` | 彩色流帧率。`0` 常表示自动匹配。 |
| `enable_color` | `true` | 是否开启彩色流。 |
| `color_format` | `ANY` | 彩色图像格式。`ANY` 表示自动选择。 |
| `color_rotation` | `0` | 彩色图旋转角度（常见 0/90/180/270）。 |
| `enable_color_auto_exposure` | `true` | 彩色自动曝光开关。 |
| `color_exposure` | `-1` | 彩色曝光时间；`-1` 表示不手动设置。 |

## 4) 深度相机参数（Depth）

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `depth_width` | `0` | 深度流宽度。`0` 常表示自动匹配。 |
| `depth_height` | `0` | 深度流高度。`0` 常表示自动匹配。 |
| `depth_fps` | `0` | 深度流帧率。`0` 常表示自动匹配。 |
| `enable_depth` | `true` | 是否开启深度流。 |
| `depth_format` | `ANY` | 深度图格式。`ANY` 表示自动选择。 |
| `depth_registration` | `false` | 是否做深度到彩色配准。 |
| `depth_rotation` | `0` | 深度图旋转角度（常见 0/90/180/270）。 |

## 5) 左/右 IR 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `left_ir_width` | `0` | 左 IR 流宽度。 |
| `left_ir_height` | `0` | 左 IR 流高度。 |
| `left_ir_fps` | `0` | 左 IR 帧率。 |
| `enable_left_ir` | `false` | 是否开启左 IR。 |
| `left_ir_format` | `ANY` | 左 IR 格式。 |
| `left_ir_rotation` | `0` | 左 IR 图旋转角度。 |
| `enable_left_ir_sequence_id_filter` | `false` | 是否启用左 IR 序列号过滤。 |
| `left_ir_sequence_id_filter_id` | `-1` | 左 IR 序列过滤 ID。 |
| `right_ir_width` | `0` | 右 IR 流宽度。 |
| `right_ir_height` | `0` | 右 IR 流高度。 |
| `right_ir_fps` | `0` | 右 IR 帧率。 |
| `enable_right_ir` | `false` | 是否开启右 IR。 |
| `right_ir_format` | `ANY` | 右 IR 格式。 |
| `right_ir_rotation` | `0` | 右 IR 图旋转角度。 |
| `enable_right_ir_sequence_id_filter` | `false` | 是否启用右 IR 序列号过滤。 |
| `right_ir_sequence_id_filter_id` | `-1` | 右 IR 序列过滤 ID。 |

## 6) 通用 IR 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enable_ir_auto_exposure` | `true` | IR 自动曝光开关。 |
| `ir_exposure` | `-1` | IR 曝光时间（通常 us）；`-1` 表示不手动设置。 |
| `enable_ir_long_exposure` | `false` | IR 长曝光开关。 |

## 7) 点云与对齐参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enable_point_cloud` | `true` | 是否发布点云。 |
| `enable_colored_point_cloud` | `false` | 是否发布彩色点云。 |
| `ordered_pc` | `false` | 是否输出有序点云（organized）。 |
| `align_mode` | `SW` | 对齐模式（注释说明 335/335L 建议 `SW`）。 |

## 8) 滤波/后处理参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enable_decimation_filter` | `false` | 采样/降采样滤波开关。 |
| `enable_hdr_merge` | `false` | HDR 合并开关。 |
| `enable_sequenced_filter` | `false` | 序列滤波开关。 |
| `enable_threshold_filter` | `false` | 阈值滤波开关。 |
| `enable_hardware_noise_removal_filter` | `false` | 硬件去噪滤波开关。 |
| `enable_noise_removal_filter` | `true` | 软件去噪滤波开关。 |
| `enable_spatial_filter` | `false` | 空间滤波开关。 |
| `enable_temporal_filter` | `false` | 时间滤波开关。 |
| `enable_hole_filling_filter` | `false` | 孔洞填充滤波开关。 |
| `sequence_id_filter_id` | `1` | 序列滤波目标 ID。 |
| `threshold_filter_max` | `-1` | 阈值滤波上限；`-1` 表示不设。 |
| `threshold_filter_min` | `-1` | 阈值滤波下限；`-1` 表示不设。 |
| `hardware_noise_removal_filter_threshold` | `-1.0` | 硬件去噪阈值；`-1.0` 表示不设。 |
| `noise_removal_filter_min_diff` | `256` | 软件去噪最小差异阈值。 |
| `noise_removal_filter_max_size` | `80` | 软件去噪最大连通域/区域尺寸。 |
| `spatial_filter_alpha` | `-1.0` | 空间滤波 alpha；`-1.0` 表示不设。 |
| `spatial_filter_diff_threshold` | `-1` | 空间滤波差分阈值；`-1` 表示不设。 |
| `spatial_filter_magnitude` | `-1` | 空间滤波强度；`-1` 表示不设。 |
| `spatial_filter_radius` | `-1` | 空间滤波半径；`-1` 表示不设。 |
| `temporal_filter_diff_threshold` | `-1.0` | 时间滤波差分阈值；`-1.0` 表示不设。 |
| `temporal_filter_weight` | `-1.0` | 时间滤波权重；`-1.0` 表示不设。 |
| `hole_filling_filter_mode` | `` | 孔洞填充模式（空表示默认）。 |
| `hdr_merge_exposure_1` | `-1` | HDR 合并曝光参数 1。 |
| `hdr_merge_gain_1` | `-1` | HDR 合并增益参数 1。 |
| `hdr_merge_exposure_2` | `-1` | HDR 合并曝光参数 2。 |
| `hdr_merge_gain_2` | `-1` | HDR 合并增益参数 2。 |

## 9) IMU 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enable_sync_output_accel_gyro` | `true` | 是否同步输出加速度计与陀螺仪数据。 |
| `enable_accel` | `true` | 是否开启加速度计流。 |
| `accel_rate` | `200hz` | 加速度计输出频率。 |
| `accel_range` | `4g` | 加速度计量程。 |
| `enable_gyro` | `true` | 是否开启陀螺仪流。 |
| `gyro_rate` | `200hz` | 陀螺仪输出频率。 |
| `gyro_range` | `1000dps` | 陀螺仪量程。 |
| `linear_accel_cov` | `0.01` | 线加速度协方差参数。 |

## 10) 设备连接与杂项参数（Misc）

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `usb_port` | `` | 指定 USB 端口（可用于多设备区分）。 |
| `serial_number` | `` | 指定设备序列号。 |
| `device_num` | `1` | 目标设备数量。 |
| `retry_on_usb3_detection_failure` | `false` | USB3 检测失败时是否重试。 |
| `enable_d2c_viewer` | `false` | 是否启用 D2C 可视化相关功能。 |
| `laser_energy_level` | `-1` | 激光能量等级；`-1` 表示不设。 |
| `enable_ldp` | `true` | 是否启用 LDP（激光相关保护/策略，依设备支持）。 |
| `enable_heartbeat` | `false` | 心跳保活开关。 |
| `enable_hardware_reset` | `false` | 启动时是否执行硬件复位。 |
| `frame_aggregate_mode` | `ANY` | 帧聚合模式（如 `full_frame`/`color_frame`/`ANY`/`disable`）。 |

## 11) 视差搜索偏移参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `disparity_range_mode` | `-1` | 视差搜索范围模式；`-1` 表示不设置。 |
| `disparity_search_offset` | `-1` | 视差搜索偏移量；`-1` 表示不设置。 |
| `disparity_offset_config` | `false` | 是否启用视差偏移配置。 |
| `offset_index0` | `-1` | 视差偏移索引 0。 |
| `offset_index1` | `-1` | 视差偏移索引 1。 |

## 12) 话题重映射（launch 中固定配置）

该 launch 还包含若干 `remap`，将点云、IMU 信息、diagnostics 等话题统一到 `/<camera_name>/...` 命名空间，便于多相机场景下区分。
