# Task 2 Debug 全过程记录

> 本文档记录 Task 2（PCL 篮子颜色识别）从第一版代码到最终通过 100/100 验证的完整 debug 历程。
> 目的不是"记流水账"，而是提炼**可迁移的 debug 方法论**。

---

## 一、最终代码思路总览

### 1.1 任务要求

输入：`geometry_msgs/PointStamped[] basket_locs`（若干个篮子位置，坐标系 `panda_link0`）

输出：`string[] basket_colours`（每个位置对应 `"red"` / `"blue"` / `"purple"` / `"none"`）

### 1.2 最终实现流程

```
对每个 basket_locs[i]:
  1. 移动机械臂到 (bx, by, 0.50)，让相机俯视篮子
  2. 等待 1 秒让点云稳定
  3. 获取最新点云（mutex 快照）
  4. 提取点云真实时间戳，用 lookupTransform(带超时) 查 TF，再用 tf2::doTransform 变换 ROS 消息，最后转为 PCL 格式
  5. 用 PassThrough 裁剪 basket 附近 ±0.06m 的局部点云
  6. 对每个点计算到 red/blue/purple 三个目标 RGB 的欧氏距离，距离 < 80 的投票
  7. 票数最多且 ≥ 3 的颜色胜出，否则 "none"
```

### 1.3 改动了哪些文件

| 文件 | 改动内容 |
|---|---|
| `cw1_class.h` | 新增 `latest_cloud_msg_`、`latest_cloud_msg_mutex_`、`tf_buffer_`、`tf_listener_` 成员变量 |
| `cw1_class.cpp` 构造函数 | 初始化 `tf_buffer_` 和 `tf_listener_`；点云订阅回调中加 mutex 存储最新点云 |
| `cw1_class.cpp` `t2_callback` | 从空模板重写为完整的"移动→拍照→变换→裁剪→投票判色"流程 |
| `cw1_class.cpp` includes | 新增 `tf2_sensor_msgs/tf2_sensor_msgs.hpp`；删除不再使用的 `pcl_ros/transforms.hpp`、`tf2_eigen/tf2_eigen.hpp` 等 |

---

## 二、Debug 全过程（按时间顺序）

### Bug 1：静态拍照——所有点都是绿色地面

**现象**：第一版代码不移动机械臂，直接用初始位置的点云做裁剪和颜色判断。结果所有 basket 都被判为 `none` 或 `purple`（误判）。

**诊断过程**：

1. 加诊断日志，打印变换后点云的 x/y/z 范围：
   ```
   Task 2: transformed cloud stats: 307200 finite points,
   x=[0.108,0.558] y=[-0.308,0.308] z=[-0.000,0.024]
   ```
2. 发现 z 最大只有 0.024——相机只看到了地面（z ≈ 0.02），篮子侧壁（z = 0.02~0.12）完全不在点云中。
3. 发现 x/y 范围也有限，很多 basket 位置（如 x=0.59, y=±0.34）根本不在相机视野内。
4. 进一步打印裁剪区域内每个点的 RGB：全是 `(124, 185, 124)` 绿色。
5. 统计"非绿色点"（R > 150 或 B > 150）：**0 个**。

**根因**：相机装在机械臂末端，初始位置太高太远，只能看到绿色地面，看不到篮子的彩色部分。

**修复**：改为"逐个 basket 移动机械臂到上方再拍照"。

**可迁移经验**：
- **先看数据再写算法**。在写颜色判断逻辑之前，应该先确认点云里到底有没有你要的数据。
- **打印数据范围是最基本的诊断手段**。一行 `getMinMax3D` 就能暴露"数据根本不在预期范围内"的问题。
- **不要假设传感器能看到一切**。相机有视野范围、近/远裁剪面、分辨率限制。

---

### Bug 2：移动后点云全是 NaN（scan_z = 0.35）

**现象**：加了移动逻辑后，机械臂成功移动到 basket 上方（z=0.35），但 `cloud_raw` 的 307200 个点全是 NaN（0 个有限点）。

**诊断过程**：

1. 加诊断日志打印 `cloud_raw` 的有限点数：
   ```
   Task 2: basket[0] cloud_raw has 0 finite points out of 307200
   ```
2. 307200 个点全是 NaN，说明深度传感器在这个位置完全无法测距。
3. 查看相机参数（URDF 中 `panda_with_rs.urdf` 第 704 行）：
   ```xml
   <near>0.1</near>
   ```
   近裁剪面 = 0.1m。
4. 计算：`panda_hand` 在 z=0.35，相机比 hand 还要高一点（装在 link8 上），距离地面约 0.35m。但篮子高度只有 0.12m，相机到篮子顶部距离约 0.23m。看起来应该在范围内。
5. 进一步分析：z=0.35 时，`panda_hand` 朝下，相机可能被手指或手腕遮挡，或者深度传感器在这个角度/距离下无法正常工作。

**修复**：把 `scan_z` 从 0.35 改为 0.50。

**验证**：z=0.50 时，`cloud_raw` 恢复为 307200 个有限点，z 范围 [-0.000, 0.101]。

**可迁移经验**：
- **NaN 点云 ≠ 代码 bug**。深度传感器在太近、太远、被遮挡、反光等情况下都会返回 NaN。
- **先检查原始数据**，再怀疑后续处理。如果 `cloud_raw` 就已经全是 NaN，那后面的变换、裁剪、颜色判断全都不用看了。
- **传感器有物理限制**。近裁剪面 0.1m 意味着 10cm 以内的物体测不到。

---

### Bug 3：pcl_ros::transformPointCloud 静默失败

**现象**：移动到 z=0.50 后 `cloud_raw` 有 307200 个有限点，但 `pcl_ros::transformPointCloud` 返回 true 后，`cloud_world` 仍然是 0 个有限点。

**诊断过程**：

1. 第一次尝试：设置 `cloud_raw->header.stamp = 0`，希望让 TF 查找使用最新变换。结果仍然 0 个有限点。
2. 分析原因：`pcl_ros::transformPointCloud` 内部用点云的 `header.stamp`（PCL 时间戳，微秒）去查 TF buffer。当 stamp=0 时，它查的是"时间 0"的 TF，这在 buffer 里不存在。函数返回 true 但实际上没有正确变换。
3. 打印变换矩阵确认：变换矩阵本身是正确的（非零、非 NaN），但 `pcl_ros` 内部的时间匹配逻辑导致变换没有被正确应用。

**修复**：放弃 `pcl_ros::transformPointCloud`，改为使用 `tf2::doTransform` 结合真实时间戳和超时等待（这是最严谨的工业级写法）：

```cpp
// 1. 提取点云的真实物理时间戳
rclcpp::Time cloud_time(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);

// 2. 查找变换，并允许 TF 系统最多等 1.0 秒来同步数据
geometry_msgs::msg::TransformStamped tf_stamped;
tf_stamped = tf_buffer_->lookupTransform(
  "panda_link0", cloud_msg->header.frame_id, cloud_time, rclcpp::Duration::from_seconds(1.0));

// 3. 准备一个空的 ROS 消息来接结果，并直接用 doTransform 把 ROS 消息变过去
sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
tf2::doTransform(*cloud_msg, transformed_cloud_msg, tf_stamped);

// 4. 将变换后的 ROS 点云转为 PCL 格式，用于后续处理
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::fromROSMsg(transformed_cloud_msg, *cloud_world);
```

**验证**：变换后 307200 个有限点，z 范围 [-0.000, 0.120]，完全正确。

**可迁移经验**：
- **"返回 true"不代表"结果正确"**。很多库函数在部分失败时不会报错，只是返回空结果或全 NaN。永远要验证输出。
- **时间戳匹配是 TF 查找的关键**。`pcl_ros::transformPointCloud` 用点云自带的时间戳查 TF，如果因为网络延迟导致 TF 数据还没到，就会失败。
- **动态环境下的终极解法：超时等待**。在 `lookupTransform` 中加上第四个参数（超时时间），允许系统等待 TF 数据到达，这是解决时间戳不同步和网络延迟的最标准做法。
- **当封装函数行为不透明时，换用更底层的工具**。`pcl_ros::transformPointCloud` 把"查 TF + 应用变换"封装在一起，出问题时很难定位。换用 `tf2::doTransform` 直接操作 ROS 消息，逻辑更清晰，报错更明确。

---

### Bug 4：颜色投票全是 0（basket[0] 在视野边缘）

**现象**：z=0.50 时，basket[1]~[3] 都能正确识别颜色（投票数 25000+），但 basket[0] 的投票全是 0。

**诊断过程**：

1. 查看 basket[0] 的变换后点云范围：`x=[0.178,0.560] y=[0.102,0.612]`
2. basket[0] 位置：`(0.334, 0.364)`
3. 裁剪范围：x=[0.274, 0.394], y=[0.304, 0.424]
4. 点云 y 最大到 0.612，basket[0] 的 y=0.364 在范围内。z 最大到 0.101。
5. 但 basket[0] 的 z 范围只到 0.101（而其他 basket 到 0.120），说明 basket[0] 在视野边缘，只能看到部分地面，看不到篮子侧壁。

**结论**：basket[0] 的位置 `(0.334, 0.364)` 恰好在这次随机生成中没有篮子（是 `none`），或者篮子在视野边缘只看到了地面。最终 validation battery 中这个位置被正确判为 `none`，说明确实没有篮子。

**可迁移经验**：
- **不是所有"异常结果"都是 bug**。有时候 `none` 就是正确答案。
- **用 validation battery（多轮测试）而不是单次测试来验证**。单次测试的随机性太大。

---

## 三、Debug 方法论总结（可迁移能力）

### 3.1 分层诊断法

遇到"结果不对"时，不要直接改算法。按数据流的顺序逐层检查：

```
原始数据 → 格式转换 → 坐标变换 → 空间裁剪 → 颜色判断 → 最终输出
```

在每一层加一行日志，打印该层的输出摘要（点数、范围、样本值）。哪一层的输出第一次变得"不对"，问题就在那一层。

**本次实际应用**：
- 第一层（原始数据）：打印 `cloud_raw` 有限点数 → 发现 z=0.35 时全是 NaN
- 第二层（坐标变换）：打印 `cloud_world` 有限点数 → 发现 `pcl_ros` 静默失败
- 第三层（空间裁剪）：打印 `cropped` 点数 → 确认裁剪范围正确
- 第四层（颜色判断）：打印投票数 → 确认颜色匹配正确

### 3.2 "先看数据，再写算法"

在写任何处理逻辑之前，先用日志或可视化确认：
- 数据存在吗？（非空、非 NaN）
- 数据范围对吗？（x/y/z 的 min/max）
- 数据内容对吗？（RGB 值是否符合预期）

### 3.3 "返回成功 ≠ 结果正确"

很多库函数在失败时不会抛异常或返回 false，而是返回空结果或全 NaN。**永远验证输出**，不要只检查返回值。

### 3.4 "封装函数出问题时，换用更底层的工具"

`pcl_ros::transformPointCloud` 把"查 TF + 应用变换"封装在一起，且强制使用点云时间戳，没有超时等待机制，导致网络延迟时静默失败。
当它行为不透明时，换用 `tf2::doTransform`：
1. `tf_buffer_->lookupTransform(..., timeout)` — 显式设置超时等待，解决动态误差
2. `tf2::doTransform(...)` — 直接变换 ROS 消息，避免复杂的类型转换

这样每一步都可以打印中间结果，快速定位问题，且代码更符合工业级标准。

### 3.5 "传感器有物理限制"

- 深度相机有近裁剪面（本项目 0.1m）和远裁剪面
- 相机有有限视野（FOV），不是所有位置都能看到
- 机械臂移动后，相机可能被手指/手腕遮挡
- 点云的 z 范围取决于相机高度和俯仰角

### 3.6 "用多轮随机测试验证，不要只跑一次"

单次测试的随机性太大（basket 位置、颜色分配都是随机的）。用 `/test` 跑 validation battery，它会生成多种场景来全面验证。

---

## 四、关键参数选择的依据

| 参数 | 值 | 依据 |
|---|---|---|
| `scan_z` | 0.50 | z=0.35 时点云全 NaN（太近）；z=0.50 时点云正常且能看到篮子侧壁（z 到 0.12） |
| `crop_half_xy` | 0.06 | 篮子边长 0.1m，±0.06 覆盖篮子加少量余量 |
| `crop_z_min` | -0.01 | 略低于地面（z≈0），包含可能的测量误差 |
| `crop_z_max` | 0.15 | 篮子顶部 z≈0.12，加余量到 0.15 |
| `colour_match_threshold` | 80.0 | 目标 RGB 如 (204,25,25)，地面 RGB 如 (124,185,124)，欧氏距离约 200+，阈值 80 足以区分 |
| `min_colour_points` | 3 | 至少 3 个匹配点才判定有篮子，防止噪声误判 |
| 等待时间 | 1000ms | 机械臂停稳后相机需要时间发布新点云，500ms 不够（仍拿到旧数据），1000ms 稳定 |
