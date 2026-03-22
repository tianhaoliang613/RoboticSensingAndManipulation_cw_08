/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include "../include/cw1_class.h"

#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>


///////////////////////////////////////////////////////////////////////////////

cw1::cw1(const rclcpp::Node::SharedPtr &node)
{
  /* class constructor */

  node_ = node;
  service_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sensor_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // advertise solutions for coursework tasks
  t1_service_ = node_->create_service<cw1_world_spawner::srv::Task1Service>(
    "/task1_start",
    std::bind(&cw1::t1_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t2_service_ = node_->create_service<cw1_world_spawner::srv::Task2Service>(
    "/task2_start",
    std::bind(&cw1::t2_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t3_service_ = node_->create_service<cw1_world_spawner::srv::Task3Service>(
    "/task3_start",
    std::bind(&cw1::t3_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  // Service and sensor callbacks use separate callback groups to align with the
  // current runtime architecture used in cw1_team_0.
  rclcpp::SubscriptionOptions joint_state_sub_options;
  joint_state_sub_options.callback_group = sensor_cb_group_;
  auto joint_state_qos = rclcpp::QoS(rclcpp::KeepLast(50));
  joint_state_qos.reliable();
  joint_state_qos.durability_volatile();
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", joint_state_qos,
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      const int64_t stamp_ns =
        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg->header.stamp.nanosec);
      latest_joint_state_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
      joint_state_msg_count_.fetch_add(1, std::memory_order_relaxed);
    },
    joint_state_sub_options);

  rclcpp::SubscriptionOptions cloud_sub_options;
  cloud_sub_options.callback_group = sensor_cb_group_;
  auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cloud_qos.reliable();
  cloud_qos.durability_volatile();
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/r200/camera/depth_registered/points", cloud_qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      {
        // Protect shared access because callbacks may run concurrently.
        std::lock_guard<std::mutex> lock{this->latest_cloud_msg_mutex_};
        this->latest_cloud_msg_ = msg;
      }
      const int64_t stamp_ns =
        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg->header.stamp.nanosec);
      latest_cloud_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
      cloud_msg_count_.fetch_add(1, std::memory_order_relaxed);
    },
    cloud_sub_options);

  // 初始化 tf2 坐标变换监听器。
  // Buffer 缓存 Gazebo 发布的所有坐标系变换（/tf 话题），
  // TransformListener 在后台自动订阅 /tf 并填充 Buffer。
  // 后续 t2_callback 中用 Buffer 把点云从相机坐标系变换到世界坐标系。
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameter declarations intentionally mirror cw1_team_0 for compatibility.
  const bool use_gazebo_gui = node_->declare_parameter<bool>("use_gazebo_gui", true);
  (void)use_gazebo_gui;
  enable_cloud_viewer_ = node_->declare_parameter<bool>("enable_cloud_viewer", false);
  move_home_on_start_ = node_->declare_parameter<bool>("move_home_on_start", false);
  use_path_constraints_ = node_->declare_parameter<bool>("use_path_constraints", false);
  use_cartesian_reach_ = node_->declare_parameter<bool>("use_cartesian_reach", false);
  allow_position_only_fallback_ = node_->declare_parameter<bool>(
    "allow_position_only_fallback", allow_position_only_fallback_);
  cartesian_eef_step_ = node_->declare_parameter<double>(
    "cartesian_eef_step", cartesian_eef_step_);
  cartesian_jump_threshold_ = node_->declare_parameter<double>(
    "cartesian_jump_threshold", cartesian_jump_threshold_);
  cartesian_min_fraction_ = node_->declare_parameter<double>(
    "cartesian_min_fraction", cartesian_min_fraction_);
  publish_programmatic_debug_ = node_->declare_parameter<bool>(
    "publish_programmatic_debug", publish_programmatic_debug_);
  enable_task1_snap_ = node_->declare_parameter<bool>("enable_task1_snap", false);
  return_home_between_pick_place_ = node_->declare_parameter<bool>(
    "return_home_between_pick_place", return_home_between_pick_place_);
  return_home_after_pick_place_ = node_->declare_parameter<bool>(
    "return_home_after_pick_place", return_home_after_pick_place_);
  pick_offset_z_ = node_->declare_parameter<double>("pick_offset_z", pick_offset_z_);
  task3_pick_offset_z_ = node_->declare_parameter<double>(
    "task3_pick_offset_z", task3_pick_offset_z_);
  task2_capture_enabled_ = node_->declare_parameter<bool>(
    "task2_capture_enabled", task2_capture_enabled_);
  task2_capture_dir_ = node_->declare_parameter<std::string>(
    "task2_capture_dir", task2_capture_dir_);
  place_offset_z_ = node_->declare_parameter<double>("place_offset_z", place_offset_z_);
  grasp_approach_offset_z_ = node_->declare_parameter<double>(
    "grasp_approach_offset_z", grasp_approach_offset_z_);
  post_grasp_lift_z_ = node_->declare_parameter<double>(
    "post_grasp_lift_z", post_grasp_lift_z_);
  gripper_grasp_width_ = node_->declare_parameter<double>(
    "gripper_grasp_width", gripper_grasp_width_);
  joint_state_wait_timeout_sec_ = node_->declare_parameter<double>(
    "joint_state_wait_timeout_sec", joint_state_wait_timeout_sec_);

  if (task2_capture_enabled_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Template capture mode enabled, output dir: %s",
      task2_capture_dir_.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "cw1 template class initialised with compatibility scaffold");
}

///////////////////////////////////////////////////////////////////////////////
// Shared helper functions (used by Task 1 and Task 3)
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Execute the target already configured on a MoveIt planning group.
 * @param group Type: `moveit::planning_interface::MoveGroupInterface &`.
 * Meaning: the MoveIt group to execute, e.g. `panda_arm` or `hand`. The caller
 * should already have set a pose target or joint target on this group.
 * @return `true` if planning and execution both succeed.
 */
bool
cw1::execute_plan(
  moveit::planning_interface::MoveGroupInterface &group)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(group.plan(plan))) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed");
    return false;
  }
  if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed");
    return false;
  }
  return true;
}

/**
 * @brief Move the arm to a target end-effector pose.
 * @param arm Type: `moveit::planning_interface::MoveGroupInterface &`.
 * Meaning: the Panda arm MoveIt group used for planning and execution.
 * @param target Type: `const geometry_msgs::msg::Pose &`.
 * Meaning: desired end-effector pose; position is in metres and orientation is
 * a quaternion.
 * @param start_state Type: `const moveit::core::RobotState *`.
 * Meaning: optional planning start state; use `nullptr` to start from the
 * robot's current state.
 * @param target_link Type: `const char *`.
 * Meaning: name of the end-effector link that should reach `target`, usually
 * `"panda_hand"`.
 * @return `true` if the motion is planned and executed successfully.
 */
bool
cw1::move_arm_to_pose(
  moveit::planning_interface::MoveGroupInterface &arm,
  const geometry_msgs::msg::Pose &target,
  const moveit::core::RobotState *start_state,
  const char *target_link)
{
  // Use the live robot state by default, but allow callers to override it
  // when they need to plan from an explicit start configuration.
  if (start_state == nullptr)
    arm.setStartStateToCurrentState();
  else
    arm.setStartState(*start_state);

  arm.setPoseTarget(target, target_link);
  return execute_plan(arm);
}

/**
 * @brief Execute a short Cartesian move from the current pose to `target`.
 * @param arm Type: `moveit::planning_interface::MoveGroupInterface &`.
 * Meaning: the Panda arm MoveIt group used to compute the Cartesian path.
 * @param target Type: `const geometry_msgs::msg::Pose &`.
 * Meaning: final end-effector pose; position is in metres and orientation is a
 * quaternion.
 * @return `true` if a sufficiently complete Cartesian path is executed.
 */
bool
cw1::cartesian_move(
  moveit::planning_interface::MoveGroupInterface &arm,
  const geometry_msgs::msg::Pose &target)
{
  std::vector<geometry_msgs::msg::Pose> waypoints = {target};
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = arm.computeCartesianPath(
    waypoints, cartesian_eef_step_, cartesian_jump_threshold_, trajectory, true);
  if (fraction < cartesian_min_fraction_) {
    RCLCPP_ERROR(node_->get_logger(), "Cartesian fraction too low: %.3f", fraction);
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory_ = trajectory;
  auto result = arm.execute(cart_plan);
  arm.stop();
  return result == moveit::core::MoveItErrorCode::SUCCESS;
}

/**
 * @brief Open or close the gripper to a requested total width.
 * @param hand Type: `moveit::planning_interface::MoveGroupInterface &`.
 * Meaning: the hand MoveIt group controlling the two finger joints.
 * @param total_width Type: `double`.
 * Meaning: requested total distance between the two fingertips, measured in
 * metres; the function splits it equally across both fingers.
 * @return `true` if the finger joint motion is planned and executed.
 */
bool
cw1::set_gripper(
  moveit::planning_interface::MoveGroupInterface &hand,
  double total_width)
{
  const double per_finger = total_width / 2.0;
  std::vector<double> joint_values = hand.getCurrentJointValues();
  joint_values[0] = per_finger;
  joint_values[1] = per_finger;
  hand.setStartStateToCurrentState();
  hand.setJointValueTarget(joint_values);
  return execute_plan(hand);
}

/**
 * @brief Create a downward-facing grasp pose at `(x, y, z)`.
 * @param x Type: `double`. Meaning: target X coordinate in metres.
 * @param y Type: `double`. Meaning: target Y coordinate in metres.
 * @param z Type: `double`. Meaning: target Z coordinate in metres.
 * @return Pose whose palm faces downward for top-down approach motions.
 */
geometry_msgs::msg::Pose
cw1::make_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);  // Rotate 180 degrees around X so the palm faces downward.
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::t1_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task1Service::Response> response)
{
  (void)response;
  RCLCPP_INFO(node_->get_logger(), "Task 1 callback triggered: starting pick-and-place");

  const auto start_wait = node_->now();
  while (joint_state_msg_count_.load(std::memory_order_relaxed) == 0) {
    if ((node_->now() - start_wait).seconds() > joint_state_wait_timeout_sec_) {
      RCLCPP_WARN(node_->get_logger(), "Task 1: no /joint_states before timeout; continuing");
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  auto arm = moveit::planning_interface::MoveGroupInterface(node_, "panda_arm");
  auto hand = moveit::planning_interface::MoveGroupInterface(node_, "hand");

  arm.setPoseReferenceFrame("panda_link0");
  arm.setEndEffectorLink("panda_hand");
  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);
  hand.setPlanningTime(2.0);
  hand.setNumPlanningAttempts(5);

  const double obj_x = request->object_loc.pose.position.x;
  const double obj_y = request->object_loc.pose.position.y;
  const double obj_z = request->object_loc.pose.position.z;
  const double goal_x = request->goal_loc.point.x;
  const double goal_y = request->goal_loc.point.y;
  const double goal_z = request->goal_loc.point.z;

  const double safe_z = obj_z + pick_offset_z_;
  const double grasp_z = obj_z + grasp_approach_offset_z_;
  const double place_safe_z = goal_z + place_offset_z_;
  const double place_z = goal_z + 0.20;

  // 1. Open gripper
  if (!set_gripper(hand, 0.08)) return;

  // 2. Move above object
  if (!move_arm_to_pose(arm, make_pose(obj_x, obj_y, safe_z))) return;

  // 3. Cartesian descend to grasp height
  if (!cartesian_move(arm, make_pose(obj_x, obj_y, grasp_z))) {
    return;
  }

  // 4. Close gripper
  if (!set_gripper(hand, gripper_grasp_width_)) {
    return;
  }

  // 5. Lift object
  if (!cartesian_move(arm, make_pose(obj_x, obj_y, safe_z))) {
    return;
  }

  // 6. Move above basket
  if (!move_arm_to_pose(arm, make_pose(goal_x, goal_y, place_safe_z))) return;

  // 7. Descend to place height
  if (!cartesian_move(arm, make_pose(goal_x, goal_y, place_z))) {
    return;
  }

  // 8. Open gripper to release (0.06 to avoid tolerance issues)
  set_gripper(hand, 0.06);

  // 9. Retreat upward
  if (!move_arm_to_pose(arm, make_pose(goal_x, goal_y, place_safe_z))) {
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Task 1 callback complete");
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::t2_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task2Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Task 2 callback triggered");

  // ---- 初始化 ----
  const int num_baskets = static_cast<int>(request->basket_locs.size());
  response->basket_colours.assign(num_baskets, "none");

  // 设置 MoveIt arm（和 Task 1 一样的方式）
  auto arm = moveit::planning_interface::MoveGroupInterface(node_, "panda_arm");
  arm.setPoseReferenceFrame("panda_link0");
  arm.setEndEffectorLink("panda_hand");
  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);

  // 观测高度：panda_hand 在 basket 上方多高。
  // 相机装在手腕上，太高看不到篮子颜色，太低可能碰撞。
  // 篮子顶部 z ≈ 0.12，hand 在 0.35 时相机离篮子约 20cm，足够看到侧壁。
  const double scan_z = 0.50;

  // 裁剪和颜色参数
  const double crop_half_xy = 0.06;
  const double crop_z_min   = -0.01;
  const double crop_z_max   = 0.15;
  const int min_colour_points = 3;
  const double colour_match_threshold_sq = 80.0 * 80.0;

  // 题目给定的篮子 RGB（0~1 → 0~255）
  struct TargetColour { const char *name; double r, g, b; };
  const TargetColour targets[] = {
    {"red",    0.8 * 255, 0.1 * 255, 0.1 * 255},
    {"blue",   0.1 * 255, 0.1 * 255, 0.8 * 255},
    {"purple", 0.8 * 255, 0.1 * 255, 0.8 * 255},
  };

  // ---- 逐个 basket 移动 + 拍照 + 判色 ----
  for (int i = 0; i < num_baskets; ++i) {
    const double bx = request->basket_locs[i].point.x;
    const double by = request->basket_locs[i].point.y;

    // 1) 移动机械臂到 basket 上方
    if (!move_arm_to_pose(arm, make_pose(bx, by, scan_z))) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 2: basket[%d] failed to move arm, skipping", i);
      continue;
    }

    // 2) 等待新的点云到达（相机需要时间稳定）
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 3) 获取最新点云
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg;
    {
      std::lock_guard<std::mutex> lock{latest_cloud_msg_mutex_};
      cloud_msg = latest_cloud_msg_;
    }
    if (!cloud_msg) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 2: basket[%d] no point cloud available", i);
      continue;
    }

    // 获取点云真实时间戳
    rclcpp::Time cloud_time(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);

    // 查找 depth → panda_link0 变换（使用点云真实时间戳，并等待最多 1 秒）
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      // 核心通用技能：使用真实时间戳，并给 TF 系统一点时间来同步数据
      tf_stamped = tf_buffer_->lookupTransform(
        "panda_link0", 
        cloud_msg->header.frame_id,
        cloud_time,
        rclcpp::Duration::from_seconds(1.0)); // 等待超时时间
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 2: basket[%d] TF lookup failed for cloud time: %s", i, ex.what());
      continue;
    }

    // 5) 使用 tf2::doTransform 直接变换 ROS 的 PointCloud2 消息
    sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
    tf2::doTransform(*cloud_msg, transformed_cloud_msg, tf_stamped);

    // 6) 将变换后的 ROS 点云转为 PCL 格式，用于后续裁剪和判色
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_world(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(transformed_cloud_msg, *cloud_world);

    // 5) 裁剪 basket 附近的局部点云
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local(new pcl::PointCloud<pcl::PointXYZRGB>);

    pass.setInputCloud(cloud_world);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(bx - crop_half_xy, bx + crop_half_xy);
    pass.filter(*tmp);

    pass.setInputCloud(tmp);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(by - crop_half_xy, by + crop_half_xy);
    pass.filter(*local);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.setInputCloud(local);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(crop_z_min, crop_z_max);
    pass.filter(*cropped);

    RCLCPP_INFO(node_->get_logger(),
      "Task 2: basket[%d] cropped %zu points", i, cropped->points.size());

    // 6) 对每个点投票：找最近的目标颜色，距离够近才计入
    int votes[3] = {0, 0, 0}; // red, blue, purple

    for (const auto &pt : cropped->points) {
      if (!std::isfinite(pt.x)) continue;

      double best_dist_sq = 1e18;
      int best_idx = -1;
      for (int c = 0; c < 3; ++c) {
        const double dr = static_cast<double>(pt.r) - targets[c].r;
        const double dg = static_cast<double>(pt.g) - targets[c].g;
        const double db = static_cast<double>(pt.b) - targets[c].b;
        const double dist_sq = dr * dr + dg * dg + db * db;
        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best_idx = c;
        }
      }
      if (best_dist_sq <= colour_match_threshold_sq)
        ++votes[best_idx];
    }

    RCLCPP_INFO(node_->get_logger(),
      "Task 2: basket[%d] votes R/B/P = (%d, %d, %d)",
      i, votes[0], votes[1], votes[2]);

    // 7) 判定颜色：票数最多且超过阈值
    int best_c = -1;
    int best_v = min_colour_points - 1; // 至少要达到 min_colour_points
    for (int c = 0; c < 3; ++c) {
      if (votes[c] > best_v) {
        best_v = votes[c];
        best_c = c;
      }
    }

    if (best_c >= 0) {
      response->basket_colours[i] = targets[best_c].name;
    }
    // 否则保持 "none"

    RCLCPP_INFO(node_->get_logger(),
      "Task 2: basket[%d] -> '%s'", i, response->basket_colours[i].c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "Task 2: completed");
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::t3_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task3Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task3Service::Response> response)
{
  /* function which should solve task 3 */

  (void)request;
  (void)response;
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "Task 3 callback triggered (template stub). joint_msgs=" <<
      joint_state_msg_count_.load(std::memory_order_relaxed) <<
      ", cloud_msgs=" << cloud_msg_count_.load(std::memory_order_relaxed));
}
