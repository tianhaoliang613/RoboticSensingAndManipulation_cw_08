/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire
solution is contained within the cw1_team_<your_team_number> package */

#include "../include/cw1_class.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

///////////////////////////////////////////////////////////////////////////////

cw1::cw1(const rclcpp::Node::SharedPtr &node): tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_)
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

      latest_cloud_ = msg;


      const int64_t stamp_ns =
        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg->header.stamp.nanosec);
      latest_cloud_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
      cloud_msg_count_.fetch_add(1, std::memory_order_relaxed);
    },
    cloud_sub_options);

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
  RCLCPP_INFO(node_->get_logger(),"Task2 started");

  auto arm = moveit::planning_interface::MoveGroupInterface(node_, "panda_arm");

  arm.setPoseReferenceFrame("panda_link0");
  arm.setEndEffectorLink("panda_hand");
  arm.setPlanningTime(5.0);

  std::vector<geometry_msgs::msg::Pose> views;
  views.push_back(make_pose(0.4,0.3,0.6));
  views.push_back(make_pose(0.4,0.0,0.6));
  views.push_back(make_pose(0.4,-0.3,0.6));

  std::vector<std::string> results(request->basket_locs.size(), "none");
  std::vector<int> best_counts(request->basket_locs.size(), 0);

  for(const auto &v : views)
  {
    move_arm_to_pose(arm,v);
    rclcpp::sleep_for(std::chrono::milliseconds(800));

    if(!latest_cloud_) {
      RCLCPP_WARN(node_->get_logger(), "No point cloud received yet");
      continue;
    }
    RCLCPP_INFO(node_->get_logger(), "Cloud frame: %s", latest_cloud_->header.frame_id.c_str());

    for(size_t i=0;i<request->basket_locs.size();i++)
    {
      geometry_msgs::msg::PointStamped cam_point;

      try
      {
        auto world_point = request->basket_locs[i];
        world_point.header.stamp = rclcpp::Time(0);

        cam_point = tf_buffer_.transform(
          world_point,
          latest_cloud_->header.frame_id,
          tf2::durationFromSec(1.0));
          RCLCPP_INFO(
            node_->get_logger(),
            "Basket %zu in cloud frame: x=%.3f y=%.3f z=%.3f",
            i, cam_point.point.x, cam_point.point.y, cam_point.point.z);
      }
      catch(...)
      {
        continue;
      }

      double tx = cam_point.point.x;
      double ty = cam_point.point.y;

      double r_sum=0, g_sum=0, b_sum=0;
      int count=0;

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_,"x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_,"y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud_,"z");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(*latest_cloud_,"r");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(*latest_cloud_,"g");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(*latest_cloud_,"b");

      for(; iter_x != iter_x.end();
          ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
      {
        double dx = *iter_x - tx;
        double dy = *iter_y - ty;
        double dist = std::sqrt(dx*dx + dy*dy);


        if(dist < 0.04 && *iter_z > 0.00)
        {
          r_sum += *iter_r;
          g_sum += *iter_g;
          b_sum += *iter_b;
          count++;
        }
      }

      RCLCPP_INFO(node_->get_logger(), "Basket %zu candidate count = %d", i, count);
      if(count < 10) continue;

      double r = r_sum / count;
      double g = g_sum / count;
      double b = b_sum / count;

      RCLCPP_INFO(
        node_->get_logger(),
        "Basket %zu avg rgb = (%.1f, %.1f, %.1f)",
        i, r, g, b);

      // double total = r + g + b;

      // if(total < 200) continue;

      std::string color;


      if(r > 100 && b > 100 && g < 90)
        color = "purple";
      else if(r > 100 && r > g + 20 && r > b + 20)
        color = "red";
      else if(b > 100 && b > g + 20 && b > r + 20)
        color = "blue";
      else
        continue;

      RCLCPP_INFO(node_->get_logger(), "Basket %zu classified as %s", i, color.c_str());
      if(count > best_counts[i])
      {
        best_counts[i] = count;
        results[i] = color;
      }
    }
  }

  response->basket_colours = results;

  for(size_t i=0;i<results.size();i++)
  {
    RCLCPP_INFO(node_->get_logger(),
      "Basket %ld -> %s (points=%d)",
      i,
      results[i].c_str(),
      best_counts[i]);
  }

  RCLCPP_INFO(node_->get_logger(),"Task2 finished");
}

///////////////////////////////////////////////////////////////////////////////
void
cw1::t3_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task3Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task3Service::Response> response)
{
  (void)request;
  (void)response;

  RCLCPP_INFO(node_->get_logger(), "Task 3 started");

  // Query Gazebo ground truth positions for debugging.
  {
    auto gz_pose = [&](const char *model_name) {
      std::string cmd = std::string("gz model -m ") + model_name + " -p 2>/dev/null";
      FILE *fp = popen(cmd.c_str(), "r");
      if (!fp) return;
      char buf[256];
      std::string out;
      while (fgets(buf, sizeof(buf), fp)) out += buf;
      pclose(fp);
      if (!out.empty())
        RCLCPP_INFO(node_->get_logger(), "Task3 GT [%s]: %s",
                    model_name, out.c_str());
    };
    // Query all known basket and box model names
    FILE *fp = popen("gz model --list 2>/dev/null", "r");
    if (fp) {
      char buf[256];
      while (fgets(buf, sizeof(buf), fp)) {
        std::string name(buf);
        while (!name.empty() && (name.back() == '\n' || name.back() == '\r'))
          name.pop_back();
        if (name.find("box") != std::string::npos ||
            name.find("basket") != std::string::npos) {
          gz_pose(name.c_str());
        }
      }
      pclose(fp);
    }
  }

  latest_cloud_.reset();

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

  std::vector<geometry_msgs::msg::Pose> views;
  views.push_back(make_pose(0.4,  0.3, 0.6));
  views.push_back(make_pose(0.4,  0.0, 0.6));
  views.push_back(make_pose(0.4, -0.3, 0.6));

  // Cluster: accumulates same-colour points belonging to one object.
  // Tracks overall centroid (for merge) and top-surface centroid (for box grab).
  // "Top" = points within 1cm of the highest point seen so far.
  struct Cluster {
    double sum_x = 0, sum_y = 0;         // all points (for merge distance)
    double top_sum_x = 0, top_sum_y = 0; // top-surface points only
    int top_count = 0;
    double z_max = -1e9;
    int count = 0;

    void add(const geometry_msgs::msg::Point &p) {
      sum_x += p.x;
      sum_y += p.y;
      count++;
      const double top_band = 0.01;
      if (p.z > z_max + top_band) {
        // New highest point — reset top accumulator
        z_max = p.z;
        top_sum_x = p.x;
        top_sum_y = p.y;
        top_count = 1;
      } else if (p.z > z_max - top_band) {
        // Within top band — accumulate
        if (p.z > z_max) z_max = p.z;
        top_sum_x += p.x;
        top_sum_y += p.y;
        top_count++;
      }
    }
  };

  std::vector<Cluster> cl_red, cl_blue, cl_purple;

  auto add_to_clusters = [](std::vector<Cluster> &clusters,
                            const geometry_msgs::msg::Point &p,
                            double thresh)
  {
    for (auto &c : clusters) {
      double cx = c.sum_x / c.count;
      double cy = c.sum_y / c.count;
      double dx = p.x - cx;
      double dy = p.y - cy;
      if (std::sqrt(dx*dx + dy*dy) < thresh) {
        c.add(p);
        return;
      }
    }
    Cluster nc;
    nc.add(p);
    clusters.push_back(nc);
  };

  for (const auto &v : views)
  {
    move_arm_to_pose(arm, v);
    latest_cloud_.reset();  // discard old frame before waiting for a new one
    rclcpp::sleep_for(std::chrono::milliseconds(1500));

    const auto wait_start = node_->now();
    while (!latest_cloud_) {
      if ((node_->now() - wait_start).seconds() > 3.0) {
        RCLCPP_WARN(node_->get_logger(), "Task 3: no point cloud yet");
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Capture a local copy of the shared_ptr so the subscriber callback
    // cannot replace it (and free the data) while we iterate.
    auto cloud = latest_cloud_;
    if (!cloud) {
      continue;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(*cloud, "r");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(*cloud, "g");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(*cloud, "b");

    for (; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
    {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z))
        continue;

      if (*iter_z < 0.2 || *iter_z > 1.5)
        continue;

      std::string color = "none";
      double r = *iter_r;
      double g = *iter_g;
      double b = *iter_b;

      if (r > 100 && b > 100 && g < 90)
        color = "purple";
      else if (r > 100 && r > g + 20 && r > b + 20)
        color = "red";
      else if (b > 100 && b > g + 20 && b > r + 20)
        color = "blue";
      else
        continue;

      geometry_msgs::msg::PointStamped p_cam, p_world;
      p_cam.header = cloud->header;
      p_cam.point.x = *iter_x;
      p_cam.point.y = *iter_y;
      p_cam.point.z = *iter_z;

      try {
        p_world = tf_buffer_.transform(p_cam, "panda_link0", tf2::durationFromSec(1.0));
      }
      catch (...) {
        continue;
      }

      if (p_world.point.x < 0.15 || p_world.point.x > 0.8 ||
          p_world.point.y < -0.5 || p_world.point.y > 0.5)
        continue;

      // Merge radius 0.06: large enough to keep a single cube (diagonal 5.7cm)
      // whole, small enough to separate two cubes (centres >=8cm apart →
      // edge-to-centroid >=6cm > 0.06). Baskets fragment but are merged later.
      if (color == "red") {
        add_to_clusters(cl_red, p_world.point, 0.06);
      } else if (color == "blue") {
        add_to_clusters(cl_blue, p_world.point, 0.06);
      } else if (color == "purple") {
        add_to_clusters(cl_purple, p_world.point, 0.06);
      }
    }
  }

  // Classify clusters using z_max:
  //   box (cube): z_max ≈ 0.04-0.06 (only top face visible)
  //   basket:     z_max ≈ 0.12 (tall walls)
  // Use fixed z for pick/place heights to match Task 1.
  const int min_cluster_points = 200;
  const double z_max_thresh = 0.08;
  // Spawner gives cube centre z ≈ 0.02 (4cm cube on 2cm tile).
  // Spawner gives basket z = tile_thickness = 0.02.
  const double cube_centre_z = 0.02;
  const double basket_spawn_z = 0.02;

  std::vector<geometry_msgs::msg::Point> box_red, box_blue, box_purple;
  std::vector<geometry_msgs::msg::Point> basket_red, basket_blue, basket_purple;

  auto classify = [&](const std::vector<Cluster> &clusters,
                      std::vector<geometry_msgs::msg::Point> &boxes,
                      std::vector<geometry_msgs::msg::Point> &baskets,
                      const char *colour_name)
  {
    // Basket fragments: accumulate all basket-classified clusters and merge
    // into one basket per colour (there is exactly one basket per colour).
    double bsk_sum_x = 0, bsk_sum_y = 0;
    double bsk_top_sum_x = 0, bsk_top_sum_y = 0;
    int bsk_top_count = 0, bsk_total = 0;
    double bsk_z_max = -1e9;

    for (const auto &c : clusters) {
      if (c.count < min_cluster_points) continue;
      bool is_basket = (c.z_max > z_max_thresh);

      double all_x = c.sum_x / c.count;
      double all_y = c.sum_y / c.count;

      RCLCPP_INFO(node_->get_logger(),
        "Task3 [%s %s]: top_xy=(%.3f, %.3f) all_xy=(%.3f, %.3f) "
        "z_max=%.3f count=%d top=%d",
        colour_name, is_basket ? "basket" : "box",
        (c.top_count > 0) ? c.top_sum_x / c.top_count : all_x,
        (c.top_count > 0) ? c.top_sum_y / c.top_count : all_y,
        all_x, all_y, c.z_max, c.count, c.top_count);

      if (is_basket) {
        // Accumulate basket fragments
        bsk_sum_x += c.sum_x;
        bsk_sum_y += c.sum_y;
        bsk_top_sum_x += c.top_sum_x;
        bsk_top_sum_y += c.top_sum_y;
        bsk_top_count += c.top_count;
        bsk_total += c.count;
        if (c.z_max > bsk_z_max) bsk_z_max = c.z_max;
      } else {
        // Each box cluster = one cube.
        // Use detected top-surface z minus half cube height (2 cm) to get the
        // actual cube centre.  This is more reliable than the hardcoded
        // cube_centre_z constant, because it adapts to the real scene height.
        geometry_msgs::msg::Point pt;
        pt.x = (c.top_count > 0) ? c.top_sum_x / c.top_count : all_x;
        pt.y = (c.top_count > 0) ? c.top_sum_y / c.top_count : all_y;
        pt.z = std::max(c.z_max - 0.02, cube_centre_z);  // centre = top - half-height
        boxes.push_back(pt);
      }
    }

    // Emit merged basket (if any fragments found).
    // Use ALL-surface centroid for baskets (not top-surface), because the
    // symmetric wall points cancel camera-angle bias better.  Verified from
    // logs: all_xy error ~1cm vs top_xy error ~4cm for baskets.
    // No position snapping — the spawner adds ±5cm random noise to basket
    // positions, so the nominal locations are not the actual positions.
    if (bsk_total > 0) {
      geometry_msgs::msg::Point pt;
      pt.x = bsk_sum_x / bsk_total;
      pt.y = bsk_sum_y / bsk_total;
      pt.z = basket_spawn_z;
      double top_x = (bsk_top_count > 0) ? bsk_top_sum_x / bsk_top_count : pt.x;
      double top_y = (bsk_top_count > 0) ? bsk_top_sum_y / bsk_top_count : pt.y;
      RCLCPP_INFO(node_->get_logger(),
        "Task3 [%s basket merged]: all_xy=(%.3f,%.3f) top_xy=(%.3f,%.3f) "
        "total=%d top=%d",
        colour_name, pt.x, pt.y, top_x, top_y, bsk_total, bsk_top_count);
      baskets.push_back(pt);
    }
  };

  classify(cl_red, box_red, basket_red, "red");
  classify(cl_blue, box_blue, basket_blue, "blue");
  classify(cl_purple, box_purple, basket_purple, "purple");

  RCLCPP_INFO(node_->get_logger(), "Task3 detected: red boxes=%ld red baskets=%ld",
              box_red.size(), basket_red.size());
  RCLCPP_INFO(node_->get_logger(), "Task3 detected: blue boxes=%ld blue baskets=%ld",
              box_blue.size(), basket_blue.size());
  RCLCPP_INFO(node_->get_logger(), "Task3 detected: purple boxes=%ld purple baskets=%ld",
              box_purple.size(), basket_purple.size());

  // ── Unified close-up refinement scan ─────────────────────────────────────
  // Moves directly above the approximate position and re-scans.  From straight
  // above the perspective error is negligible, so the all-surface centroid of
  // same-colour points gives accurate xy for both cubes and baskets.
  // Also returns the detected z_max, which the caller uses to derive cube
  // grasp height (z_max - half_cube_height).
  //
  // Parameters:
  //   scan_z        – arm height during the scan
  //   search_radius – only consider points within this xy radius of (cx,cy)
  //   fallback_z    – returned as z_max when no points are found
  auto refine_pos = [this, &arm](
      double cx, double cy,
      double scan_z, double search_radius,
      double fallback_z,
      const std::string &colour) -> std::tuple<double, double, double>
  {
    if (!move_arm_to_pose(arm, make_pose(cx, cy, scan_z))) {
      RCLCPP_WARN(node_->get_logger(), "refine_pos: failed to reach scan pose");
      return {cx, cy, fallback_z};
    }

    latest_cloud_.reset();
    rclcpp::sleep_for(std::chrono::milliseconds(800));
    const auto t0 = node_->now();
    while (!latest_cloud_) {
      if ((node_->now() - t0).seconds() > 3.0) break;
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    auto cloud = latest_cloud_;
    if (!cloud) return {cx, cy, fallback_z};

    double sum_x = 0, sum_y = 0, z_max = -1e9;
    int cnt = 0;

    sensor_msgs::PointCloud2ConstIterator<float>   ix(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float>   iy(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float>   iz(*cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ir(*cloud, "r");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ig(*cloud, "g");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ib(*cloud, "b");

    for (; ix != ix.end(); ++ix, ++iy, ++iz, ++ir, ++ig, ++ib) {
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz))
        continue;

      double r = *ir, g = *ig, b = *ib;
      bool match = false;
      if      (colour == "red"    && r > 100 && r > g + 20 && r > b + 20) match = true;
      else if (colour == "blue"   && b > 100 && b > g + 20 && b > r + 20) match = true;
      else if (colour == "purple" && r > 100 && b > 100 && g < 90)        match = true;
      if (!match) continue;

      geometry_msgs::msg::PointStamped p_cam, p_world;
      p_cam.header  = cloud->header;
      p_cam.point.x = *ix; p_cam.point.y = *iy; p_cam.point.z = *iz;
      try {
        p_world = tf_buffer_.transform(p_cam, "panda_link0", tf2::durationFromSec(1.0));
      } catch (...) { continue; }

      double dx = p_world.point.x - cx;
      double dy = p_world.point.y - cy;
      if (std::sqrt(dx*dx + dy*dy) > search_radius) continue;

      sum_x += p_world.point.x;
      sum_y += p_world.point.y;
      if (p_world.point.z > z_max) z_max = p_world.point.z;
      cnt++;
    }

    if (cnt < 20) {
      RCLCPP_WARN(node_->get_logger(),
        "refine_pos [%s]: only %d pts near (%.3f,%.3f), keeping initial estimate",
        colour.c_str(), cnt, cx, cy);
      return {cx, cy, fallback_z};
    }

    double ref_x = sum_x / cnt;
    double ref_y = sum_y / cnt;
    RCLCPP_INFO(node_->get_logger(),
      "refine_pos [%s]: (%.3f,%.3f) -> (%.3f,%.3f)  pts=%d z_max=%.3f",
      colour.c_str(), cx, cy, ref_x, ref_y, cnt, z_max);
    return {ref_x, ref_y, z_max};
  };

  // ── Pick-and-place ────────────────────────────────────────────────────────
  auto pick_and_place_one_colour =
    [this, &arm, &hand, &refine_pos](
      const std::vector<geometry_msgs::msg::Point> &boxes,
      const std::vector<geometry_msgs::msg::Point> &baskets,
      const std::string &colour)
    {
      if (boxes.empty() || baskets.empty())
        return;

      // Refine basket xy once per colour before the pick loop.
      auto [goal_x, goal_y, _bz] = refine_pos(
          baskets[0].x, baskets[0].y,
          /*scan_z=*/0.50, /*radius=*/0.18,
          /*fallback_z=*/baskets[0].z, colour);

      for (size_t bi = 0; bi < boxes.size(); ++bi)
      {

        // Transit height: must clear basket walls (z≈0.12) with margin.
        const double safe_z      = 0.50;
        const double place_safe_z = 0.50;
        // Drop cube above basket rim (panda_hand at 0.26 → fingertips ≈0.16,
        // cube bottom ≈0.14, well above basket wall top ≈0.12).
        const double place_z = 0.26;

        // 1. Open gripper
        if (!set_gripper(hand, 0.08)) continue;

        // 2. Retreat straight up to safe_z from wherever the arm currently is.
        {
          auto cur = arm.getCurrentPose().pose;
          move_arm_to_pose(arm, make_pose(cur.position.x, cur.position.y, safe_z));
        }

        // 3. Close-up refinement scan: move directly above the approximate cube
        //    position at z=0.40 and re-compute a precise xy centroid from the
        //    straight-down view.  z_max - 0.02 gives the cube centre z.
        auto [rx, ry, z_top] = refine_pos(
            boxes[bi].x, boxes[bi].y,
            /*scan_z=*/0.40, /*radius=*/0.12,
            /*fallback_z=*/boxes[bi].z + 0.02, colour);

        const double rz      = std::max(z_top - 0.02, 0.04);  // cube centre (tile z=0.02 + half-cube 0.02)
        const double grasp_z = rz + grasp_approach_offset_z_;

        RCLCPP_INFO(node_->get_logger(),
          "Task3 pick box[%zu] refined=(%.3f,%.3f,%.3f) grasp_z=%.3f "
          "-> basket=(%.3f,%.3f)",
          bi, rx, ry, rz, grasp_z, goal_x, goal_y);

        // 4. Cartesian descend from current close-up scan height to grasp_z.
        //    The arm is already at (rx, ry, 0.40) after the refinement scan,
        //    so a small lateral correction followed by descent is safe.
        if (!move_arm_to_pose(arm, make_pose(rx, ry, 0.40))) continue;
        if (!cartesian_move(arm, make_pose(rx, ry, grasp_z))) continue;

        // 5. Close gripper to cube width so the controller reaches its target
        //    and exits cleanly (no timeout fighting an unreachable position).
        set_gripper(hand, gripper_grasp_width_);

        // 6. Lift straight up (Cartesian keeps the cube from swinging).
        if (!cartesian_move(arm, make_pose(rx, ry, safe_z))) continue;

        // 7. Move above basket
        if (!move_arm_to_pose(arm, make_pose(goal_x, goal_y, place_safe_z))) continue;

        // 8. Descend to release height (above basket rim)
        if (!cartesian_move(arm, make_pose(goal_x, goal_y, place_z))) continue;

        // 9. Open gripper — cube drops into basket
        set_gripper(hand, 0.06);

        // 10. Cartesian retreat upward before any lateral move.
        cartesian_move(arm, make_pose(goal_x, goal_y, place_safe_z));
      }
    };

  pick_and_place_one_colour(box_red,    basket_red,    "red");
  pick_and_place_one_colour(box_blue,   basket_blue,   "blue");
  pick_and_place_one_colour(box_purple, basket_purple, "purple");

  RCLCPP_INFO(node_->get_logger(), "Task 3 finished");
}
