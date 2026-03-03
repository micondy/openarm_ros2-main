#include <chrono>
#include <atomic>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[]) {
  // ======================== 1) 初始化 ROS2 节点 ========================
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "openarm_move_to_xyz",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // group_name / ee_link / plan_only 仍通过参数控制
  const std::string group_name = node->declare_parameter<std::string>("group_name", "right_arm");
  // 这里的默认值仅作为“手动输入时直接回车”的兜底值
  const double default_x = node->declare_parameter<double>("x", 0.35);
  const double default_y = node->declare_parameter<double>("y", -0.20);
  const double default_z = node->declare_parameter<double>("z", 0.40);
  const std::string ee_link = node->declare_parameter<std::string>("ee_link", "");
  const bool plan_only = node->declare_parameter<bool>("plan_only", false);

  // ======================== 2) 启动 executor（供 MoveIt 通信）========================
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // ======================== 3) 创建 MoveGroup 接口 ========================
  moveit::planning_interface::MoveGroupInterface move_group(node, group_name);

  if (!ee_link.empty()) {
    move_group.setEndEffectorLink(ee_link);
  }

  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);
  move_group.setStartStateToCurrentState();

  // ======================== 4) 交互式控制循环 ========================
  // 支持：
  // - 输入 "x y z"：规划并执行到新目标
  // - 输入 "s"：停止当前轨迹
  // - 输入 "q"：退出程序
  std::atomic<bool> trajectory_active{false};

  std::cout << "\n交互命令：\n"
            << "  - 输入 x y z (m)，例如：0.35 -0.20 0.40\n"
            << "  - 输入 s：停止当前轨迹\n"
            << "  - 输入 q：退出程序\n"
            << "  - 直接回车：使用默认值 " << default_x << " " << default_y << " "
            << default_z << "\n\n";

  while (rclcpp::ok()) {
    std::cout << "请输入目标或命令 [x y z | s | q]: ";

    std::string input_line;
    if (!std::getline(std::cin, input_line)) {
      break;
    }

    if (input_line == "q" || input_line == "Q") {
      RCLCPP_INFO(node->get_logger(), "收到退出命令，程序结束。");
      break;
    }

    if (input_line == "s" || input_line == "S") {
      move_group.stop();
      trajectory_active.store(false);
      RCLCPP_WARN(node->get_logger(), "已发送停止命令。");
      continue;
    }

    double x = default_x;
    double y = default_y;
    double z = default_z;

    if (!input_line.empty()) {
      std::istringstream iss(input_line);
      if (!(iss >> x >> y >> z)) {
        RCLCPP_ERROR(node->get_logger(),
                     "输入格式错误。请使用：x y z，例如：0.35 -0.20 0.40");
        continue;
      }
    }

    // 若正在执行旧轨迹，先停止，再重规划新目标。
    if (trajectory_active.load()) {
      move_group.stop();
      trajectory_active.store(false);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node->get_logger(), "Target position = (%.3f, %.3f, %.3f)", x, y, z);

    // ======================== 5) 组织目标位姿 ========================
    move_group.setStartStateToCurrentState();
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    if (current_pose.pose.orientation.w == 0.0 &&
        current_pose.pose.orientation.x == 0.0 &&
        current_pose.pose.orientation.y == 0.0 &&
        current_pose.pose.orientation.z == 0.0) {
      target_pose.orientation.w = 1.0;
      target_pose.orientation.x = 0.0;
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 0.0;
    } else {
      target_pose.orientation = current_pose.pose.orientation;
    }

    move_group.setPoseTarget(target_pose);

    // ======================== 6) 先规划，再执行 ========================
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = static_cast<bool>(move_group.plan(plan));

    if (!success) {
      RCLCPP_ERROR(node->get_logger(),
                   "Planning failed. group=%s, target=(%.3f, %.3f, %.3f)",
                   group_name.c_str(), x, y, z);
      move_group.clearPoseTargets();
      continue;
    }

    RCLCPP_INFO(node->get_logger(),
                "Planning succeeded. group=%s, target=(%.3f, %.3f, %.3f)",
                group_name.c_str(), x, y, z);

    if (plan_only) {
      RCLCPP_INFO(node->get_logger(), "plan_only=true，跳过执行。");
      move_group.clearPoseTargets();
      continue;
    }

    const bool execute_success = static_cast<bool>(move_group.asyncExecute(plan));
    if (!execute_success) {
      RCLCPP_ERROR(node->get_logger(), "Trajectory start failed.");
      move_group.clearPoseTargets();
      trajectory_active.store(false);
      continue;
    }

    trajectory_active.store(true);
    RCLCPP_INFO(node->get_logger(), "Trajectory started. 可在中途输入新目标或输入 s 停止。");
    move_group.clearPoseTargets();
  }

  // ======================== 7) 清理并退出 ========================
  move_group.stop();
  move_group.clearPoseTargets();
  rclcpp::shutdown();
  return 0;
}
