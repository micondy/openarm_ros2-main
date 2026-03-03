// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_hardware/v10_simple_hardware.hpp"

#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

/**
 * =============================================================================
 * @file v10_simple_hardware.cpp
 * @brief OpenArm V10 的 ros2_control 硬件接口实现（含可选动力学补偿）
 *
 * 本文件职责：
 * 1) 在生命周期 on_init/on_activate 中完成硬件初始化与使能；
 * 2) 在 read() 周期读取关节状态（位置/速度/力矩）；
 * 3) 在 write() 周期下发 MIT 指令，并可叠加补偿力矩；
 * 4) 对 MoveIt2 / ros2_control 上层保持标准接口兼容。
 *
 * 核心控制关系（MIT 模式）：
 *   τ_motor = Kp*(q_cmd - q) + Kd*(qdot_cmd - qdot) + τ_ff
 *
 * 本实现中前馈项：
 *   τ_ff = τ_user + τ_gravity + τ_coriolis
 *
 * 说明：
 * - τ_user     来自 ros2_control 的 effort command interface；
 * - τ_gravity  由 Dynamics::GetGravity(q) 计算；
 * - τ_coriolis 由 Dynamics::GetCoriolis(q, qdot) 计算。
 * =============================================================================
 */

OpenArm_v10HW::OpenArm_v10HW() = default;

bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // ===========================================================================
  // 配置解析说明
  // - 所有参数都来自 <ros2_control><hardware><param .../></hardware></ros2_control>
  // - 若参数缺失，则使用本函数中的默认值
  // ===========================================================================

  // 1) 基础通信参数
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  it = info.hardware_parameters.find("hand");
  hand_ = (it == info.hardware_parameters.end()) ? true : (it->second == "true");

  it = info.hardware_parameters.find("can_fd");
  can_fd_ = (it == info.hardware_parameters.end()) ? true : (it->second == "true");

  // 2) 补偿功能开关（默认关闭，按需开启）
  it = info.hardware_parameters.find("enable_gravity_comp");
    enable_gravity_comp_ =
      (it != info.hardware_parameters.end()) && (it->second == "true");

  it = info.hardware_parameters.find("enable_coriolis_comp");
    enable_coriolis_comp_ =
      (it != info.hardware_parameters.end()) && (it->second == "true");

  // 3) 动力学链路端点：如果未配置 tip_link，则按 arm_prefix 自动推断。
  it = info.hardware_parameters.find("root_link");
  root_link_ = (it != info.hardware_parameters.end()) ? it->second : "openarm_body_link0";

  it = info.hardware_parameters.find("tip_link");
  if (it != info.hardware_parameters.end()) {
    tip_link_ = it->second;
  } else if (arm_prefix_.empty()) {
    tip_link_ = "openarm_hand";
  } else if (arm_prefix_ == "left_") {
    tip_link_ = "openarm_left_hand";
  } else {
    tip_link_ = "openarm_right_hand";
  }

  // 4) 读取URDF每个关节的 MIT 增益。
  // 注意：这里按 joint1..joint7 的顺序写入 kp_[0..6], kd_[0..6]。
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "【配置总览】CAN=%s | arm_prefix=%s | 手部=%s | CAN-FD=%s | 重力补偿=%s | 科里奥利补偿=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "开启" : "关闭", can_fd_ ? "开启" : "关闭",
              enable_gravity_comp_ ? "开启" : "关闭",
              enable_coriolis_comp_ ? "开启" : "关闭");

  if (enable_gravity_comp_ || enable_coriolis_comp_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "【动力学链】root=%s | tip=%s",
                root_link_.c_str(), tip_link_.c_str());
  }

  return true;
}

void OpenArm_v10HW::generate_joint_names() {
  joint_names_.clear();

  // 关节命名规则必须与 URDF / controller 配置保持一致。
  // 例如：openarm_left_joint1, openarm_right_joint1, openarm_joint1
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  // 生命周期入口点
  // 典型调用顺序：on_init -> on_configure -> on_activate -> (read/write循环)
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  //解析配置
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }
  // 生成关节名称
  generate_joint_names();
  // 验证关节数量（7 臂关节 + 可选 1 夹爪关节）
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }
  // 初始化 OpenArm CAN 通信
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  // 初始化电机            
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);
  // arm 7个关节的 send/recv CAN ID 必须与硬件拓扑一致。

  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Initializing gripper...");
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }
  // 初始化状态/命令向量
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  // 动力学模型使用 URDF 原始 XML，避免依赖额外文件路径。
  // 这样在 launch/仿真/真机环境中都更稳定，不依赖绝对路径。
  if (enable_gravity_comp_ || enable_coriolis_comp_) {
    dynamics_ = std::make_unique<Dynamics>(info.original_xml, root_link_, tip_link_, true);
    if (!dynamics_->Init()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Failed to initialize dynamics solver");
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // 预热一帧通信，确保 activate 前状态缓存有效。
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10HW::export_state_interfaces() {
  // 导出给上层读取的状态接口：position / velocity / effort。
  // 这些值在 read() 中更新。
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10HW::export_command_interfaces() {
  // 导出给上层写入的命令接口：position / velocity / effort。
  // 这些目标值在 write() 中打包成 MITParam。
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // activate 阶段：切到 STATE 回调模式、使能电机、执行回零。
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Activating OpenArm V10...");
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // deactivate 阶段：安全下电，避免残余力矩。
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10...");

  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // read() 数据流：CAN反馈 -> pos/vel/tau 状态接口。
  // 这一步通常由 controller_manager 以固定频率调用。
  openarm_->refresh_all();
  openarm_->recv_all();
  
  // 读取臂关节状态
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }
  // 读取夹爪状态（如果启用）
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // 当前未做精确 gripper 速度/力矩映射，先保持 0。
      // 若后续你需要夹爪力控，可以在这里补完整映射。
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // write() 数据流：controller命令 +（可选）补偿 -> MIT命令 -> CAN。
  // 这里是你后续继续扩展“控制策略/补偿策略”的主入口。
  std::vector<double> gravity_torque(ARM_DOF, 0.0);
  std::vector<double> coriolis_torque(ARM_DOF, 0.0);

  if (dynamics_) {
    if (enable_gravity_comp_) {
      // τ_g = G(q)
      dynamics_->GetGravity(pos_states_.data(), gravity_torque.data());
    }

    if (enable_coriolis_comp_) {
      // τ_c = C(q, qdot) * qdot
      dynamics_->GetCoriolis(pos_states_.data(), vel_states_.data(),
                             coriolis_torque.data());
    }
  }
  // 构建MIT控制指令：前馈力矩 = 用户命令 + 补偿
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  arm_params.reserve(ARM_DOF);
  for (size_t i = 0; i < ARM_DOF; ++i) {
    // 最终前馈力矩：用户命令 + 动力学补偿
    // τ_ff = τ_user + τ_g + τ_c
    // 若你以后加摩擦补偿，可扩展为：
    // τ_ff = τ_user + τ_g + τ_c + τ_friction
    const double tau_ff =
        tau_commands_[i] + gravity_torque[i] + coriolis_torque[i];
    arm_params.push_back(
        {kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], tau_ff});
  }

  openarm_->get_arm().mit_control_all(arm_params);
  // 夹爪控制（如果启用） 
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // gripper 仍按位置主导控制，保持与 MoveIt2 默认控制器兼容。
    // 若后续改为夹爪力控，可将最后一项 tau_ff 作为抓取力前馈。
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_KP, GRIPPER_KD, motor_command, 0, 0}});
  }
  openarm_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void OpenArm_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position...");

  std::vector<openarm::damiao_motor::MITParam> arm_params;
  arm_params.reserve(ARM_DOF);
  for (size_t i = 0; i < ARM_DOF; ++i) {
    // 激活后回零：使用当前 Kp/Kd，目标 q=0, qdot=0, tau_ff=0。
    arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  if (hand_) {
    // 夹爪回到闭合零位（与当前项目的默认定义一致）。
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
}

double OpenArm_v10HW::joint_to_motor_radians(double joint_value) {
  // 线性映射：joint [0, 0.044] -> motor [0, -1.0472]
  // 注：这是当前近似模型，适合现有硬件；后续可按机构实测曲线替换。
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;
}

double OpenArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // 逆映射：motor [0, -1.0472] -> joint [0, 0.044]
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians / GRIPPER_MOTOR_1_RADIANS);
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
