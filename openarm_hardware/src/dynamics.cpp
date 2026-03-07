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

#include "openarm_hardware/dynamics.hpp"

#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

namespace {

const char* joint_type_to_string(KDL::Joint::JointType type) {
  switch (type) {
    case KDL::Joint::None:
      return "None";
    case KDL::Joint::RotAxis:
      return "RotAxis";
    case KDL::Joint::RotX:
      return "RotX";
    case KDL::Joint::RotY:
      return "RotY";
    case KDL::Joint::RotZ:
      return "RotZ";
    case KDL::Joint::TransAxis:
      return "TransAxis";
    case KDL::Joint::TransX:
      return "TransX";
    case KDL::Joint::TransY:
      return "TransY";
    case KDL::Joint::TransZ:
      return "TransZ";
    default:
      return "Unknown";
  }
}

}  // namespace

Dynamics::Dynamics(const std::string& urdf_or_xml, const std::string& root_link,
                   const std::string& tip_link, bool is_urdf_xml)
    : urdf_or_xml_(urdf_or_xml),
      root_link_(root_link),
      tip_link_(tip_link),
      is_urdf_xml_(is_urdf_xml) {}

bool Dynamics::Init() {
  // ===========================================================================
  // Step 1) 获取 URDF 文本
  // ===========================================================================
  std::string urdf_content;

  // 支持两种输入：URDF 文件路径 / URDF XML 字符串。
  if (is_urdf_xml_) {
    urdf_content = urdf_or_xml_;
  } else {
    std::ifstream file(urdf_or_xml_);
    if (!file.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Failed to open URDF file: %s", urdf_or_xml_.c_str());
      return false;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    urdf_content = buffer.str();
  }

  // ===========================================================================
  // Step 2) 解析 URDF -> urdf::Model
  // ===========================================================================
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_content)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"), "Failed to parse URDF");
    return false;
  }

  // ===========================================================================
  // Step 3) urdf model -> KDL Tree（完整机器人）
  // ===========================================================================
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to extract KDL tree from URDF");
    return false;
  }

  // ===========================================================================
  // Step 4) 从 Tree 中抽取 root->tip 的 KDL Chain（控制所需关节链）
  // ===========================================================================
  // 从完整机器人树中提取 root_link -> tip_link 对应的运动链。
  if (!kdl_tree.getChain(root_link_, tip_link_, kdl_chain_)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to get KDL chain from '%s' to '%s'", root_link_.c_str(),
                 tip_link_.c_str());
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "KDL chain built: root=%s tip=%s segments=%u joints=%u",
              root_link_.c_str(), tip_link_.c_str(),
              kdl_chain_.getNrOfSegments(), kdl_chain_.getNrOfJoints());

  const auto& segments = kdl_chain_.segments;
  for (size_t i = 0; i < segments.size(); ++i) {
    const auto& segment = segments[i];
    const auto& joint = segment.getJoint();
    const auto axis = joint.JointAxis();
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "KDL seg[%zu]: seg=%s joint=%s type=%s axis=(%.6f, %.6f, %.6f)",
                i, segment.getName().c_str(), joint.getName().c_str(),
                joint_type_to_string(joint.getType()),
                axis.x(), axis.y(), axis.z());
  }

  // ===========================================================================
  // Step 5) 初始化中间缓存与求解器
  // ===========================================================================
  // 分配缓存，避免控制周期中重复分配内存。
  gravity_forces_.resize(kdl_chain_.getNrOfJoints());
  coriolis_forces_.resize(kdl_chain_.getNrOfJoints());
  
  gravity_forces_.data.setZero();
  coriolis_forces_.data.setZero();

  // 重力方向默认采用世界坐标 -Z（m/s^2）。
  solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Dynamics initialized, chain joints=%u", kdl_chain_.getNrOfJoints());
  return true;
}

void Dynamics::GetGravity(const double* joint_position, double* gravity) {
  // 输入 q（关节角）
  // 将裸数组转为 KDL::JntArray，调用 KDL 求解 τ_g = G(q)。
  KDL::JntArray q(kdl_chain_.getNrOfJoints());
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
    q(i) = joint_position[i];
  }

  solver_->JntToGravity(q, gravity_forces_);

  // 输出 τ_g
  // 回写到调用方缓冲区。
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
    gravity[i] = gravity_forces_(i);
  }
}

void Dynamics::GetCoriolis(const double* joint_position,
                           const double* joint_velocity, double* coriolis) {
  // 输入 q, qdot（关节角、关节速度）
  // 调用 KDL 求解 τ_c = C(q, qdot) * qdot。
  KDL::JntArray q(kdl_chain_.getNrOfJoints());
  KDL::JntArray qd(kdl_chain_.getNrOfJoints());

  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
    q(i) = joint_position[i];
    qd(i) = joint_velocity[i];
  }

  solver_->JntToCoriolis(q, qd, coriolis_forces_);

  // 输出 τ_c
  // 回写到调用方缓冲区。
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
    coriolis[i] = coriolis_forces_(i);
  }
}

}  // namespace openarm_hardware
