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

#pragma once

#include <memory>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace openarm_hardware {

/**
 * @brief 轻量动力学封装（KDL）
 *
 * 设计目的：
 * - 只提供 hardware 周期中最常用、最稳定的两个补偿量；
 * - 保持接口简单，便于维护和调试；
 * - 不把复杂控制策略耦合到本类中。
 *
 * 当前暴露：
 * - GetGravity : 计算 τ_g = G(q)
 * - GetCoriolis: 计算 τ_c = C(q, qdot) * qdot
 *
 * 典型使用：
 * - 在 write() 周期中，根据当前状态(q, qdot)获取补偿力矩，
 *   再与上层给定的 tau 指令叠加后下发到电机。
 */
class Dynamics {
 public:
  // urdf_or_xml:
  // - is_urdf_xml=false: 传入 URDF 文件路径
  // - is_urdf_xml=true : 传入 URDF XML 字符串（推荐用于 ros2_control info.original_xml）
  Dynamics(const std::string& urdf_or_xml, const std::string& root_link,
           const std::string& tip_link, bool is_urdf_xml = false);

  // 解析 URDF -> 构建 KDL tree/chain -> 初始化动力学求解器。
  // 返回 false 通常表示：
  // - URDF 解析失败；
  // - root_link / tip_link 不存在；
  // - 无法从树中提取目标运动链。
  bool Init();

  // 输入当前关节角，输出每个关节的重力补偿力矩（Nm）。
  // 输入/输出数组长度应等于链路关节数（通常是7）。
  void GetGravity(const double* joint_position, double* gravity);

  // 输入关节角和关节速度，输出科里奥利/离心补偿力矩（Nm）。
  // 在慢速运动中该项通常较小，高速跟踪时价值更明显。
  void GetCoriolis(const double* joint_position, const double* joint_velocity,
                   double* coriolis);

 private:
  std::string urdf_or_xml_;
  std::string root_link_;
  std::string tip_link_;
  bool is_urdf_xml_ = false;

  KDL::Chain kdl_chain_;
  KDL::JntArray gravity_forces_;
  KDL::JntArray coriolis_forces_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
};

}  // namespace openarm_hardware
