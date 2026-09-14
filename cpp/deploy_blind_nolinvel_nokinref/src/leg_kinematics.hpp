#pragma once
/// 文件：leg_kinematics.hpp
/// Go2 腿部正运动学和解析雅可比。
/// 腿编号采用仿真顺序：0=FL，1=FR，2=RL，3=RR。

#include <Eigen/Core>
#include <cassert>
#include <cmath>

namespace jave {

// URDF 几何常量。

inline constexpr double THIGH_LEN = 0.213;
inline constexpr double CALF_LEN = 0.213;

/// 髋关节相对机身质心的偏移，机身坐标系为 x 前、y 左、z 上。
/// 行顺序：FL、FR、RL、RR。
inline const Eigen::Matrix<double, 4, 3> &hip_offsets() {
  static const Eigen::Matrix<double, 4, 3> H =
      (Eigen::Matrix<double, 4, 3>() << +0.1934, +0.0465, 0.0, // 左前腿
       +0.1934, -0.0465, 0.0,                                  // 右前腿
       -0.1934, +0.0465, 0.0,                                  // 左后腿
       -0.1934, -0.0465, 0.0                                   // 右后腿
       )
          .finished();
  return H;
}

/// 髋关节到大腿关节的横向偏移，正值表示向外。
inline constexpr double HIP_LENGTHS[4] = {+0.0955, -0.0955, +0.0955, -0.0955};

// 正运动学。

/// 单条腿的正运动学。
/// 参数 leg：腿编号，0=FL，1=FR，2=RL，3=RR。
/// 参数 q：[髋外展，大腿，小腿] 三个关节角。
/// 返回值：机身坐标系下的足端位置，维度为 3。
inline Eigen::Vector3d leg_fk(int leg, const Eigen::Vector3d &q) {
  assert(leg >= 0 && leg < 4);
  const double q0 = q(0), q1 = q(1), q2 = q(2);
  const double d = HIP_LENGTHS[leg];

  const double dx = -THIGH_LEN * std::sin(q1) - CALF_LEN * std::sin(q1 + q2);
  const double dz = -THIGH_LEN * std::cos(q1) - CALF_LEN * std::cos(q1 + q2);

  const double c0 = std::cos(q0), s0 = std::sin(q0);
  Eigen::Vector3d foot_rel_hip;
  foot_rel_hip << dx, d * c0 - dz * s0, d * s0 + dz * c0;

  return hip_offsets().row(leg).transpose() + foot_rel_hip;
}

// 解析雅可比。

/// 单条腿足端位置对关节角 q 的解析雅可比。
/// 返回值：3x3 矩阵，列对应 [d/dq0, d/dq1, d/dq2]。
inline Eigen::Matrix3d leg_jacobian(int leg, const Eigen::Vector3d &q) {
  assert(leg >= 0 && leg < 4);
  const double q0 = q(0), q1 = q(1), q2 = q(2);
  const double d = HIP_LENGTHS[leg];
  const double c0 = std::cos(q0), s0 = std::sin(q0);

  const double dz = -THIGH_LEN * std::cos(q1) - CALF_LEN * std::cos(q1 + q2);

  const double ddx_dq1 =
      -THIGH_LEN * std::cos(q1) - CALF_LEN * std::cos(q1 + q2);
  const double ddz_dq1 =
      THIGH_LEN * std::sin(q1) + CALF_LEN * std::sin(q1 + q2);

  const double ddx_dq2 = -CALF_LEN * std::cos(q1 + q2);
  const double ddz_dq2 = CALF_LEN * std::sin(q1 + q2);

  Eigen::Matrix3d J;
  // 对 q0 求导。
  J(0, 0) = 0.0;
  J(1, 0) = -d * s0 - dz * c0;
  J(2, 0) = d * c0 - dz * s0;
  // 对 q1 求导。
  J(0, 1) = ddx_dq1;
  J(1, 1) = -ddz_dq1 * s0;
  J(2, 1) = ddz_dq1 * c0;
  // 对 q2 求导。
  J(0, 2) = ddx_dq2;
  J(1, 2) = -ddz_dq2 * s0;
  J(2, 2) = ddz_dq2 * c0;

  return J;
}

} // 命名空间 jave
