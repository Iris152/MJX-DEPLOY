#pragma once
/// 文件：math_utils.hpp
/// Go2 部署使用的四元数和旋转辅助函数。
/// 所有四元数均采用 (w, x, y, z) 约定，这也是 MuJoCo/Eigen 默认顺序。

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

namespace jave {

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;

// 四元数辅助函数。

/// 四元数共轭；对于单位四元数等价于逆。
inline Vec4 quat_inv(const Vec4 &q) { return {q(0), -q(1), -q(2), -q(3)}; }

/// 使用四元数 q 旋转向量 v，这里采用哈密顿乘法的简化形式。
inline Vec3 quat_rotate(const Vec3 &v, const Vec4 &q) {
  const double w = q(0);
  const Vec3 u = q.tail<3>();
  return 2.0 * u.dot(v) * u + (w * w - u.dot(u)) * v + 2.0 * w * u.cross(v);
}

/// 将 (w,x,y,z) 四元数转换为 3x3 旋转矩阵。
inline Mat3 quat_to_rotmat(const Vec4 &q) {
  const double w = q(0), x = q(1), y = q(2), z = q(3);
  Mat3 R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
      2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
      2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);
  return R;
}

// 激活函数。

/// 逐元素 ELU，alpha=1。
template <int N>
inline Eigen::Matrix<double, N, 1> elu(const Eigen::Matrix<double, N, 1> &x) {
  return x.array().max(0.0) + (x.array().min(0.0).exp() - 1.0).min(0.0);
}

/// 动态尺寸向量的 ELU。
inline Eigen::VectorXd elu(const Eigen::VectorXd &x) {
  return x.array().max(0.0) + (x.array().min(0.0).exp() - 1.0).min(0.0);
}

/// 逐元素 tanh，为了接口一致性封装 Eigen 实现。
inline Eigen::VectorXd tanh_vec(const Eigen::VectorXd &x) {
  return x.array().tanh();
}

// 层归一化。

/// 层归一化：scale * (x - mean) / sqrt(var + eps) + bias。
inline Eigen::VectorXd layer_norm(const Eigen::VectorXd &x,
                                  const Eigen::VectorXd &scale,
                                  const Eigen::VectorXd &bias,
                                  double eps = 1e-6) {
  const double mean = x.mean();
  const double var = (x.array() - mean).square().mean();
  return scale.array() * (x.array() - mean) / std::sqrt(var + eps) +
         bias.array();
}

} // 命名空间 jave
