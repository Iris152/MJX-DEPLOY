/// 文件：policy.cpp
#include "policy.hpp"
#include "math_utils.hpp"
#include "npz_reader.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace jave {

// 辅助函数。

namespace {

Eigen::VectorXd load_vec(const npz::NpzFile &npz, const std::string &key) {
  auto it = npz.find(key);
  if (it == npz.end())
    throw std::runtime_error("Missing key in .npz: " + key);
  return it->second.to_vector();
}

Eigen::MatrixXd load_mat(const npz::NpzFile &npz, const std::string &key) {
  auto it = npz.find(key);
  if (it == npz.end())
    throw std::runtime_error("Missing key in .npz: " + key);
  return it->second.to_matrix();
}

double load_scalar(const npz::NpzFile &npz, const std::string &key) {
  auto it = npz.find(key);
  if (it == npz.end())
    throw std::runtime_error("Missing key in .npz: " + key);
  return it->second.to_scalar();
}

Eigen::Vector2d load_range_or_default(const npz::NpzFile &npz,
                                      const std::string &key,
                                      const Eigen::Vector2d &fallback) {
  if (!npz::has_key(npz, key))
    return fallback;
  Eigen::VectorXd v = load_vec(npz, key);
  if (v.size() != 2)
    throw std::runtime_error("Expected 2 values in .npz key: " + key);
  return Eigen::Vector2d(v(0), v(1));
}

} // 匿名命名空间

// 构造函数。

NumpyPolicy::NumpyPolicy(const std::string &npz_path) {
  std::cout << "Loading policy: " << npz_path << "\n";
  auto npz = npz::load_npz(npz_path);

  // 观测归一化统计量。
  norm_mean = load_vec(npz, "norm_mean");
  norm_var = load_vec(npz, "norm_var");

  // 网络层权重。
  const int n_hidden = static_cast<int>(load_scalar(npz, "n_hidden"));
  layers_.resize(n_hidden);
  for (int i = 0; i < n_hidden; ++i) {
    auto &L = layers_[i];
    L.kernel = load_mat(npz, "dense_" + std::to_string(i) + "_kernel");
    L.bias = load_vec(npz, "dense_" + std::to_string(i) + "_bias");
    L.ln_scale = load_vec(npz, "ln_" + std::to_string(i) + "_scale");
    L.ln_bias = load_vec(npz, "ln_" + std::to_string(i) + "_bias");
  }
  out_kernel_ = load_mat(npz, "dense_" + std::to_string(n_hidden) + "_kernel");
  out_bias_ = load_vec(npz, "dense_" + std::to_string(n_hidden) + "_bias");

  // 打印网络结构。
  const int in_dim = static_cast<int>(layers_[0].kernel.rows());
  const int out_dim = static_cast<int>(out_kernel_.cols());
  std::cout << "  Actor: " << in_dim;
  for (int i = 0; i < n_hidden; ++i)
    std::cout << " -> " << layers_[i].kernel.cols();
  std::cout << " -> " << out_dim << "\n";
  std::cout << "  Hidden layers: " << n_hidden
            << " (each Dense + LayerNorm + ELU)\n";

  // 环境配置。
  default_joints = load_vec(npz, "default_joints");
  action_scale = load_vec(npz, "action_scale");
  dt = load_scalar(npz, "dt");
  cmd_vel_x_range =
      load_range_or_default(npz, "cmd_vel_x_range", cmd_vel_x_range);
  cmd_vel_y_range =
      load_range_or_default(npz, "cmd_vel_y_range", cmd_vel_y_range);
  cmd_yaw_rate_range =
      load_range_or_default(npz, "cmd_yaw_rate_range", cmd_yaw_rate_range);
  if (!npz::has_key(npz, "actor_history_len") ||
      !npz::has_key(npz, "actor_frame_obs_dim"))
    throw std::runtime_error(
        "Policy predates the reduced actor observation history format");
  actor_history_len = static_cast<int>(load_scalar(npz, "actor_history_len"));
  actor_frame_obs_dim =
      static_cast<int>(load_scalar(npz, "actor_frame_obs_dim"));
  if (in_dim != actor_history_len * actor_frame_obs_dim)
    throw std::runtime_error(
        "Actor input dimension does not match observation history metadata");
  if (norm_mean.size() != actor_frame_obs_dim)
    throw std::runtime_error(
        "Normalizer dimension does not match actor frame size");

  std::cout << "  Deployment obs: " << actor_history_len << "x"
            << actor_frame_obs_dim << "D\n";

  std::cout << "  dt=" << dt << "s\n";
  std::cout << "  Command ranges: vx=[" << cmd_vel_x_range(0) << ", "
            << cmd_vel_x_range(1) << "], vy=[" << cmd_vel_y_range(0)
            << ", " << cmd_vel_y_range(1) << "], yaw=["
            << cmd_yaw_rate_range(0) << ", " << cmd_yaw_rate_range(1)
            << "]\n";

  // 电机增益。
  if (npz::has_key(npz, "actuator_kp")) {
    training_kp = load_vec(npz, "actuator_kp")(0);
    double kd_act = npz::has_key(npz, "actuator_kd")
                        ? load_vec(npz, "actuator_kd")(0)
                        : 0.0;
    double kd_joint = npz::has_key(npz, "joint_damping")
                          ? load_vec(npz, "joint_damping")(0)
                          : 0.0;
    training_kd = kd_act + kd_joint;
    std::cout << "  Training gains: kp=" << training_kp
              << ", kd=" << training_kd << " (actuator=" << kd_act
              << " + joint_damping=" << kd_joint << ")\n";
  } else {
    std::cout << "  WARNING: No actuator gains in .npz!\n";
  }
}

// 前向推理。

Eigen::VectorXd NumpyPolicy::operator()(const Eigen::VectorXd &obs) const {
  if (obs.size() != actor_history_len * actor_frame_obs_dim)
    throw std::runtime_error("Unexpected actor observation dimension");
  Eigen::VectorXd x(obs.size());
  for (int i = 0; i < actor_history_len; ++i) {
    x.segment(i * actor_frame_obs_dim, actor_frame_obs_dim) =
        (obs.segment(i * actor_frame_obs_dim, actor_frame_obs_dim) - norm_mean)
            .array() /
        (norm_var.array() + NORM_EPS).sqrt();
  }
  return forward_raw(x);
}

Eigen::VectorXd NumpyPolicy::forward_raw(const Eigen::VectorXd &x_in) const {
  Eigen::VectorXd x = x_in;

  // 隐藏层：全连接层 --> 层归一化 --> ELU 激活。
  for (const auto &L : layers_) {
    x = (x.transpose() * L.kernel).transpose() + L.bias; // 全连接层。
    x = layer_norm(x, L.ln_scale, L.ln_bias);            // 层归一化。
    x = elu(x);                                          // ELU 激活。
  }

  // 输出层：全连接层 --> tanh。
  x = (x.transpose() * out_kernel_).transpose() + out_bias_;
  return tanh_vec(x);
}

// 目标关节位置。

Eigen::VectorXd
NumpyPolicy::get_target_joints(const Eigen::VectorXd &action) const {
  Eigen::VectorXd a = action.cwiseMax(-1.0).cwiseMin(1.0);
  // action_scale 可以是标量，也可以是逐关节 12 维向量。
  if (action_scale.size() == 1) {
    return default_joints + a * action_scale(0);
  }
  return default_joints + a.cwiseProduct(action_scale);
}

} // 命名空间 jave
