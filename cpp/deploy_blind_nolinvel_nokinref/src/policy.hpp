#pragma once
/// 文件：policy.hpp
/// 加载已导出的 .npz 权重，并执行策略网络多层感知机推理。
/// 网络结构：[全连接层 + 层归一化 + ELU 激活] x N  -->  全连接层 + tanh。

#include <Eigen/Core>
#include <limits>
#include <string>
#include <vector>

namespace jave {

/// 一个隐藏层，由全连接层、层归一化和 ELU 激活组成。
struct HiddenLayer {
  Eigen::MatrixXd kernel;   // 输入维度 x 输出维度。
  Eigen::VectorXd bias;     // 输出维度。
  Eigen::VectorXd ln_scale; // 输出维度。
  Eigen::VectorXd ln_bias;  // 输出维度。
};

class NumpyPolicy {
public:
  explicit NumpyPolicy(const std::string &npz_path);

  /// 执行策略网络前向推理，将观测映射到 [-1, 1]^12 动作。
  Eigen::VectorXd operator()(const Eigen::VectorXd &obs) const;

  /// 对已经归一化的输入执行 MLP 前向推理，跳过观测归一化。
  Eigen::VectorXd forward_raw(const Eigen::VectorXd &x_norm) const;

  /// 根据动作计算 PD 目标关节位置。
  Eigen::VectorXd get_target_joints(const Eigen::VectorXd &action) const;

  // 构造后可读取的公开配置。

  // 观测归一化统计量。
  Eigen::VectorXd norm_mean;
  Eigen::VectorXd norm_var;
  int actor_history_len = 1;
  int actor_frame_obs_dim = 0;
  static constexpr double NORM_EPS = 1e-4;

  // 环境配置。
  Eigen::VectorXd default_joints; // 12 维默认关节角。
  Eigen::VectorXd action_scale;   // 标量或逐关节 12 维缩放。
  // 速度指令限幅范围；只有旧导出文件缺少 cmd_vel_* 元数据时才使用默认值。
  Eigen::Vector2d cmd_vel_x_range = Eigen::Vector2d(-1.5, 1.5);
  Eigen::Vector2d cmd_vel_y_range = Eigen::Vector2d(-1.0, 1.0);
  Eigen::Vector2d cmd_yaw_rate_range = Eigen::Vector2d(-1.5, 1.5);
  double dt = 0.0;
  // 训练时的电机增益；不可用时为 NaN。
  double training_kp = std::numeric_limits<double>::quiet_NaN();
  double training_kd = std::numeric_limits<double>::quiet_NaN();

private:
  std::vector<HiddenLayer> layers_;
  Eigen::MatrixXd out_kernel_; // 隐藏层维度 x 12。
  Eigen::VectorXd out_bias_;   // 12 维输出偏置。
};

} // 命名空间 jave
