#pragma once
/// 文件：controller.hpp
/// Go2 部署控制器，负责状态机、DDS 通信和电机指令。
///
/// 编译需要 unitree_sdk2 C++ SDK。
/// 参考：https://github.com/unitreerobotics/unitree_sdk2

#include "policy.hpp"

#include <Eigen/Core>
#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace jave {

// 常量定义。

inline constexpr int NUM_MOTORS = 12;
inline constexpr int NUM_MOTOR_SLOTS = 20;

/// 关节顺序转换：仿真顺序 <--> 实机硬件顺序。
/// Go2 上两个方向刚好使用同一个排列。
inline constexpr int SIM_TO_HW[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
inline constexpr int HW_TO_SIM[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

/// 硬件顺序下的蹲伏姿态，用于坐下流程。
inline const Eigen::Matrix<double, 12, 1> &crouch_pos_hw() {
  static const Eigen::Matrix<double, 12, 1> v =
      (Eigen::Matrix<double, 12, 1>() << -0.35, 1.36, -2.65, // 右前腿
       0.35, 1.36, -2.65,                                    // 左前腿
       -0.50, 1.36, -2.65,                                   // 右后腿
       0.50, 1.36, -2.65                                     // 左后腿
       )
          .finished();
  return v;
}

// 关节顺序转换辅助函数。

/// 将 12 维向量从仿真顺序转换为硬件顺序。
inline Eigen::Matrix<double, 12, 1>
sim_to_hw(const Eigen::Matrix<double, 12, 1> &v) {
  Eigen::Matrix<double, 12, 1> out;
  for (int i = 0; i < 12; ++i)
    out(SIM_TO_HW[i]) = v(i);
  return out;
}

/// 动态 VectorXd 的重载，例如 default_joints。
inline Eigen::Matrix<double, 12, 1> sim_to_hw(const Eigen::VectorXd &v) {
  Eigen::Matrix<double, 12, 1> out;
  for (int i = 0; i < 12; ++i)
    out(SIM_TO_HW[i]) = v(i);
  return out;
}

/// 将 12 维向量从硬件顺序转换为仿真顺序。
inline Eigen::Matrix<double, 12, 1>
hw_to_sim(const Eigen::Matrix<double, 12, 1> &v) {
  Eigen::Matrix<double, 12, 1> out;
  for (int i = 0; i < 12; ++i)
    out(HW_TO_SIM[i]) = v(i);
  return out;
}

// 控制状态机。

enum class State {
  IDLE,
  STANDUP,
  READY,
  WALKING,
  SITDOWN,
  ESTOP,
};

const char *state_name(State s);

enum class CommandSource {
  TERMINAL,
  WIRELESS,
  ROS2,
};

// 控制器主体。

class Go2Deploy {
public:
  Go2Deploy(std::shared_ptr<NumpyPolicy> policy,
            const std::string &interface = "lo", int domain_id = 0,
            CommandSource command_source = CommandSource::TERMINAL,
            const std::string &cmd_topic = "/velocity_command");
  ~Go2Deploy();

  /// 主阻塞循环，键盘输入在调用线程中处理。
  void run();

  // PD 增益，可在 run() 前覆盖；默认从策略文件自动加载。
  double kp = 35.0;
  double kd = 0.5;

  // 起立/坐下使用平滑增益；宇树示例的高增益线性插值在仿真中容易振动。
  static constexpr double STANDUP_KP = 50.0;
  static constexpr double STANDUP_KD = 3.5;
  static constexpr double STANDUP_KP_START = 20.0;
  static constexpr double STANDUP_TANH_SCALE = 1.2;
  static constexpr double SAFETY_TILT_MAX = 1.05;

private:
  // SDK 初始化。
  void init_sdk(const std::string &interface, int domain_id);
  void release_sport_mode();

  // 500 赫兹命令循环，由 CreateRecurrentThreadEx 调用。
  void LowCmdWrite();

  // SDK 回调。
  void LowStateHandler(const void *message);
  void update_wireless_command(const uint8_t *data, std::size_t size);

  // 电机写入和发布辅助函数。
  void set_motor(int i, float q, float kp_val, float dq, float kd_val,
                 float tau);
  void publish_cmd();

  // 状态处理函数，由 LowCmdWrite 调用。
  void handle_idle();
  void handle_standup();
  void handle_ready();
  void handle_walking();
  void handle_sitdown();
  void handle_estop();

  // 观测构造与安全检查。
  Eigen::VectorXd build_obs();
  bool check_safety();

  // 状态切换。
  void transition(State to);

  // 键盘输入处理。
  void process_key(const std::string &key);
  void set_cmd(double vx, double vy, double yaw_rate);
  Eigen::Vector3d get_cmd() const;

  // 成员变量。
  std::shared_ptr<NumpyPolicy> policy_;
  std::string interface_;
  CommandSource command_source_;

  // 控制时序。
  double dt_cmd_ = 0.002;      // 500 赫兹命令频率。
  int policy_decimation_ = 10; // 50 赫兹策略频率。

  // 状态机变量：键盘线程写入，命令线程读取。
  std::atomic<State> state_{State::IDLE};
  mutable std::mutex cmd_mutex_;
  Eigen::Vector3d cmd_ = Eigen::Vector3d::Zero(); // [vx, vy, 偏航角速度]

  // 行走状态。
  Eigen::Matrix<double, 12, 1> last_action_ =
      Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> walking_target_hw_ =
      Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::VectorXd actor_obs_history_;
  int step_count_ = 0;

  // 起立状态。
  double standup_time_ = 0.0;
  Eigen::Matrix<double, 12, 1> standup_start_pos_ =
      Eigen::Matrix<double, 12, 1>::Zero();
  bool standup_first_run_ = true;

  // 坐下状态。
  double sitdown_time_ = 0.0;
  Eigen::Matrix<double, 12, 1> sitdown_start_pos_ =
      Eigen::Matrix<double, 12, 1>::Zero();
  std::atomic<bool> sitdown_done_{false};

  // 急停状态。
  Eigen::Matrix<double, 12, 1> estop_hold_pos_ =
      Eigen::Matrix<double, 12, 1>::Zero();

  // 运动计数器，每次 LowCmdWrite 调用都会递增，即 500 赫兹。
  int motiontime_ = 0;

  // 传感器数据由订阅回调写入，并由 LowCmdWrite 读取。
  // SDK 回调可能运行在不同线程，因此使用互斥锁保护。
  mutable std::mutex sensor_mutex_;
  std::atomic<bool> state_received_{false};

  // 传感器缓存，访问时需要持有 sensor_mutex_。
  Eigen::Matrix<double, 12, 1> hw_pos_ = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> hw_vel_ = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Vector4d imu_quat_ = Eigen::Vector4d(1, 0, 0, 0);
  Eigen::Vector3d imu_gyro_ = Eigen::Vector3d::Zero();

  // SDK 句柄，仅在 controller.cpp 内部使用。
  // 这里使用前向声明，避免在头文件中引入 SDK 头文件。
  struct SdkHandles;
  std::unique_ptr<SdkHandles> sdk_;
};

} // 命名空间 jave
