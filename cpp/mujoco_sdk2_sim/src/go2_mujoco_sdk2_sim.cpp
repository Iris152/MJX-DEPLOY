/// 文件：go2_mujoco_sdk2_sim.cpp
/// 使用 MuJoCo 模拟 Go2 低层硬件，并通过 unitree_sdk2 发布/订阅 LowState/LowCmd。

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelPublisher;
using unitree::robot::ChannelPublisherPtr;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;

namespace {

using LowCmd = unitree_go::msg::dds_::LowCmd_;
using LowState = unitree_go::msg::dds_::LowState_;

constexpr int kMotorCount = 12;
constexpr int kMotorSlotCount = 20;
constexpr double kPosStopF = 2.146e9;
constexpr double kVelStopF = 16000.0;

constexpr std::array<int, kMotorCount> kPolicyToUnitree = {
    3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
constexpr std::array<int, kMotorCount> kUnitreeToPolicy = {
    3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
constexpr std::array<int, 4> kPolicyFootToUnitree = {1, 0, 3, 2};

constexpr std::array<const char *, kMotorCount> kJointNames = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

constexpr std::array<const char *, kMotorCount> kActuatorNames = {
    "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf"};

constexpr std::array<const char *, 4> kFootGeomNames = {"FL", "FR", "RL", "RR"};

enum class ControlMode { Auto, PositionServo, PdTorque };
enum class InitialPose { Home, Crouch, Prone };
enum class IdleTarget { Initial, Home };

struct Args {
  std::string scene = "models/go2/scene_mjx.xml";
  std::string interface = "lo";
  int domain_id = 1;
  double sim_dt = 0.002;
  double viewer_dt = 0.02;
  double cmd_timeout = 0.25;
  double idle_kp = 80.0;
  double idle_kd = 6.0;
  double max_time = 0.0;
  double status_period = 1.0;
  bool headless = false;
  ControlMode control_mode = ControlMode::Auto;
  InitialPose initial_pose = InitialPose::Prone;
  IdleTarget idle_target = IdleTarget::Initial;
};

struct CommandSnapshot {
  bool active = false;
  std::array<double, kMotorCount> q{};
  std::array<double, kMotorCount> dq{};
  std::array<double, kMotorCount> kp{};
  std::array<double, kMotorCount> kd{};
  std::array<double, kMotorCount> tau{};
  double received_at = 0.0;
};

struct ControlSnapshot {
  CommandSnapshot cmd;
  std::array<double, kMotorCount> hold_q{};
  std::array<double, kMotorCount> hold_dq{};
  std::array<double, kMotorCount> hold_kp{};
  std::array<double, kMotorCount> hold_kd{};
  bool hold_from_command = false;
};

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

double now_seconds() {
  using clock = std::chrono::steady_clock;
  static const auto start = clock::now();
  return std::chrono::duration<double>(clock::now() - start).count();
}

void print_usage(const char *prog) {
  std::cout
      << "用法：" << prog << " [选项]\n\n"
      << "选项：\n"
      << "  --scene <path>             MuJoCo XML，默认 models/go2/scene_mjx.xml\n"
      << "  --interface <iface>        DDS 网卡，仿真默认 lo\n"
      << "  --domain-id <id>           DDS 域编号，仿真默认 1\n"
      << "  --sim-dt <sec>             仿真步长，默认 0.002\n"
      << "  --viewer-dt <sec>          界面刷新间隔，默认 0.02\n"
      << "  --cmd-timeout <sec>        LowCmd 超时时间，默认 0.25\n"
      << "  --idle-kp <value>          无有效命令时保持姿态的 kp\n"
      << "  --idle-kd <value>          无有效命令时保持姿态的 kd\n"
      << "  --initial-pose <pose>      home、crouch 或 prone，默认 prone\n"
      << "  --idle-target <target>     initial 或 home，默认 initial\n"
      << "  --control-mode <mode>      auto、position_servo 或 pd_torque\n"
      << "  --headless                 不打开 MuJoCo 界面\n"
      << "  --max-time <sec>           最长运行时间，0 表示一直运行\n"
      << "  --status-period <sec>      状态打印间隔，默认 1\n"
      << "  -h, --help                 显示帮助\n";
}

double parse_double(const std::string &value, const std::string &name) {
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error("参数 " + name + " 需要浮点数：" + value);
  }
}

int parse_int(const std::string &value, const std::string &name) {
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    throw std::runtime_error("参数 " + name + " 需要整数：" + value);
  }
}

ControlMode parse_control_mode(const std::string &value) {
  if (value == "auto")
    return ControlMode::Auto;
  if (value == "position_servo")
    return ControlMode::PositionServo;
  if (value == "pd_torque")
    return ControlMode::PdTorque;
  throw std::runtime_error("未知 control-mode：" + value);
}

InitialPose parse_initial_pose(const std::string &value) {
  if (value == "home")
    return InitialPose::Home;
  if (value == "crouch")
    return InitialPose::Crouch;
  if (value == "prone")
    return InitialPose::Prone;
  throw std::runtime_error("未知 initial-pose：" + value);
}

IdleTarget parse_idle_target(const std::string &value) {
  if (value == "initial")
    return IdleTarget::Initial;
  if (value == "home")
    return IdleTarget::Home;
  throw std::runtime_error("未知 idle-target：" + value);
}

Args parse_args(int argc, char **argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto require_value = [&](const std::string &name) -> std::string {
      if (i + 1 >= argc)
        throw std::runtime_error("缺少参数值：" + name);
      return argv[++i];
    };

    if (arg == "--scene")
      args.scene = require_value(arg);
    else if (arg == "--interface" || arg == "--network")
      args.interface = require_value(arg);
    else if (arg == "--domain-id" || arg == "--domain_id")
      args.domain_id = parse_int(require_value(arg), arg);
    else if (arg == "--sim-dt" || arg == "--sim_dt")
      args.sim_dt = parse_double(require_value(arg), arg);
    else if (arg == "--viewer-dt" || arg == "--viewer_dt")
      args.viewer_dt = parse_double(require_value(arg), arg);
    else if (arg == "--cmd-timeout" || arg == "--cmd_timeout")
      args.cmd_timeout = parse_double(require_value(arg), arg);
    else if (arg == "--idle-kp" || arg == "--idle_kp")
      args.idle_kp = parse_double(require_value(arg), arg);
    else if (arg == "--idle-kd" || arg == "--idle_kd")
      args.idle_kd = parse_double(require_value(arg), arg);
    else if (arg == "--max-time" || arg == "--max_time")
      args.max_time = parse_double(require_value(arg), arg);
    else if (arg == "--status-period" || arg == "--status_period")
      args.status_period = parse_double(require_value(arg), arg);
    else if (arg == "--initial-pose" || arg == "--initial_pose")
      args.initial_pose = parse_initial_pose(require_value(arg));
    else if (arg == "--idle-target" || arg == "--idle_target")
      args.idle_target = parse_idle_target(require_value(arg));
    else if (arg == "--control-mode" || arg == "--control_mode")
      args.control_mode = parse_control_mode(require_value(arg));
    else if (arg == "--headless")
      args.headless = true;
    else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("未知参数：" + arg);
    }
  }
  return args;
}

const char *control_mode_name(ControlMode mode) {
  switch (mode) {
  case ControlMode::Auto:
    return "auto";
  case ControlMode::PositionServo:
    return "position_servo";
  case ControlMode::PdTorque:
    return "pd_torque";
  }
  return "unknown";
}

std::array<double, kMotorCount> make_pose(InitialPose pose,
                                          const std::array<double, kMotorCount> &home) {
  if (pose == InitialPose::Home)
    return home;
  std::array<double, kMotorCount> out{};
  const double thigh = pose == InitialPose::Crouch ? 1.25 : 1.45;
  const double calf = pose == InitialPose::Crouch ? -2.45 : -2.60;
  for (int leg = 0; leg < 4; ++leg) {
    out[3 * leg + 0] = 0.0;
    out[3 * leg + 1] = thigh;
    out[3 * leg + 2] = calf;
  }
  return out;
}

std::array<double, kMotorCount>
unitree_to_policy(const std::array<double, kMotorCount> &unitree) {
  std::array<double, kMotorCount> policy{};
  for (int i = 0; i < kMotorCount; ++i)
    policy[kUnitreeToPolicy[i]] = unitree[i];
  return policy;
}

std::array<double, kMotorCount>
policy_to_unitree(const std::array<double, kMotorCount> &policy) {
  std::array<double, kMotorCount> unitree{};
  for (int i = 0; i < kMotorCount; ++i)
    unitree[kPolicyToUnitree[i]] = policy[i];
  return unitree;
}

double clamp_to_range(double value, const mjModel *model, int actuator_id) {
  const double lo = model->actuator_ctrlrange[2 * actuator_id + 0];
  const double hi = model->actuator_ctrlrange[2 * actuator_id + 1];
  return std::clamp(value, lo, hi);
}

std::array<double, 9> quat_to_rotmat(const double *q) {
  double w = q[0], x = q[1], y = q[2], z = q[3];
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  if (n < 1e-9) {
    w = 1.0;
    x = y = z = 0.0;
  } else {
    w /= n;
    x /= n;
    y /= n;
    z /= n;
  }
  return {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),
          2.0 * (x * z + w * y),       2.0 * (x * y + w * z),
          1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
          2.0 * (x * z - w * y),       2.0 * (y * z + w * x),
          1.0 - 2.0 * (x * x + y * y)};
}

Vec3 rotate_world_to_body(const double *quat_wxyz, const mjtNum *world_vec) {
  const auto r = quat_to_rotmat(quat_wxyz);
  return {r[0] * world_vec[0] + r[3] * world_vec[1] + r[6] * world_vec[2],
          r[1] * world_vec[0] + r[4] * world_vec[1] + r[7] * world_vec[2],
          r[2] * world_vec[0] + r[5] * world_vec[1] + r[8] * world_vec[2]};
}

class Go2MujocoSdk2Sim {
public:
  explicit Go2MujocoSdk2Sim(const Args &args) : args_(args) { init(); }

  ~Go2MujocoSdk2Sim() {
    if (window_) {
      mjr_freeContext(&context_);
      mjv_freeScene(&scene_);
      glfwDestroyWindow(window_);
      glfwTerminate();
    }
    if (data_)
      mj_deleteData(data_);
    if (model_)
      mj_deleteModel(model_);
  }

  void run() {
    print_startup_status();
    if (!args_.headless)
      init_viewer();

    const double start = now_seconds();
    double next_step = now_seconds();
    double next_render = now_seconds();
    double next_status = now_seconds() + args_.status_period;

    while (!shutdown_requested()) {
      const double elapsed = now_seconds() - start;
      if (args_.max_time > 0.0 && elapsed >= args_.max_time)
        break;

      step();

      const double t = now_seconds();
      if (args_.status_period > 0.0 && t >= next_status) {
        print_runtime_status();
        next_status = t + args_.status_period;
      }
      if (window_ && t >= next_render) {
        render();
        next_render = t + args_.viewer_dt;
      }

      next_step += args_.sim_dt;
      const double sleep_s = next_step - now_seconds();
      if (sleep_s > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
      } else {
        next_step = now_seconds();
      }
    }
  }

private:
  void init() {
    char error[1024] = {0};
    model_ = mj_loadXML(args_.scene.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
      throw std::runtime_error("加载 MuJoCo XML 失败：" + args_.scene + "\n" + error);
    }
    model_->opt.timestep = args_.sim_dt;
    data_ = mj_makeData(model_);
    if (!data_)
      throw std::runtime_error("创建 MuJoCo data 失败");

    base_body_id_ = mj_name2id(model_, mjOBJ_BODY, "base");
    if (base_body_id_ < 0)
      base_body_id_ = mj_name2id(model_, mjOBJ_BODY, "base_link");
    if (base_body_id_ < 0)
      throw std::runtime_error("MuJoCo 模型中找不到 base 或 base_link");

    home_key_id_ = mj_name2id(model_, mjOBJ_KEY, "home");
    if (home_key_id_ < 0)
      throw std::runtime_error("MuJoCo 模型中找不到 home keyframe");

    for (int i = 0; i < kMotorCount; ++i) {
      const int joint_id = mj_name2id(model_, mjOBJ_JOINT, kJointNames[i]);
      if (joint_id < 0)
        throw std::runtime_error(std::string("找不到关节：") + kJointNames[i]);
      qpos_adr_[i] = model_->jnt_qposadr[joint_id];
      qvel_adr_[i] = model_->jnt_dofadr[joint_id];

      const int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, kActuatorNames[i]);
      if (actuator_id < 0)
        throw std::runtime_error(std::string("找不到执行器：") + kActuatorNames[i]);
      actuator_id_[i] = actuator_id;
    }

    for (int i = 0; i < 4; ++i)
      foot_geom_id_[i] = mj_name2id(model_, mjOBJ_GEOM, kFootGeomNames[i]);

    accelerometer_sensor_id_ = mj_name2id(model_, mjOBJ_SENSOR, "accelerometer");
    control_mode_ = resolve_control_mode(args_.control_mode);

    mj_resetDataKeyframe(model_, data_, home_key_id_);
    for (int i = 0; i < kMotorCount; ++i)
      home_q_[i] = data_->qpos[qpos_adr_[i]];

    initial_q_ = make_pose(args_.initial_pose, home_q_);
    idle_q_ = args_.idle_target == IdleTarget::Home ? home_q_ : initial_q_;
    reset_to_initial_pose();

    ChannelFactory::Instance()->Init(args_.domain_id, args_.interface);
    lowstate_pub_.reset(new ChannelPublisher<LowState>("rt/lowstate"));
    lowstate_pub_->InitChannel();
    lowcmd_sub_.reset(new ChannelSubscriber<LowCmd>("rt/lowcmd"));
    lowcmd_sub_->InitChannel(
        std::bind(&Go2MujocoSdk2Sim::lowcmd_callback, this, std::placeholders::_1), 10);

    init_lowstate_template();
    publish_lowstate();
  }

  ControlMode resolve_control_mode(ControlMode requested) const {
    if (requested != ControlMode::Auto)
      return requested;
    for (int i = 0; i < kMotorCount; ++i) {
      const int id = actuator_id_[i];
      if (std::abs(model_->actuator_biasprm[id * mjNBIAS + 1]) > 1e-9)
        return ControlMode::PositionServo;
    }
    return ControlMode::PdTorque;
  }

  void reset_to_initial_pose() {
    mj_resetDataKeyframe(model_, data_, home_key_id_);
    for (int i = 0; i < kMotorCount; ++i)
      data_->qpos[qpos_adr_[i]] = initial_q_[i];
    if (args_.initial_pose == InitialPose::Crouch)
      data_->qpos[2] = 0.20;
    else if (args_.initial_pose == InitialPose::Prone)
      data_->qpos[2] = 0.13;
    mju_zero(data_->qvel, model_->nv);
    mj_forward(model_, data_);
    reset_hold_to_idle();
  }

  void reset_hold_to_idle() {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_cmd_ = CommandSnapshot{};
    hold_q_ = idle_q_;
    hold_dq_.fill(0.0);
    hold_kp_.fill(args_.idle_kp);
    hold_kd_.fill(args_.idle_kd);
    hold_from_command_ = false;
  }

  void init_lowstate_template() {
    lowstate_.head()[0] = 0xFE;
    lowstate_.head()[1] = 0xEF;
    lowstate_.level_flag() = 0xFF;
    for (int i = 0; i < kMotorSlotCount; ++i) {
      lowstate_.motor_state()[i].mode() = 0x01;
      lowstate_.motor_state()[i].q() = 0.0f;
      lowstate_.motor_state()[i].dq() = 0.0f;
      lowstate_.motor_state()[i].ddq() = 0.0f;
      lowstate_.motor_state()[i].tau_est() = 0.0f;
      lowstate_.motor_state()[i].temperature() = 0;
      lowstate_.motor_state()[i].lost() = 0;
    }
  }

  bool shutdown_requested() const {
    if (window_ && glfwWindowShouldClose(window_))
      return true;
    return false;
  }

  bool motor_cmd_is_active(const LowCmd &msg, int i) const {
    const auto &motor = msg.motor_cmd()[i];
    if (static_cast<int>(motor.mode()) != 0x01)
      return false;
    if (std::abs(static_cast<double>(motor.q()) - kPosStopF) < 1e3)
      return false;
    if (std::abs(static_cast<double>(motor.dq()) - kVelStopF) < 1e-3)
      return false;
    if (std::abs(static_cast<double>(motor.kp())) < 1e-6 &&
        std::abs(static_cast<double>(motor.kd())) < 1e-6 &&
        std::abs(static_cast<double>(motor.tau())) < 1e-6)
      return false;
    return true;
  }

  void lowcmd_callback(const void *message) {
    const auto &msg = *static_cast<const LowCmd *>(message);

    std::array<double, kMotorCount> q_unitree{};
    std::array<double, kMotorCount> dq_unitree{};
    std::array<double, kMotorCount> kp_unitree{};
    std::array<double, kMotorCount> kd_unitree{};
    std::array<double, kMotorCount> tau_unitree{};
    bool active = false;
    for (int i = 0; i < kMotorCount; ++i) {
      const auto &motor = msg.motor_cmd()[i];
      q_unitree[i] = motor.q();
      dq_unitree[i] = motor.dq();
      kp_unitree[i] = motor.kp();
      kd_unitree[i] = motor.kd();
      tau_unitree[i] = motor.tau();
      active = active || motor_cmd_is_active(msg, i);
    }

    CommandSnapshot snapshot;
    snapshot.active = active;
    snapshot.q = unitree_to_policy(q_unitree);
    snapshot.dq = unitree_to_policy(dq_unitree);
    snapshot.kp = unitree_to_policy(kp_unitree);
    snapshot.kd = unitree_to_policy(kd_unitree);
    snapshot.tau = unitree_to_policy(tau_unitree);
    snapshot.received_at = now_seconds();

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      latest_cmd_ = snapshot;
      if (snapshot.active) {
        hold_q_ = snapshot.q;
        hold_dq_ = snapshot.dq;
        hold_kp_ = snapshot.kp;
        hold_kd_ = snapshot.kd;
        hold_from_command_ = true;
      }
    }

    if (!lowcmd_seen_) {
      lowcmd_seen_ = true;
      std::cout << "[仿真] 已收到第一帧 rt/lowcmd" << std::endl;
    }
  }

  CommandSnapshot get_command() const {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    return latest_cmd_;
  }

  ControlSnapshot get_control_snapshot() const {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    return {latest_cmd_, hold_q_, hold_dq_, hold_kp_, hold_kd_, hold_from_command_};
  }

  void read_joint_state(std::array<double, kMotorCount> &q,
                        std::array<double, kMotorCount> &dq) const {
    for (int i = 0; i < kMotorCount; ++i) {
      q[i] = data_->qpos[qpos_adr_[i]];
      dq[i] = data_->qvel[qvel_adr_[i]];
    }
  }

  void set_position_servo_pd(const std::array<double, kMotorCount> &kp,
                             const std::array<double, kMotorCount> &kd) {
    for (int i = 0; i < kMotorCount; ++i) {
      const int id = actuator_id_[i];
      model_->actuator_gainprm[id * mjNGAIN + 0] = kp[i];
      model_->actuator_biasprm[id * mjNBIAS + 1] = -kp[i];
      model_->actuator_biasprm[id * mjNBIAS + 2] = -kd[i];
    }
  }

  void set_position_servo_pd(double kp, double kd) {
    std::array<double, kMotorCount> kp_arr{};
    std::array<double, kMotorCount> kd_arr{};
    kp_arr.fill(kp);
    kd_arr.fill(kd);
    set_position_servo_pd(kp_arr, kd_arr);
  }

  void compute_control() {
    std::array<double, kMotorCount> q{};
    std::array<double, kMotorCount> dq{};
    read_joint_state(q, dq);

    const auto control = get_control_snapshot();
    const auto &cmd = control.cmd;
    const bool fresh = (now_seconds() - cmd.received_at) <= args_.cmd_timeout;
    std::array<double, kMotorCount> ctrl{};

    if (control_mode_ == ControlMode::PositionServo) {
      if (cmd.active && fresh) {
        set_position_servo_pd(cmd.kp, cmd.kd);
        ctrl = cmd.q;
      } else {
        set_position_servo_pd(control.hold_kp, control.hold_kd);
        ctrl = control.hold_q;
      }
    } else {
      if (cmd.active && fresh) {
        for (int i = 0; i < kMotorCount; ++i)
          ctrl[i] = cmd.tau[i] + cmd.kp[i] * (cmd.q[i] - q[i]) + cmd.kd[i] * (cmd.dq[i] - dq[i]);
      } else {
        for (int i = 0; i < kMotorCount; ++i)
          ctrl[i] = control.hold_kp[i] * (control.hold_q[i] - q[i]) +
                    control.hold_kd[i] * (control.hold_dq[i] - dq[i]);
      }
    }

    for (int i = 0; i < kMotorCount; ++i)
      data_->ctrl[actuator_id_[i]] = clamp_to_range(ctrl[i], model_, actuator_id_[i]);
  }

  std::array<double, 4> compute_foot_force_policy_order() const {
    std::array<double, 4> force{};
    for (int ci = 0; ci < data_->ncon; ++ci) {
      const mjContact &contact = data_->contact[ci];
      for (int leg = 0; leg < 4; ++leg) {
        const int foot_id = foot_geom_id_[leg];
        if (foot_id < 0)
          continue;
        if (contact.geom1 == foot_id || contact.geom2 == foot_id) {
          mjtNum contact_force[6] = {0, 0, 0, 0, 0, 0};
          mj_contactForce(model_, data_, ci, contact_force);
          force[leg] += std::abs(contact_force[0]);
        }
      }
    }
    return force;
  }

  void publish_lowstate() {
    std::array<double, kMotorCount> q_policy{};
    std::array<double, kMotorCount> dq_policy{};
    std::array<double, kMotorCount> tau_policy{};
    read_joint_state(q_policy, dq_policy);
    for (int i = 0; i < kMotorCount; ++i)
      tau_policy[i] = data_->actuator_force[actuator_id_[i]];

    const auto q_unitree = policy_to_unitree(q_policy);
    const auto dq_unitree = policy_to_unitree(dq_policy);
    const auto tau_unitree = policy_to_unitree(tau_policy);

    for (int i = 0; i < kMotorCount; ++i) {
      auto &motor = lowstate_.motor_state()[i];
      motor.mode() = 0x01;
      motor.q() = static_cast<float>(q_unitree[i]);
      motor.dq() = static_cast<float>(dq_unitree[i]);
      motor.ddq() = 0.0f;
      motor.tau_est() = static_cast<float>(tau_unitree[i]);
      motor.temperature() = 0;
      motor.lost() = 0;
    }

    const double *quat = data_->xquat + 4 * base_body_id_;
    auto &imu = lowstate_.imu_state();
    for (int i = 0; i < 4; ++i)
      imu.quaternion()[i] = static_cast<float>(quat[i]);

    mjtNum vel_world[6] = {0, 0, 0, 0, 0, 0};
    mj_objectVelocity(model_, data_, mjOBJ_BODY, base_body_id_, vel_world, 0);
    const Vec3 gyro_body = rotate_world_to_body(quat, vel_world);
    imu.gyroscope()[0] = static_cast<float>(gyro_body.x);
    imu.gyroscope()[1] = static_cast<float>(gyro_body.y);
    imu.gyroscope()[2] = static_cast<float>(gyro_body.z);

    if (accelerometer_sensor_id_ >= 0) {
      const int adr = model_->sensor_adr[accelerometer_sensor_id_];
      imu.accelerometer()[0] = static_cast<float>(data_->sensordata[adr + 0]);
      imu.accelerometer()[1] = static_cast<float>(data_->sensordata[adr + 1]);
      imu.accelerometer()[2] = static_cast<float>(data_->sensordata[adr + 2]);
    } else {
      imu.accelerometer()[0] = 0.0f;
      imu.accelerometer()[1] = 0.0f;
      imu.accelerometer()[2] = 0.0f;
    }

    const auto foot_force_policy = compute_foot_force_policy_order();
    for (int leg = 0; leg < 4; ++leg) {
      const int unitree_leg = kPolicyFootToUnitree[leg];
      const double value = std::clamp(foot_force_policy[leg], 0.0, 65535.0);
      lowstate_.foot_force()[unitree_leg] = static_cast<int>(value);
    }

    lowstate_pub_->Write(lowstate_);
  }

  void step() {
    compute_control();
    mj_step(model_, data_);
    ++step_count_;
    publish_lowstate();
  }

  void print_startup_status() const {
    std::cout << "\n========== AHAC Go2 MuJoCo + SDK2 仿真器 ==========" << std::endl;
    std::cout << "场景 XML: " << args_.scene << std::endl;
    std::cout << "DDS: interface=" << args_.interface << " domain_id=" << args_.domain_id << std::endl;
    std::cout << "仿真步长: " << args_.sim_dt << " s" << std::endl;
    std::cout << "控制模式: " << control_mode_name(control_mode_) << std::endl;
    std::cout << "发布 rt/lowstate，订阅 rt/lowcmd" << std::endl;
    std::cout << "按 Esc 关闭 MuJoCo 界面，按 r 重置到初始姿态。" << std::endl;
  }

  void print_runtime_status() const {
    const auto cmd = get_command();
    const bool fresh = (now_seconds() - cmd.received_at) <= args_.cmd_timeout;
    const double vx = data_->qvel[0];
    const double vy = data_->qvel[1];
    const double z = data_->qpos[2];
    std::cout << "[仿真] t=" << data_->time << " lowcmd="
              << (cmd.active ? "active" : "inactive") << "/" << (fresh ? "fresh" : "stale")
              << " base_z=" << z << " qvel_xy=(" << vx << ", " << vy << ")" << std::endl;
  }

  void init_viewer() {
    if (!glfwInit())
      throw std::runtime_error("初始化 GLFW 失败");
    window_ = glfwCreateWindow(1280, 900, "AHAC Go2 SDK2 仿真部署测试", nullptr, nullptr);
    if (!window_)
      throw std::runtime_error("创建 MuJoCo 窗口失败");
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, key_callback);
    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetCursorPosCallback(window_, mouse_move_callback);
    glfwSetScrollCallback(window_, scroll_callback);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    camera_.azimuth = -130.0;
    camera_.elevation = -20.0;
    camera_.distance = 2.2;
    camera_.lookat[0] = 0.0;
    camera_.lookat[1] = 0.0;
    camera_.lookat[2] = 0.25;
  }

  void render() {
    glfwPollEvents();
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    mjrRect viewport{0, 0, width, height};
    mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &context_);
    glfwSwapBuffers(window_);
  }

  static Go2MujocoSdk2Sim *from_window(GLFWwindow *window) {
    return static_cast<Go2MujocoSdk2Sim *>(glfwGetWindowUserPointer(window));
  }

  static void key_callback(GLFWwindow *window, int key, int, int action, int) {
    if (action != GLFW_PRESS)
      return;
    auto *sim = from_window(window);
    if (key == GLFW_KEY_ESCAPE)
      glfwSetWindowShouldClose(window, GLFW_TRUE);
    else if (key == GLFW_KEY_R && sim) {
      sim->reset_to_initial_pose();
      std::cout << "[仿真] 已重置到初始姿态" << std::endl;
    }
  }

  static void mouse_button_callback(GLFWwindow *window, int, int, int) {
    auto *sim = from_window(window);
    if (!sim)
      return;
    sim->button_left_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    sim->button_middle_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    sim->button_right_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &sim->last_x_, &sim->last_y_);
  }

  static void mouse_move_callback(GLFWwindow *window, double xpos, double ypos) {
    auto *sim = from_window(window);
    if (!sim || (!sim->button_left_ && !sim->button_middle_ && !sim->button_right_))
      return;
    const double dx = xpos - sim->last_x_;
    const double dy = ypos - sim->last_y_;
    sim->last_x_ = xpos;
    sim->last_y_ = ypos;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);
    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    int action = mjMOUSE_ZOOM;
    if (sim->button_right_)
      action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (sim->button_left_)
      action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else if (sim->button_middle_)
      action = mjMOUSE_ZOOM;
    mjv_moveCamera(sim->model_, action, dx / std::max(1, height), dy / std::max(1, height),
                   &sim->scene_, &sim->camera_);
  }

  static void scroll_callback(GLFWwindow *window, double, double yoffset) {
    auto *sim = from_window(window);
    if (!sim)
      return;
    mjv_moveCamera(sim->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &sim->scene_,
                   &sim->camera_);
  }

  Args args_;
  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;
  GLFWwindow *window_ = nullptr;
  mjvCamera camera_{};
  mjvOption option_{};
  mjvScene scene_{};
  mjrContext context_{};

  int base_body_id_ = -1;
  int home_key_id_ = -1;
  int accelerometer_sensor_id_ = -1;
  std::array<int, kMotorCount> qpos_adr_{};
  std::array<int, kMotorCount> qvel_adr_{};
  std::array<int, kMotorCount> actuator_id_{};
  std::array<int, 4> foot_geom_id_{};
  std::array<double, kMotorCount> home_q_{};
  std::array<double, kMotorCount> initial_q_{};
  std::array<double, kMotorCount> idle_q_{};
  std::array<double, kMotorCount> hold_q_{};
  std::array<double, kMotorCount> hold_dq_{};
  std::array<double, kMotorCount> hold_kp_{};
  std::array<double, kMotorCount> hold_kd_{};
  ControlMode control_mode_ = ControlMode::PositionServo;

  ChannelPublisherPtr<LowState> lowstate_pub_;
  ChannelSubscriberPtr<LowCmd> lowcmd_sub_;
  LowState lowstate_{};
  mutable std::mutex cmd_mutex_;
  CommandSnapshot latest_cmd_{};
  bool hold_from_command_ = false;
  bool lowcmd_seen_ = false;
  long long step_count_ = 0;

  bool button_left_ = false;
  bool button_middle_ = false;
  bool button_right_ = false;
  double last_x_ = 0.0;
  double last_y_ = 0.0;
};

} // 命名空间

int main(int argc, char **argv) {
  try {
    const Args args = parse_args(argc, argv);
    Go2MujocoSdk2Sim sim(args);
    sim.run();
    return 0;
  } catch (const std::exception &exc) {
    std::cerr << "错误：" << exc.what() << std::endl;
    return 1;
  }
}
