# AHAC MuJoCo + SDK2 仿真部署测试

这套仿真测试用于在不上实机前验证 AHAC 部署链路：MuJoCo 进程模拟 Go2 低层硬件，通过 C++ `unitree_sdk2` 发布 `rt/lowstate` 并订阅 `rt/lowcmd`；AHAC 部署进程仍使用仓库里的 C++ 实机控制器，按真实部署流程完成状态机、观测构造、策略推理和 LowCmd 发布。

## 1. 测试覆盖范围

- `IDLE`：部署控制器发布零力矩，仿真器用空闲 PD 保持初始姿态。
- `STANDUP`：部署控制器从当前关节插值到策略默认站立姿态。
- `READY`：站立完成后保持默认姿态，等待进入行走。
- `WALKING`：AHAC 策略 50 Hz 推理，500 Hz 发送目标关节，仿真器实时执行。
- `ESTOP`：终端输入 `x` 后部署控制器保持当前关节。
- `SITDOWN`：`Ctrl-C` 或输入结束后触发平滑坐下并退出。

## 2. 安装依赖

基础依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3 python3-venv python3-pip \
  libeigen3-dev zlib1g-dev libglfw3-dev libgl1-mesa-dev
```

需要先安装 Unitree SDK2 C++，让 CMake 能找到 `unitree_sdk2`：

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

MuJoCo 可以使用系统安装，也可以直接复用 Python `mujoco` 包内自带的 C 头文件和动态库。若已安装 Python 包，构建脚本会自动尝试识别；如果识别不到，手动指定：

```bash
export MUJOCO_ROOT=/path/to/python/site-packages/mujoco
```

## 3. 构建仿真端和部署端

从 `MJX-DEPLOY` 仓库根目录执行：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

构建 MuJoCo + SDK2 仿真低层：

```bash
python -m mjx_deploy.ahac_go2_deploy build-sim
```

如果 `cmake` 已安装但不在 PATH，可以显式指定：

```bash
CMAKE=/path/to/cmake python -m mjx_deploy.ahac_go2_deploy build-sim
```

等价脚本：

```bash
./scripts/build_sim.sh
```

构建 AHAC C++ 部署控制器：

```bash
python -m mjx_deploy.ahac_go2_deploy build
```

## 4. 交互式查看策略表现

终端 B 先启动仿真器，默认打开 MuJoCo 界面：

```bash
python -m mjx_deploy.ahac_go2_deploy sim \
  --interface lo \
  --domain-id 1 \
  --build-if-missing
```

等价脚本：

```bash
./scripts/run_mujoco_sdk2_sim.sh
```

终端 A 再启动 AHAC 部署控制器，连接同一个 loopback DDS 域：

```bash
python -m mjx_deploy.ahac_go2_deploy run \
  --interface lo \
  --domain-id 1 \
  --command-source terminal \
  --build-if-missing
```

等价脚本：

```bash
./scripts/run_ahac_sim_terminal.sh
```

终端 A 的操作方式：

| 输入 | 作用 |
| --- | --- |
| 空 Enter | `IDLE -> STANDUP -> READY` |
| 再按空 Enter | `READY -> WALKING`，速度指令为零 |
| `w` | `vx += 0.1 m/s` |
| `s` | `vx -= 0.1 m/s` |
| `a` | `vy += 0.1 m/s` |
| `d` | `vy -= 0.1 m/s` |
| `q` | `yaw += 0.1 rad/s` |
| `e` | `yaw -= 0.1 rad/s` |
| `0` | 三个速度指令清零 |
| `x` | 进入 `ESTOP` 并保持当前关节 |
| `Ctrl-C` | 平滑坐下并退出 |

建议第一轮按这个顺序验证：

```text
Enter
Enter
w
0
s
0
a
0
d
0
q
0
e
0
Ctrl-C
```

## 5. MuJoCo 界面操作

- 鼠标左键拖动：旋转视角。
- 鼠标右键拖动：平移视角。
- 鼠标中键或滚轮：缩放视角。
- `r`：把仿真中的 Go2 重置到初始姿态。
- `Esc`：关闭仿真窗口。

窗口左上角会显示 DDS 网卡、DDS 域、是否收到有效 LowCmd、仿真时间和机身高度。终端里也会周期性打印 `base_z` 和机身平面速度。

## 6. 无界面冒烟测试

只验证 DDS 链路和状态切换时，可以让仿真器无界面运行：

```bash
python -m mjx_deploy.ahac_go2_deploy sim \
  --interface lo \
  --domain-id 1 \
  --headless \
  --max-time 20 \
  --build-if-missing
```

另一个终端启动部署控制器后，至少按两次空 Enter 进入行走，再输入 `0` 和 `Ctrl-C`。仿真端应打印已收到第一帧 `rt/lowcmd`，部署端应打印已收到 robot state，并正常完成坐下流程。

## 7. 常用参数

- `--scene models/go2/scene_mjx.xml`：默认使用 AHAC 训练同源 Go2 MuJoCo 模型。
- `--initial-pose home|crouch|prone`：仿真初始姿态，默认 `home`。
- `--idle-target initial|home`：无有效 LowCmd 时保持初始姿态或 home 姿态，默认 `home`。
- `--control-mode auto|position_servo|pd_torque`：默认自动识别 MuJoCo 执行器语义。
- `--sim-dt 0.002`：仿真低层步长，匹配 500 Hz LowCmd 频率。
- `--viewer-dt 0.02`：界面刷新间隔，匹配 50 Hz 策略节奏。

## 8. 结果判断

- `STANDUP` 期间四腿应平滑站起，不应突然翻倒或高频抖动。
- `READY` 期间机身应基本稳定，腿部不应明显歪斜。
- `WALKING` 零速度时应近似原地踏步或稳定站立。
- 前后、侧向和偏航指令应能在 MuJoCo 界面里看到明确响应。
- `x` 后应保持当前关节，`Ctrl-C` 后应进入坐下并退出。

如果仿真端一直显示没有有效 LowCmd，检查两个终端是否都使用 `--interface lo --domain-id 1`。如果部署端提示 10 秒内没有 state，检查仿真端是否已经启动、是否订阅/发布同一 DDS 域。若 CMake 找不到 MuJoCo，显式设置 `MUJOCO_ROOT`；若找不到 `unitree_sdk2`，设置 `CMAKE_PREFIX_PATH=/usr/local` 或 SDK2 的安装前缀。
