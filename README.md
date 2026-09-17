# MJX-DEPLOY

这是用于在 Unitree Go2 上部署 MJX / Open-DiffLoco 训练策略的独立仓库。

当前默认打包策略仍然是 **AHAC 0828-180341** 这一版：

```text
policies/ahac_go2_blind_nolinvel_nokinref/policy_best_tracking_deploy.npz
```

仓库内容：

- `policies/ahac_go2_blind_nolinvel_nokinref/policy_best_tracking_deploy.npz`：默认 AHAC 部署策略。
- `cpp/deploy_blind_nolinvel_nokinref`：基于 `unitree_sdk2` 的 C++ 低层部署控制器。
- `python/mjx_deploy`：构建、运行、检查策略的 Python 命令入口。
- `examples/unitree_sdk2/go2_stand_example.cpp`：宇树官方 Go2 起立示例，用于部署前测试低层链路。
- `docs/AHAC_GO2_DEPLOYMENT.md`：完整实机部署说明。
- `docs/AHAC_MUJOCO_SDK2_SIM_TEST.md`：本机 MuJoCo + SDK2 仿真部署流程测试说明。

## 1. 首次安装

在部署笔记本上执行：

```bash
git clone https://github.com/Iris152/MJX-DEPLOY.git
cd MJX-DEPLOY
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

需要先安装 Unitree SDK2 C++，并保证 CMake 能找到 `unitree_sdk2`：

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

回到本仓库后构建部署程序和官方 stand example：

```bash
cd /path/to/MJX-DEPLOY
source .venv/bin/activate
python -m mjx_deploy.ahac_go2_deploy build --build-dir build
```

## 2. 确认网卡

先查看连接 Go2 的有线网卡名：

```bash
ip -br link
```

下面命令里的 `eth0` 请替换成你实际连接 Go2 的网卡，例如 `enp3s0`、`enx...` 等。

## 3. 先运行宇树官方 Stand Example

正式跑学习策略前，先用宇树官方起立示例确认：网卡正确、SDK2 通信正常、机器人低层控制可用。

```bash
cd /path/to/MJX-DEPLOY
source .venv/bin/activate
python -m mjx_deploy.ahac_go2_deploy stand-example \
  --interface eth0 \
  --build-if-missing
```

这条命令实际会运行：

```bash
./build/go2_stand_example eth0
```

如果 stand example 无法收到状态、无法起立或机器人状态异常，不要继续运行学习策略，先检查网卡、供电、急停、运动服务占用和 SDK2 安装。

## 4. 正式运行 AHAC 策略

确认 stand example 正常后，再启动 AHAC 0828-180341 部署策略：

```bash
cd /path/to/MJX-DEPLOY
source .venv/bin/activate
python -m mjx_deploy.ahac_go2_deploy run \
  --interface eth0 \
  --domain-id 0 \
  --command-source terminal \
  --build-if-missing
```

如果你要显式指定策略文件，可以使用：

```bash
python -m mjx_deploy.ahac_go2_deploy run \
  --policy policies/ahac_go2_blind_nolinvel_nokinref/policy_best_tracking_deploy.npz \
  --interface eth0 \
  --domain-id 0 \
  --command-source terminal \
  --build-if-missing
```

终端控制方式：

| 输入 | 作用 |
| --- | --- |
| 空 Enter | 从 `IDLE` 起立到 `READY` |
| 再按空 Enter | 从 `READY` 进入 `WALKING`，速度指令为 0 |
| `w` + Enter | `vx += 0.1 m/s`，前进速度增加 |
| `s` + Enter | `vx -= 0.1 m/s`，后退速度增加 |
| `a` + Enter | `vy += 0.1 m/s`，向左横移 |
| `d` + Enter | `vy -= 0.1 m/s`，向右横移 |
| `q` + Enter | `yaw += 0.1 rad/s`，左转 |
| `e` + Enter | `yaw -= 0.1 rad/s`，右转 |
| `0` + Enter | 三个速度指令清零 |
| `x` + Enter | 三个速度指令清零，并从当前动作平滑回到策略默认站立姿态 |
| `Ctrl-C` | 平滑坐下并退出程序 |

部署控制器内置安全保护：

- 默认机身倾角阈值约 `40 deg`，可用 `--tilt-limit-deg` 覆盖。
- 默认 `LowState` watchdog 为 `0.20s`，可用 `--lowstate-timeout` 覆盖。
- tilt safety、LowState 超时、IMU 四元数异常会触发 `ESTOP` 并保持当前关节。

## 5. 本机仿真验证

不上实机前，可以用两个终端在本机验证完整部署流程。

终端 A 启动 MuJoCo + SDK2 低层仿真：

```bash
cd /path/to/MJX-DEPLOY
source .venv/bin/activate
python -m mjx_deploy.ahac_go2_deploy sim \
  --interface lo \
  --domain-id 1 \
  --build-if-missing
```

终端 B 启动同一个 C++ 部署控制器：

```bash
cd /path/to/MJX-DEPLOY
source .venv/bin/activate
python -m mjx_deploy.ahac_go2_deploy run \
  --interface lo \
  --domain-id 1 \
  --command-source terminal \
  --build-if-missing
```

仿真器默认从趴地姿态开始；部署端第一次空 Enter 起立，第二次空 Enter 进入策略行走。按 `x` 会清零速度并回站立，按 `Ctrl-C` 会坐下并退出。

更多细节见：

- `docs/AHAC_GO2_DEPLOYMENT.md`
- `docs/AHAC_MUJOCO_SDK2_SIM_TEST.md`
