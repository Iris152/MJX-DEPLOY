# AHAC Go2 Real-Robot Deployment

This repository contains the deployment-side code and exported policy needed to
run the accepted AHAC `blind_nolinvel_nokinref` policy on a Unitree Go2 from a
separate laptop.

## Contents

- `policies/ahac_go2_blind_nolinvel_nokinref/policy_best_tracking_deploy.npz`:
  exported AHAC policy from `ahac_20260828_180341`.
- `cpp/deploy_blind_nolinvel_nokinref`: C++ low-level DDS controller and MLP
  policy runtime.
- `python/mjx_deploy/ahac_go2_deploy.py`: build/run wrapper for the packaged
  AHAC policy.
- `python/mjx_deploy/terminal_command.py`: optional ROS2 keyboard publisher for
  `/velocity_command`.
- `python/mjx_deploy/wireless_command.py`: optional ROS2 Unitree remote publisher
  for `/velocity_command`.
- `examples/unitree_sdk2/go2_stand_example.cpp`: official Unitree SDK2 Go2 stand
  example, copied from `unitree_sdk2/example/go2/go2_stand_example.cpp`.

## 1. Prepare the Laptop

Use Ubuntu with wired Ethernet connected to the Go2 DDS network. Install system
packages:

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3 python3-venv python3-pip \
  libeigen3-dev zlib1g-dev
```

Install Unitree SDK2 C++ and make sure CMake can find `unitree_sdk2`:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir -p build && cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

If `cmake -S . -B build` later cannot find `unitree_sdk2`, pass the SDK install
prefix, for example:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local
```

## 2. Install Python Helpers

From the `MJX-DEPLOY` repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
python -m mjx_deploy.inspect_policy_npz
```

The policy inspection should report:

```text
actor: 450 -> 512 -> 256 -> 128 -> 12
obs history: 10 x 45
dt: 0.0200s
cmd_vel_x_range: [-1.5, 1.5]
cmd_vel_y_range: [-1.0, 1.0]
cmd_yaw_rate_range: [-1.5, 1.5]
```

## 3. Build the Controller and Stand Example

Build the normal terminal/wireless deployment binary:

```bash
python -m mjx_deploy.ahac_go2_deploy build
```

Equivalent direct CMake command:

```bash
cmake -S . -B build -DMJX_DEPLOY_ENABLE_ROS2=OFF -DMJX_DEPLOY_BUILD_STAND_EXAMPLE=ON
cmake --build build -j"$(nproc)"
```

This produces:

```text
build/cpp/deploy_blind_nolinvel_nokinref/deploy_blind_nolinvel_nokinref
build/go2_stand_example
```

Only build with ROS2 if you need a `/velocity_command` command topic:

```bash
python -m mjx_deploy.ahac_go2_deploy build --ros2 --build-dir build-ros2
```

## 4. Identify the Robot Network Interface

Run:

```bash
ip -br link
```

Use the wired adapter connected to the Go2, commonly `eth0`, `enp3s0`, or
`enx...`. Use `lo` only for simulator/loopback testing.

## 5. Test Unitree Low-Level Control First

Run the official Unitree stand example before launching the learned policy:

```bash
python -m mjx_deploy.ahac_go2_deploy stand-example \
  --interface eth0 --build-if-missing
```

The upstream example prints `WARNING: Make sure the robot is hung up or lying on
the ground.` Follow that warning. This file is intentionally unchanged from the
official SDK example, except for its location inside this repository.

## 6. Dry-Run AHAC Launch

Print the exact command without sending motor commands:

```bash
python -m mjx_deploy.ahac_go2_deploy run \
  --interface eth0 --command-source terminal --dry-run
```

Replace `eth0` with the real interface from `ip -br link`.

## 7. Run AHAC on the Go2

Start with the robot supported or with plenty of clear space, keep the physical
emergency stop ready, and make sure no other sport/low-level controller is
running.

```bash
python -m mjx_deploy.ahac_go2_deploy run \
  --interface eth0 --command-source terminal --build-if-missing
```

Terminal operation is line based:

| Input | Effect |
| --- | --- |
| empty Enter from `IDLE` | stand up to the policy default pose |
| empty Enter from `READY` | enter `WALKING` with zero velocity |
| `w` + Enter | increase forward `vx` by `+0.1 m/s` |
| `s` + Enter | decrease `vx` by `-0.1 m/s` for backward walking |
| `a` + Enter | increase lateral `vy` left by `+0.1 m/s` |
| `d` + Enter | decrease lateral `vy` right by `-0.1 m/s` |
| `q` + Enter | increase yaw rate left by `+0.1 rad/s` |
| `e` + Enter | decrease yaw rate right by `-0.1 rad/s` |
| `0` + Enter | zero all velocity commands |
| `x` + Enter | emergency stop and hold current joints |
| `Ctrl-C` | sit down and exit |

Recommended first test:

```text
Enter      # IDLE -> STANDUP -> READY
Enter      # READY -> WALKING, command remains zero
w Enter    # vx = +0.1 m/s
0 Enter    # zero command
s Enter    # vx = -0.1 m/s
0 Enter    # zero command
Ctrl-C     # sit down and exit
```

Stay inside the exported command ranges: `vx +/-1.5 m/s`, `vy +/-1.0 m/s`, and
`yaw +/-1.5 rad/s`. Increase commands gradually.

## 8. Wireless and ROS2 Command Options

Built-in wireless control uses the Unitree remote data from `LowState` directly:

```bash
python -m mjx_deploy.ahac_go2_deploy run \
  --interface eth0 --command-source wireless --build-if-missing
```

Stick mapping:

- Left stick Y: forward/backward `vx`.
- Left stick X: lateral `vy`.
- Right stick X: yaw rate.
- Release sticks to return commands toward zero.

For ROS2 command-topic control, build with ROS2 and run:

```bash
python -m mjx_deploy.ahac_go2_deploy build --ros2 --build-dir build-ros2
python -m mjx_deploy.ahac_go2_deploy run \
  --interface eth0 --command-source ros2 --build-dir build-ros2 \
  --cmd-topic /velocity_command
```

Then publish commands from another terminal:

```bash
python -m mjx_deploy.terminal_command --control diffloco
```

For ROS2 wireless command publishing, install Unitree SDK2 Python bindings and
run:

```bash
python -m mjx_deploy.wireless_command \
  --net eth0 --control diffloco --topic /velocity_command
```

## 9. Troubleshooting

- `ERROR: No state after 10s`: check robot power, Ethernet interface, DDS domain
  ID, firewall, and whether another controller is occupying the robot.
- CMake cannot find `unitree_sdk2`: reinstall SDK2 or pass `-DCMAKE_PREFIX_PATH`
  to the SDK install location.
- Robot vibrates during stand-up: stop immediately with the physical e-stop or
  `x`, then verify joint order, network delay, and PD gains.
- Policy response is clipped: confirm the controller startup log prints the
  exported command ranges and that the `.npz` file is the packaged AHAC policy.
- Use `--kp` and `--kd` only for controlled troubleshooting. The default walking
  gains come from `policy_best_tracking_deploy.npz`.

