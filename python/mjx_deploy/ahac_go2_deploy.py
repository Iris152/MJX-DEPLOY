#!/usr/bin/env python3
"""构建并启动仓库内打包好的 AHAC Go2 部署策略。"""

from __future__ import annotations

import argparse
import importlib.util
import os
import signal
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
# 默认使用已经打包进仓库的 AHAC 实机部署策略。
POLICY_PATH = (
    REPO_ROOT
    / "policies"
    / "ahac_go2_blind_nolinvel_nokinref"
    / "policy_best_tracking_deploy.npz"
)
EXECUTABLE_NAME = "deploy_blind_nolinvel_nokinref"
EXECUTABLE_SUBPATH = Path("cpp") / "deploy_blind_nolinvel_nokinref" / EXECUTABLE_NAME
SIM_EXECUTABLE_NAME = "go2_mujoco_sdk2_sim"
SIM_EXECUTABLE_SUBPATH = Path("cpp") / "mujoco_sdk2_sim" / SIM_EXECUTABLE_NAME
SIM_SCENE_PATH = REPO_ROOT / "models" / "go2" / "scene_mjx.xml"


def _rel(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def _cmake() -> str:
    return os.environ.get("CMAKE", "cmake")


def _build_dir(value: str | None, ros2: bool) -> Path:
    """根据命令行参数选择普通构建目录或 ROS2 构建目录。"""
    build_dir = Path(value) if value else Path("build-ros2" if ros2 else "build")
    return build_dir if build_dir.is_absolute() else REPO_ROOT / build_dir


def _exe_path(build_dir: Path) -> Path:
    return build_dir / EXECUTABLE_SUBPATH


def _sim_build_dir(value: str | None) -> Path:
    build_dir = Path(value) if value else Path("build-sim")
    return build_dir if build_dir.is_absolute() else REPO_ROOT / build_dir


def _sim_exe_path(build_dir: Path) -> Path:
    return build_dir / SIM_EXECUTABLE_SUBPATH


def _default_mujoco_root() -> Path | None:
    """优先复用 Python mujoco 包内自带的 C 头文件和动态库。"""
    spec = importlib.util.find_spec("mujoco")
    if spec is None or spec.origin is None:
        return None
    root = Path(spec.origin).resolve().parent
    if (root / "include" / "mujoco" / "mujoco.h").exists() and any(
        root.glob("libmujoco.so*")
    ):
        return root
    return None


def _run_child(cmd: list[str], *, cwd: Path = REPO_ROOT) -> int:
    """运行 C++ 子进程；收到 Ctrl-C 时转发中断并等待控制器安全收尾。"""
    proc = subprocess.Popen(cmd, cwd=cwd)
    try:
        return proc.wait()
    except KeyboardInterrupt:
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            return proc.returncode or 130
        try:
            return proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            print("Interrupted; child process did not exit within 20s", file=sys.stderr)
            return 130


def build_cmd(args: argparse.Namespace) -> int:
    """配置并编译 C++ 低层控制器。"""
    build_dir = _build_dir(args.build_dir, args.ros2)
    configure = [
        _cmake(),
        "-S",
        ".",
        "-B",
        _rel(build_dir),
        f"-DMJX_DEPLOY_ENABLE_ROS2={'ON' if args.ros2 else 'OFF'}",
        "-DMJX_DEPLOY_BUILD_CONTROLLER=ON",
        f"-DMJX_DEPLOY_BUILD_STAND_EXAMPLE={'ON' if args.stand_example else 'OFF'}",
        "-DMJX_DEPLOY_BUILD_MUJOCO_SIM=OFF",
    ]
    build = [_cmake(), "--build", _rel(build_dir)]
    if args.jobs:
        build.extend(["-j", str(args.jobs)])

    print("Configuring:", " ".join(configure))
    subprocess.run(configure, cwd=REPO_ROOT, check=True)
    print("Building:", " ".join(build))
    subprocess.run(build, cwd=REPO_ROOT, check=True)
    return 0


def build_sim_cmd(args: argparse.Namespace) -> int:
    """配置并编译 C++ MuJoCo + SDK2 仿真低层。"""
    build_dir = _sim_build_dir(args.build_dir)
    configure = [
        _cmake(),
        "-S",
        ".",
        "-B",
        _rel(build_dir),
        "-DMJX_DEPLOY_ENABLE_ROS2=OFF",
        "-DMJX_DEPLOY_BUILD_CONTROLLER=OFF",
        "-DMJX_DEPLOY_BUILD_STAND_EXAMPLE=OFF",
        "-DMJX_DEPLOY_BUILD_MUJOCO_SIM=ON",
    ]
    if args.cmake_prefix_path:
        configure.append(f"-DCMAKE_PREFIX_PATH={args.cmake_prefix_path}")

    env = os.environ.copy()
    mujoco_root = Path(args.mujoco_root).expanduser() if args.mujoco_root else None
    if mujoco_root is None and not env.get("MUJOCO_ROOT"):
        mujoco_root = _default_mujoco_root()
    if mujoco_root is not None:
        env["MUJOCO_ROOT"] = str(mujoco_root.resolve())

    build = [_cmake(), "--build", _rel(build_dir)]
    if args.jobs:
        build.extend(["-j", str(args.jobs)])

    if env.get("MUJOCO_ROOT"):
        print(f"MUJOCO_ROOT: {env['MUJOCO_ROOT']}")
    print("Configuring simulator:", " ".join(configure))
    subprocess.run(configure, cwd=REPO_ROOT, env=env, check=True)
    print("Building simulator:", " ".join(build))
    subprocess.run(build, cwd=REPO_ROOT, env=env, check=True)
    return 0


def info_cmd(_: argparse.Namespace) -> int:
    """输出打包策略的关键部署元数据。"""
    print("AHAC Go2 deployment package")
    print(f"  policy:     {_rel(POLICY_PATH)}")
    print("  variant:    blind_nolinvel_nokinref")
    print("  run:        ahac_20260828_180341")
    print("  command:    vx=[-1.5, 1.5] m/s, vy=[-1.0, 1.0] m/s, yaw=[-1.5, 1.5] rad/s")
    print("  policy Hz:  50 Hz")
    print("  motor Hz:   500 Hz")
    print("  gains:      loaded from .npz unless --kp/--kd overrides are supplied")
    return 0


def run_cmd(args: argparse.Namespace) -> int:
    """拼出部署命令，并在非 dry-run 模式下启动实机控制器。"""
    policy_path = Path(args.policy) if args.policy else POLICY_PATH
    policy_path = policy_path if policy_path.is_absolute() else REPO_ROOT / policy_path
    if not policy_path.exists():
        print(f"Missing policy: {_rel(policy_path)}", file=sys.stderr)
        return 2

    ros2 = args.command_source == "ros2"
    build_dir = _build_dir(args.build_dir, ros2)
    exe = _exe_path(build_dir)
    if not exe.exists():
        if args.dry_run:
            print(f"Warning: executable is not built yet: {_rel(exe)}", file=sys.stderr)
        elif args.build_if_missing:
            build_args = argparse.Namespace(
                build_dir=str(build_dir),
                ros2=ros2,
                stand_example=True,
                jobs=args.jobs,
            )
            build_cmd(build_args)
            if not exe.exists():
                print(f"Build finished but executable is missing: {_rel(exe)}", file=sys.stderr)
                return 2
        else:
            print(f"Missing executable: {_rel(exe)}", file=sys.stderr)
            print("Build first with: python -m mjx_deploy.ahac_go2_deploy build", file=sys.stderr)
            return 2

    cmd = [
        str(exe),
        "--policy",
        _rel(policy_path),
        "--interface",
        args.interface,
        "--domain-id",
        str(args.domain_id),
        "--command-source",
        args.command_source,
    ]
    if args.command_source == "ros2":
        cmd.extend(["--cmd-topic", args.cmd_topic])
    if args.kp is not None:
        cmd.extend(["--kp", str(args.kp)])
    if args.kd is not None:
        cmd.extend(["--kd", str(args.kd)])

    print("Policy: AHAC adaptive horizon, ahac_20260828_180341")
    print("Command:")
    separator = " \\" + "\n  "
    print("  " + separator.join(cmd))
    print("Controls: Enter=stand, Enter=walk, w/s=vx, a/d=vy, q/e=yaw, 0=zero, x=estop, Ctrl-C=sit down")
    if args.dry_run:
        return 0
    sys.stdout.flush()
    return _run_child(cmd)


def sim_cmd(args: argparse.Namespace) -> int:
    """启动 C++ MuJoCo + SDK2 仿真低层。"""
    scene_path = Path(args.scene) if args.scene else SIM_SCENE_PATH
    scene_path = scene_path if scene_path.is_absolute() else REPO_ROOT / scene_path
    if not scene_path.exists():
        print(f"Missing MuJoCo scene: {_rel(scene_path)}", file=sys.stderr)
        return 2

    build_dir = _sim_build_dir(args.build_dir)
    exe = _sim_exe_path(build_dir)
    if not exe.exists():
        if args.dry_run:
            print(f"Warning: simulator is not built yet: {_rel(exe)}", file=sys.stderr)
        elif args.build_if_missing:
            build_args = argparse.Namespace(
                build_dir=str(build_dir),
                mujoco_root=args.mujoco_root,
                cmake_prefix_path=args.cmake_prefix_path,
                jobs=args.jobs,
            )
            build_sim_cmd(build_args)
            if not exe.exists():
                print(f"Build finished but simulator is missing: {_rel(exe)}", file=sys.stderr)
                return 2
        else:
            print(f"Missing simulator: {_rel(exe)}", file=sys.stderr)
            print("Build first with: python -m mjx_deploy.ahac_go2_deploy build-sim", file=sys.stderr)
            return 2

    cmd = [
        str(exe),
        "--scene",
        _rel(scene_path),
        "--interface",
        args.interface,
        "--domain-id",
        str(args.domain_id),
        "--sim-dt",
        str(args.sim_dt),
        "--viewer-dt",
        str(args.viewer_dt),
        "--cmd-timeout",
        str(args.cmd_timeout),
        "--status-period",
        str(args.status_period),
        "--initial-pose",
        args.initial_pose,
        "--idle-target",
        args.idle_target,
        "--control-mode",
        args.control_mode,
    ]
    if args.headless:
        cmd.append("--headless")
    if args.max_time > 0.0:
        cmd.extend(["--max-time", str(args.max_time)])

    print("AHAC MuJoCo + SDK2 simulator command:")
    separator = " \\" + "\n  "
    print("  " + separator.join(cmd))
    print("Simulator publishes rt/lowstate and subscribes rt/lowcmd. Use lo/domain 1 for local tests.")
    if args.dry_run:
        return 0
    sys.stdout.flush()
    return _run_child(cmd)


def stand_cmd(args: argparse.Namespace) -> int:
    """运行宇树官方 Go2 起立示例，用于低层通信测试。"""
    build_dir = _build_dir(args.build_dir, False)
    exe = build_dir / "go2_stand_example"
    if not exe.exists():
        if args.dry_run:
            print(f"Warning: executable is not built yet: {_rel(exe)}", file=sys.stderr)
        elif args.build_if_missing:
            build_args = argparse.Namespace(
                build_dir=str(build_dir),
                ros2=False,
                stand_example=True,
                jobs=args.jobs,
            )
            build_cmd(build_args)
        else:
            print(f"Missing executable: {_rel(exe)}", file=sys.stderr)
            print("Build first with: python -m mjx_deploy.ahac_go2_deploy build", file=sys.stderr)
            return 2

    cmd = [str(exe), args.interface]
    print("Unitree stand example command:")
    print("  " + " ".join(cmd))
    if args.dry_run:
        return 0
    sys.stdout.flush()
    return _run_child(cmd)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build and run the AHAC Go2 deployment package.")
    sub = parser.add_subparsers(dest="command", required=True)

    p_info = sub.add_parser("info", help="Show packaged policy metadata")
    p_info.set_defaults(func=info_cmd)

    p_build = sub.add_parser("build", help="Build C++ deploy controller and stand example")
    p_build.add_argument("--ros2", action="store_true", help="Enable ROS2 command-topic support")
    p_build.add_argument("--build-dir", help="CMake build directory; defaults to build or build-ros2")
    p_build.add_argument("--jobs", type=int, default=os.cpu_count(), help="Parallel build jobs")
    p_build.add_argument("--no-stand-example", dest="stand_example", action="store_false", help="Skip go2_stand_example target")
    p_build.set_defaults(func=build_cmd, stand_example=True)

    p_build_sim = sub.add_parser("build-sim", help="Build C++ MuJoCo + SDK2 simulator")
    p_build_sim.add_argument("--build-dir", help="CMake build directory; defaults to build-sim")
    p_build_sim.add_argument("--mujoco-root", help="MuJoCo C package root; defaults to MUJOCO_ROOT or Python mujoco package")
    p_build_sim.add_argument("--cmake-prefix-path", help="Extra CMake prefix path, for example /usr/local")
    p_build_sim.add_argument("--jobs", type=int, default=os.cpu_count(), help="Parallel build jobs")
    p_build_sim.set_defaults(func=build_sim_cmd)

    p_run = sub.add_parser("run", help="Run the AHAC policy on Go2")
    p_run.add_argument("--policy", help="Override packaged .npz policy path")
    p_run.add_argument("--interface", default="eth0", help="DDS network interface; use lo for simulator")
    p_run.add_argument("--domain-id", type=int, default=0, help="DDS domain ID")
    p_run.add_argument("--command-source", choices=["terminal", "wireless", "ros2"], default="terminal")
    p_run.add_argument("--cmd-topic", default="/velocity_command", help="ROS2 PointStamped command topic")
    p_run.add_argument("--build-dir", help="CMake build directory; defaults to build or build-ros2")
    p_run.add_argument("--build-if-missing", action="store_true", help="Run CMake if executable is missing")
    p_run.add_argument("--jobs", type=int, default=os.cpu_count(), help="Parallel build jobs")
    p_run.add_argument("--kp", type=float, help="Override walking kp")
    p_run.add_argument("--kd", type=float, help="Override walking kd")
    p_run.add_argument("--dry-run", action="store_true", help="Print launch command without sending motor commands")
    p_run.set_defaults(func=run_cmd)

    p_sim = sub.add_parser("sim", help="Run local MuJoCo low-level simulator for AHAC deployment")
    p_sim.add_argument("--scene", default=str(SIM_SCENE_PATH.relative_to(REPO_ROOT)), help="MuJoCo scene XML")
    p_sim.add_argument("--interface", default="lo", help="DDS loopback interface")
    p_sim.add_argument("--domain-id", type=int, default=1, help="DDS domain ID for simulator")
    p_sim.add_argument("--build-dir", help="CMake build directory; defaults to build-sim")
    p_sim.add_argument("--build-if-missing", action="store_true", help="Build simulator if executable is missing")
    p_sim.add_argument("--mujoco-root", help="MuJoCo C package root used when building")
    p_sim.add_argument("--cmake-prefix-path", help="Extra CMake prefix path used when building")
    p_sim.add_argument("--jobs", type=int, default=os.cpu_count(), help="Parallel build jobs")
    p_sim.add_argument("--sim-dt", type=float, default=0.002, help="MuJoCo integration timestep")
    p_sim.add_argument("--viewer-dt", type=float, default=0.02, help="Viewer refresh interval")
    p_sim.add_argument("--cmd-timeout", type=float, default=0.25, help="LowCmd stale timeout")
    p_sim.add_argument("--status-period", type=float, default=1.0, help="Console status print interval")
    p_sim.add_argument(
        "--initial-pose",
        choices=["home", "crouch", "prone"],
        default="prone",
        help="Initial robot pose; default is prone",
    )
    p_sim.add_argument(
        "--idle-target",
        choices=["initial", "home"],
        default="initial",
        help="Pose held before the first valid LowCmd; default is initial",
    )
    p_sim.add_argument("--control-mode", choices=["auto", "position_servo", "pd_torque"], default="auto")
    p_sim.add_argument("--headless", action="store_true", help="Run without MuJoCo viewer")
    p_sim.add_argument("--max-time", type=float, default=0.0, help="Maximum simulator runtime in seconds")
    p_sim.add_argument("--dry-run", action="store_true", help="Print simulator command only")
    p_sim.set_defaults(func=sim_cmd)

    p_stand = sub.add_parser("stand-example", help="Run Unitree's official Go2 stand example")
    p_stand.add_argument("--interface", default="eth0", help="DDS network interface")
    p_stand.add_argument("--build-dir", default="build", help="CMake build directory")
    p_stand.add_argument("--build-if-missing", action="store_true")
    p_stand.add_argument("--jobs", type=int, default=os.cpu_count(), help="Parallel build jobs")
    p_stand.add_argument("--dry-run", action="store_true")
    p_stand.set_defaults(func=stand_cmd)

    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
