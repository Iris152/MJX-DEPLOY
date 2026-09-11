#!/usr/bin/env python3
"""Build and launch the packaged AHAC Go2 deployment policy."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
POLICY_PATH = (
    REPO_ROOT
    / "policies"
    / "ahac_go2_blind_nolinvel_nokinref"
    / "policy_best_tracking_deploy.npz"
)
EXECUTABLE_NAME = "deploy_blind_nolinvel_nokinref"
EXECUTABLE_SUBPATH = Path("cpp") / "deploy_blind_nolinvel_nokinref" / EXECUTABLE_NAME


def _rel(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def _build_dir(value: str | None, ros2: bool) -> Path:
    build_dir = Path(value) if value else Path("build-ros2" if ros2 else "build")
    return build_dir if build_dir.is_absolute() else REPO_ROOT / build_dir


def _exe_path(build_dir: Path) -> Path:
    return build_dir / EXECUTABLE_SUBPATH


def build_cmd(args: argparse.Namespace) -> int:
    build_dir = _build_dir(args.build_dir, args.ros2)
    configure = [
        "cmake",
        "-S",
        ".",
        "-B",
        _rel(build_dir),
        f"-DMJX_DEPLOY_ENABLE_ROS2={'ON' if args.ros2 else 'OFF'}",
        f"-DMJX_DEPLOY_BUILD_STAND_EXAMPLE={'ON' if args.stand_example else 'OFF'}",
    ]
    build = ["cmake", "--build", _rel(build_dir)]
    if args.jobs:
        build.extend(["-j", str(args.jobs)])

    print("Configuring:", " ".join(configure))
    subprocess.run(configure, cwd=REPO_ROOT, check=True)
    print("Building:", " ".join(build))
    subprocess.run(build, cwd=REPO_ROOT, check=True)
    return 0


def info_cmd(_: argparse.Namespace) -> int:
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
    return subprocess.run(cmd, cwd=REPO_ROOT).returncode


def stand_cmd(args: argparse.Namespace) -> int:
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
    return subprocess.run(cmd, cwd=REPO_ROOT).returncode


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

