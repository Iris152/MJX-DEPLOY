#!/usr/bin/env python3
"""Print compact metadata for an exported deployment .npz policy."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect an exported MJX/Open-DiffLoco deployment policy.")
    parser.add_argument("policy", nargs="?", default="policies/ahac_go2_blind_nolinvel_nokinref/policy_best_tracking_deploy.npz")
    args = parser.parse_args()

    path = Path(args.policy)
    if not path.is_absolute():
        path = Path.cwd() / path
    data = np.load(path, allow_pickle=False)

    n_hidden = int(data["n_hidden"])
    in_dim = data["dense_0_kernel"].shape[0]
    out_dim = data[f"dense_{n_hidden}_kernel"].shape[1]
    hidden = " -> ".join(str(data[f"dense_{i}_kernel"].shape[1]) for i in range(n_hidden))

    print(path)
    print(f"actor: {in_dim} -> {hidden} -> {out_dim}")
    print(f"obs history: {int(data['actor_history_len'])} x {int(data['actor_frame_obs_dim'])}")
    print(f"dt: {float(data['dt']):.4f}s")
    print(f"action_scale: {np.asarray(data['action_scale']).tolist()}")
    print(f"cmd_vel_x_range: {np.asarray(data['cmd_vel_x_range']).tolist()}")
    print(f"cmd_vel_y_range: {np.asarray(data['cmd_vel_y_range']).tolist()}")
    print(f"cmd_yaw_rate_range: {np.asarray(data['cmd_yaw_rate_range']).tolist()}")
    print(f"actuator_kp[0]: {float(data['actuator_kp'][0]):.3f}")
    print(f"actuator_kd[0] + joint_damping[0]: {float(data['actuator_kd'][0] + data['joint_damping'][0]):.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

