# MJX-DEPLOY

Standalone deployment package for the accepted AHAC Go2 policy trained in MJX /
Open-DiffLoco.

The repository includes:

- AHAC deployment policy: `policies/ahac_go2_blind_nolinvel_nokinref/policy_best_tracking_deploy.npz`
- C++ Unitree low-level controller: `cpp/deploy_blind_nolinvel_nokinref`
- Python build/run helpers: `python/mjx_deploy`
- Official Unitree Go2 stand example: `examples/unitree_sdk2/go2_stand_example.cpp`
- Full laptop-to-real-Go2 instructions: `docs/AHAC_GO2_DEPLOYMENT.md`
- Local MuJoCo + SDK2 deployment-flow simulator: `docs/AHAC_MUJOCO_SDK2_SIM_TEST.md`

Quick start on the deployment laptop:

```bash
git clone https://github.com/Iris152/MJX-DEPLOY.git
cd MJX-DEPLOY
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
python -m mjx_deploy.inspect_policy_npz
python -m mjx_deploy.ahac_go2_deploy build
python -m mjx_deploy.ahac_go2_deploy run --interface eth0 --command-source terminal --dry-run
```

Replace `eth0` with the Go2 Ethernet interface from `ip -br link`. Before
running the learned policy, build and run Unitree's stand example:

```bash
python -m mjx_deploy.ahac_go2_deploy stand-example --interface eth0 --build-if-missing
```

Then start AHAC:

```bash
python -m mjx_deploy.ahac_go2_deploy run --interface eth0 --command-source terminal --build-if-missing
```

For local sim-to-deploy validation before touching the real robot, use two
terminals on loopback DDS:

```bash
python -m mjx_deploy.ahac_go2_deploy sim --interface lo --domain-id 1 --build-if-missing
python -m mjx_deploy.ahac_go2_deploy run --interface lo --domain-id 1 --command-source terminal --build-if-missing
```

The simulator opens a MuJoCo viewer and uses C++ `unitree_sdk2` topics, so the
existing AHAC deployment state machine is exercised through `rt/lowstate` and
`rt/lowcmd`.

Keep the robot supported for first tests and keep the physical emergency stop
ready.
