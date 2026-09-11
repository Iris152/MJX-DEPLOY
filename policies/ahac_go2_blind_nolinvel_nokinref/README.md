# AHAC Go2 Policy

Packaged policy:

```text
policy_best_tracking_deploy.npz
```

Source training run before packaging:

```text
ahac_20260828_180341
```

Deployment metadata:

- Variant: `blind_nolinvel_nokinref`
- Actor: `450 -> 512 -> 256 -> 128 -> 12`
- Observation history: `10 x 45`
- Policy rate: `50 Hz`, exported `dt = 0.02 s`
- Command range: `vx=[-1.5, 1.5]`, `vy=[-1.0, 1.0]`, `yaw=[-1.5, 1.5]`
- SHA256: `4ecc2ab60b285fe53f341484908c93bd90a1fb4c8fb3c32d183ac2a42494de24`
