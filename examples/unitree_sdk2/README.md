# Unitree SDK2 Go2 Stand Example

`go2_stand_example.cpp` is the official Unitree SDK2 Go2 stand example copied
from:

```text
https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_stand_example.cpp
```

It is included here as a low-level DDS sanity test before running the learned
AHAC policy. The example expects one argument: the robot network interface.

```bash
./build/go2_stand_example eth0
```

The wrapper command from this repository is:

```bash
python -m mjx_deploy.ahac_go2_deploy stand-example --interface eth0 --build-if-missing
```

