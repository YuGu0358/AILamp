# AILamp

AILamp is a Jetson-based interactive robotic lamp built from the LeLamp mechanical and motion foundation. It integrates MuJoCo simulation, virtual vision, local person detection, ST3215 servo control, Pico WH LED control, USB audio, and OpenAI/LiveKit voice interaction.

## Modeling and Simulation Toolchain

AILamp keeps the same toolchain as the upstream LeLamp repository:

- Mechanical CAD: OnShape.
- 3D print exchange files: `.3mf` files in `3D/`.
- Mesh assets for simulation: `.stl` files in `simulation/assets/`.
- Robot simulation: MuJoCo MJCF XML, with `simulation/ailamp_scene.xml` as the AILamp scene.
- Reference robot description: `simulation/robot.urdf` is kept with the upstream assets.

Do not make Blender, Gazebo, Isaac Sim, SolidWorks, or Fusion 360 the primary project workflow unless the team explicitly changes this toolchain.

## Fixed Hardware BOM

The full non-printed hardware BOM is the structured `[hardware_bom]` section in `config/hardware.toml`; `docs/en/0-prerequisites.md` mirrors it for purchasing. This includes Jetson, storage, ST3215 servos, Waveshare servo driver, both MEAN WELL power supplies, Pico WH, NeoMatrix, TXS0108E, Arducam UB0234, ReSpeaker XVF3800, Seeed 4 ohm 5W speaker, emergency switch, USB cables, servo extensions, DC barrel adapters, WAGO connectors, and wire.

## Project Layout

```text
AILamp/
  3D/                         LeLamp .3mf print files
  simulation/                 MuJoCo MJCF model, STL assets, AILamp scene
  firmware/pico_led_controller/
  ailamp_runtime/ailamp/      Python runtime package
  config/hardware.toml        Fixed hardware and runtime config
  docs/en/                    English build guide
  docs/zh/                    Chinese build guide
  tests/                      Local unit tests
```

## Setup

```bash
git clone https://github.com/YuGu0358/AILamp.git
cd AILamp
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[test]"
```

For Jetson hardware:

```bash
pip install -e ".[hardware,voice]"
```

For physical ST3215 playback and calibration, install the upstream LeLamp runtime beside AILamp:

```bash
cd ..
git clone https://github.com/humancomputerlab/lelamp_runtime.git
cd AILamp
python3 -m pip install -e ../lelamp_runtime
```

For MuJoCo simulation:

```bash
pip install -e ".[simulation]"
```

## CLI

```bash
ailamp hardware-check
ailamp hardware-check --include-devices
ailamp hardware-check --failures-only
ailamp motor-test
ailamp led-test
ailamp camera-test
ailamp audio-test
ailamp sim-demo
ailamp sim-viewer --render outputs/model.png
ailamp vision-demo
ailamp agent
```

## Verification

Run the local project check before pushing changes:

```bash
scripts/verify_local.sh
```

The script runs unit tests, lockfile validation, static hardware checks, wheel build, whitespace checks, and MuJoCo smoke tests when the sibling `mujoco_mcp` environment is present.

`docs/github-actions-ci.yml` contains the matching GitHub Actions template. Copy it to `.github/workflows/ci.yml` when pushing with a GitHub credential that has `workflow` scope.

## Documentation

- English guide: `docs/en/`
- Chinese guide: `docs/zh/`

## Attribution

AILamp copies 3D print files, MuJoCo/URDF simulation assets, and motion recording CSV files from Human Computer Lab's LeLamp projects. See `NOTICE.md` and `LICENSE`.
