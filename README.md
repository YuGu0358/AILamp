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

The non-printed hardware must match `docs/en/0-prerequisites.md` and `config/hardware.toml`.

- NVIDIA Jetson Orin Nano Super Developer Kit, MPN `945-13766-0000-000`
- Samsung 980 NVMe M.2 2280 500GB, `MZ-V8V500B`
- SanDisk Ultra microSDXC 64GB UHS-I
- 5x Waveshare ST3215 Servo, SKU `22414`
- Waveshare Servo Driver with ESP32, SKU `21593`
- MEAN WELL `GST120A12-P1J`, 12V 10A 120W
- Raspberry Pi Pico WH
- Adafruit NeoPixel NeoMatrix 8x8, Product ID `1487`
- MEAN WELL `GST60A05-P1J`, 5V 6A 30W
- Arducam `UB0234` USB UVC camera
- Seeed Studio ReSpeaker XVF3800 USB 4-Mic Array with Case, Product `6490`
- Seeed Studio Mono Enclosed Speaker, 4 ohm 5W

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
cd "/Users/yugu/Documents/New project 4/AILamp"
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[test]"
```

For Jetson hardware:

```bash
pip install -e ".[hardware,voice]"
```

For MuJoCo simulation:

```bash
pip install -e ".[simulation]"
```

## CLI

```bash
ailamp hardware-check
ailamp hardware-check --include-devices
ailamp motor-test
ailamp led-test
ailamp camera-test
ailamp audio-test
ailamp sim-demo
ailamp sim-viewer --render outputs/model.png
ailamp vision-demo
ailamp agent
```

## Documentation

- English guide: `docs/en/`
- Chinese guide: `docs/zh/`
