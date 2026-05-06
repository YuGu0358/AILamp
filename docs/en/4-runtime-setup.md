# 4. Runtime Setup

## Install

```bash
cd ~/projects/AILamp
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[hardware,simulation,voice,test]"
```

## Configure

Edit `config/hardware.toml` only if Linux assigns different device paths.

```bash
ailamp hardware-check
ailamp hardware-check --include-devices
```

## Pico WH Firmware

Copy `firmware/pico_led_controller/code.py` to the CircuitPython drive on the Pico WH.

Serial protocol:

```text
PING
CLEAR
SOLID r g b
BRIGHTNESS value
PIXELS r,g,b;r,g,b
```

