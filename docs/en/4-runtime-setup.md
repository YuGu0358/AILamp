# 4. Runtime Setup

## Install

```bash
cd ~/projects/AILamp
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[hardware,simulation,voice,test]"
```

Jetson Nano 4GB API-hybrid install:

```bash
cd ~/projects/AILamp
python3.11 -m venv .venv
source .venv/bin/activate
pip install -e ".[nano,test]"
export OPENAI_API_KEY=...
```

Do not install the local YOLO pose stack, MuJoCo, or local large models on the Jetson Nano profile.

## Configure

Edit the active profile config only if Linux assigns different device paths: `config/hardware.toml` for Orin Nano Super, or `config/hardware.jetson-nano.toml` for Jetson Nano 4GB.

```bash
ailamp hardware-check
ailamp hardware-check --include-devices
ailamp --config config/hardware.jetson-nano.toml hardware-check
ailamp --config config/hardware.jetson-nano.toml hardware-check --include-devices
```

## Pico WH Firmware

Copy `firmware/pico_led_controller/code.py` to the CircuitPython drive on the Pico WH.

Pico WH requirements:

```text
CircuitPython 9.x for Raspberry Pi Pico WH
adafruit-circuitpython-neopixel library in /lib
NeoMatrix data line on GP0
TXS0108E level shifter between Pico GP0 and NeoMatrix DIN
```

Serial protocol:

```text
PING
CLEAR
SOLID r g b
BRIGHTNESS value
PIXELS r,g,b;r,g,b
```
