# 1. 3D Print

AILamp uses the original LeLamp print files in `3D/`.

## Print Counts

| File | Qty |
| --- | ---: |
| `LampBase.3mf` | 1 |
| `LampBase - Cover.3mf` | 1 |
| `LampArm (Base-Elbow).3mf` | 1 |
| `LampArm (Elbow-Wrist).3mf` | 1 |
| `LampArm (Pitch).3mf` | 2 |
| `LampHead.3mf` | 1 |
| `LampHead - Diffuser.3mf` | 1 |

## AILamp Mechanical Changes

- Add an Arducam UB0234 opening in `LampHead.3mf`.
- Add a camera bracket and strain relief for the short USB cable.
- Reserve base routing for the Waveshare servo driver, Pico WH, DC barrel terminals, and WAGO 221-413 connectors.
- Use an external or lower-layer Jetson mounting plate if the stock base cannot contain the Jetson and cooling stack.

