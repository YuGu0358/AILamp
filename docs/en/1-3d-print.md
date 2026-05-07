# 1. 3D Print

AILamp uses the original LeLamp print files in `3D/`.

## CAD Source

Keep the upstream LeLamp mechanical workflow:

- Primary CAD software: OnShape.
- Upstream CAD reference: `https://cad.onshape.com/documents/16c9706360b5ad34f9c8db49/w/2edfa54c83253c120fbc9e58/e/a7196194821d9cfe2842a44a`
- Upstream assembly reference: `https://cad.onshape.com/documents/16c9706360b5ad34f9c8db49/w/2edfa54c83253c120fbc9e58/e/a35eec618cd78ea5d74bf01b`
- Export print files as `.3mf`.
- Export simulation meshes as `.stl` when MuJoCo assets need updating.

Do not switch the primary mechanical workflow to Blender, SolidWorks, Fusion 360, or FreeCAD for AILamp v1.

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
