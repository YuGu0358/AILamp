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

## AILamp Adapter Kit

AILamp v1 preserves the original seven LeLamp `.3mf` files. The Jetson Nano 4GB hardware profile uses reversible adapter parts in `3D/AILamp_Adapters/` instead of cutting into the original base or lamp head.

| Adapter file | Qty | Purpose |
| --- | ---: | --- |
| `AILamp_Jetson_Nano_Base_Tray.3mf` | 1 | External loose-fit tray for the Jetson Nano 4GB developer kit |
| `AILamp_Electronics_Side_Deck.3mf` | 1 | Mounts the Waveshare ST3215 driver and Raspberry Pi Pico WH |
| `AILamp_Head_Camera_Mount.3mf` | 1 | Reversible Arducam UB0234 lamp-head camera mount |
| `AILamp_NeoMatrix_Holder.3mf` | 1 | Holds the Adafruit NeoMatrix behind the diffuser |
| `AILamp_ReSpeaker_External_Mount.3mf` | 1 | External ReSpeaker XVF3800 mount |
| `AILamp_Cable_Clip_6mm.3mf` | 2-4 | USB and signal cable routing |
| `AILamp_Cable_Clip_10mm.3mf` | 2-4 | Power and servo cable routing |

Fit rule: print the first adapter pass slightly loose. PCB pockets include 1.5 mm clearance per side, cable exits include at least 2.5 mm extra width, and retention should use screws, zip ties, or removable straps rather than hard snap-fit pressure.
