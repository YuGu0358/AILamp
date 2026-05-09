# 5. Control, Vision, and Simulation

## Simulation

```bash
ailamp sim-demo
ailamp sim-viewer --render outputs/ailamp.png
```

AILamp keeps the upstream LeLamp simulation workflow:

- Simulation engine: MuJoCo.
- Main scene: `simulation/ailamp_scene.xml`.
- Upstream scene kept for reference: `simulation/scene.xml`.
- Robot MJCF: `simulation/robot.xml`.
- Reference URDF: `simulation/robot.urdf`.
- Mesh assets: `simulation/assets/*.stl`.

`simulation/ailamp_scene.xml` includes the LeLamp robot, a virtual person target, and a simulation camera.

## Vision Events

```text
no_person -> scanning -> RGB(30, 30, 80)
person_left -> headshake -> RGB(80, 120, 255)
person_center -> nod -> RGB(255, 180, 80)
person_right -> scanning -> RGB(80, 120, 255)
person_close -> shy -> RGB(255, 80, 120)
person_far -> curious -> RGB(80, 255, 160)
```

## Hardware Demo

```bash
ailamp camera-test
ailamp audio-test
ailamp led-test
ailamp vision-demo
ailamp vision-loop --frames 30
ailamp vision-loop --with-outputs
ailamp agent-tools-test --event person_close --apply
ailamp agent
```

`vision-demo` captures one frame and prints the detected event, motion, and LED color.

`vision-loop` is the continuous runtime bridge:

```text
Arducam UB0234 -> YOLO nano -> VisionEvent -> BehaviorService -> ST3215 + Pico LED
```

By default it only prints results and writes `outputs/vision_state.json`. Use `--with-outputs` on the Jetson after `led-test` and `motor-test` pass.

Run `vision-loop` alongside `agent` when you want the AI tools to use live camera state. The LiveKit/OpenAI agent reads `outputs/vision_state.json` and exposes tools to:

- describe available tools
- read the current vision state
- suggest the matching motion and LED color
- apply the current vision behavior to the physical lamp
- list available recordings
- play a named recording
- set a custom LED color

`agent-tools-test` exercises the same AI-callable methods without requiring LiveKit or hardware. It uses dry-run outputs by default:

```bash
ailamp agent-tools-test --event person_close --apply --recording nod --color 1 2 3
```
