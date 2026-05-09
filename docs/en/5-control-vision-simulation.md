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
person_left_seat -> idle -> RGB(30, 30, 80)
gesture_left -> headshake -> RGB(90, 150, 255)
gesture_right -> scanning -> RGB(90, 150, 255)
gesture_up -> curious -> RGB(180, 220, 255)
gesture_down -> idle -> RGB(180, 220, 255)
posture_studying -> idle -> RGB(255, 235, 190)
looking_at_lamp -> nod -> RGB(255, 210, 130)
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
ailamp agent-tools-test --event posture_studying --apply
ailamp agent-tools-test --event person_right --offset 0.6 --request "follow me" --apply
ailamp agent
```

`vision-demo` captures one frame and prints the detected event, motion, and LED color.

`vision-loop` is the continuous runtime bridge:

```text
Arducam UB0234 -> YOLO nano + YOLO pose -> VisionEvent -> BehaviorService -> ST3215 + Pico LED
```

By default it only prints results and writes `outputs/vision_state.json`. Use `--with-outputs` on the Jetson after `led-test` and `motor-test` pass.

## AI Decision Layer

AILamp uses a local decision layer between vision/voice input and hardware output:

```text
VisionEvent + optional voice request -> DecisionService -> recording OR joint deltas + LED
```

Continuous tracking decisions:

```text
person_left / gesture_left -> base_yaw negative delta
person_right / gesture_right -> base_yaw positive delta
person_close -> wrist_pitch negative delta, lamp head leans back
person_far -> wrist_pitch positive delta, lamp head leans forward
gesture_up -> wrist_pitch positive delta
gesture_down -> wrist_pitch negative delta
```

Voice intent can override the default visual response:

```text
"focus" / "study" / "专注" / "学习" -> idle + focus warm light
"nod" / "点头" -> nod
"follow" / "track" / "跟随" / "看着我" -> continuous tracking
"idle" / "rest" / "休息" -> idle
```

Joint delta commands are clipped by a safety limiter before they are sent to the ST3215 layer.

Gesture and posture support is heuristic in this version:

- hand left/right/up/down changes the lamp behavior toward the corresponding position response
- head down/study posture enables focus lighting
- looking up at the lamp triggers a nod
- close person still takes priority and triggers `shy`
- leaving the seat transitions to `idle`

Run `vision-loop` alongside `agent` when you want the AI tools to use live camera state. The LiveKit/OpenAI agent reads `outputs/vision_state.json` and exposes tools to:

- describe available tools
- decide a response from vision plus voice intent
- read the current vision state
- suggest the matching motion and LED color
- apply the current vision behavior to the physical lamp
- list available recordings
- play a named recording
- set a custom LED color

`agent-tools-test` exercises the same AI-callable methods without requiring LiveKit or hardware. It uses dry-run outputs by default:

```bash
ailamp agent-tools-test --event person_close --apply --recording nod --color 1 2 3
ailamp agent-tools-test --event posture_studying --apply
ailamp agent-tools-test --event person_right --offset 0.6 --request "follow me" --apply
```
