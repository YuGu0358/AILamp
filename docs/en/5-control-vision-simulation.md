# 5. Control, Vision, and Simulation

## Simulation

```bash
ailamp sim-demo
ailamp sim-viewer --render outputs/ailamp.png
```

The simulation scene is `simulation/ailamp_scene.xml`. It includes the LeLamp robot, a virtual person target, and a simulation camera.

## Vision Events

```text
no_person -> scanning -> RGB(30, 30, 80)
person_left -> look_left -> RGB(80, 120, 255)
person_center -> nod -> RGB(255, 180, 80)
person_right -> look_right -> RGB(80, 120, 255)
person_close -> shy -> RGB(255, 80, 120)
person_far -> curious -> RGB(80, 255, 160)
```

## Hardware Demo

```bash
ailamp camera-test
ailamp audio-test
ailamp led-test
ailamp vision-demo
ailamp agent
```

