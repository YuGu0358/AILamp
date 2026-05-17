# 3. Assembly

AILamp v1 keeps the original LeLamp printed parts intact. Do not cut `LampHead.3mf`, and do not cut, drill, or permanently modify the original LeLamp base, arm, head, or diffuser for the first prototype.

This assembly flow is organized as mechanical fit, wiring, power checks, first boot, and software validation.

## Mechanical Fit

1. Assemble the original LeLamp base, arm, head, and diffuser first.
2. Fit `AILamp_Jetson_Nano_Base_Tray.3mf` beside or behind the base. Confirm the Jetson Nano 4GB developer kit sits loosely, has bottom airflow, and leaves USB, power, camera, and audio cables accessible.
3. Fit `AILamp_Electronics_Side_Deck.3mf` near the base. Confirm the Waveshare ST3215 driver and Pico WH can be removed without bending headers or blocking USB-C, Micro-USB, servo bus, 12V input, or 5V LED wiring.
4. Fit `AILamp_Head_Camera_Mount.3mf` on the lamp head with a removable strap or screw clamp. Keep the Arducam UB0234 lens unobstructed and adjust the final angle with bracket placement or a thin shim.
5. Fit `AILamp_NeoMatrix_Holder.3mf` behind the diffuser. Confirm the NeoMatrix does not press LEDs into the diffuser and the data/power wires exit without sharp bends.
6. Fit `AILamp_ReSpeaker_External_Mount.3mf` on the base or rear side. Do not cover microphone openings or USB access, and keep it away from the servo driver and speaker vibration.
7. Use `AILamp_Cable_Clip_6mm.3mf` for USB/signal cables and `AILamp_Cable_Clip_10mm.3mf` for power or servo bundles.

Fit rule: print the first adapter pass slightly loose. Tighten only after board fit is confirmed, using screws, zip ties, or removable straps.

## Wiring

```text
Jetson USB -> Waveshare Servo Driver with ESP32 -> 5x Waveshare ST3215
Jetson USB -> Raspberry Pi Pico WH -> TXS0108E -> NeoMatrix DIN
Jetson USB -> Arducam UB0234
Jetson USB -> ReSpeaker XVF3800
GST120A12-P1J -> emergency switch -> barrel terminal -> WAGO 221-413 -> servo driver power
GST60A05-P1J -> barrel terminal -> WAGO 221-413 -> NeoMatrix 5V/GND
All signal-linked devices share GND where required by the device interface.
```

Keep the 12V servo power domain and 5V LED power domain physically separated. Route camera/audio USB away from servo power wiring where possible.

## LED Protection

```text
Pico GP0 -> TXS0108E A-side input
Pico 3V3 -> TXS0108E VCCA
NeoMatrix 5V -> TXS0108E VCCB
Common GND -> TXS0108E GND
TXS0108E B-side output -> 330 ohm resistor -> NeoMatrix DIN
TXS0108E OE -> VCCA
GST60A05-P1J 5V/GND -> 1000 uF capacitor -> NeoMatrix 5V/GND
```

## Power Checks

Before any software drives hardware:

1. Confirm the emergency switch disconnects the 12V servo supply.
2. Confirm `GST120A12-P1J` only powers the servo driver and ST3215 chain.
3. Confirm `GST60A05-P1J` only powers the NeoMatrix LED system.
4. Confirm the Pico, TXS0108E, and NeoMatrix share ground.
5. Confirm every ST3215 has the expected ID before installing the lamp in a constrained pose.

Do not run `--with-outputs` before `led-test` and `motor-test` pass.

## First Boot

Jetson Nano API-hybrid main flow:

```bash
ailamp --config config/hardware.jetson-nano.toml runtime-check
ailamp --config config/hardware.jetson-nano.toml hardware-check --include-devices
ailamp --config config/hardware.jetson-nano.toml camera-test
ailamp --config config/hardware.jetson-nano.toml audio-test
ailamp --config config/hardware.jetson-nano.toml led-test
ailamp --config config/hardware.jetson-nano.toml motor-test
```

For the default Orin profile or quick smoke checks, the equivalent legacy commands are `ailamp hardware-check`, `ailamp led-test`, `ailamp vision-demo`, and `ailamp agent`. On Jetson Nano, keep the explicit `--config config/hardware.jetson-nano.toml` flow above as the source of truth.

## Software Validation

Run dry-run software checks before real motion:

```bash
ailamp --config config/hardware.jetson-nano.toml agent-tools-test --event person_right --offset 0.6 --request "follow me"
ailamp --config config/hardware.jetson-nano.toml vision-loop --frames 30
```

After the lamp clears the dry-run path and has enough physical clearance:

```bash
ailamp --config config/hardware.jetson-nano.toml vision-loop --with-outputs
ailamp --config config/hardware.jetson-nano.toml agent --with-outputs
```
