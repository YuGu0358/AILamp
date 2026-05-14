# 3. Assembly

## Mechanical Adapter Installation

AILamp v1 keeps the original LeLamp printed parts intact. Install the new hardware with the reversible adapter kit in `3D/AILamp_Adapters/`.

1. Assemble the original LeLamp base, arm, head, and diffuser first.
2. Mount `AILamp_Jetson_Nano_Base_Tray.3mf` beside or behind the base, leaving airflow under the Jetson Nano 4GB developer kit.
3. Mount `AILamp_Electronics_Side_Deck.3mf` near the base with USB, servo bus, 12V input, and 5V LED wiring accessible.
4. Attach `AILamp_Head_Camera_Mount.3mf` to the lamp head as a reversible bracket. Do not cut `LampHead.3mf` for the first prototype.
5. Install `AILamp_NeoMatrix_Holder.3mf` behind the diffuser with a small air gap, so LEDs do not press directly into the diffuser.
6. Place `AILamp_ReSpeaker_External_Mount.3mf` on the base or rear side, away from the speaker surface and servo driver.
7. Use `AILamp_Cable_Clip_6mm.3mf` for USB/signal cable groups and `AILamp_Cable_Clip_10mm.3mf` for power or servo cable bundles.

Fit rule: leave the adapters slightly loose on the first print. Tighten with screws, zip ties, or removable straps after board fit is confirmed.

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

## First Power-On

1. Power Jetson only.
2. Run `ailamp hardware-check`.
3. Connect Arducam and ReSpeaker, then run camera/audio tests.
4. Flash Pico WH and run `ailamp led-test`.
5. Power ST3215 chain and run servo calibration.
6. Run `ailamp vision-demo`.
7. Run `ailamp agent`.
