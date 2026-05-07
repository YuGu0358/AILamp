# 3. Assembly

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
