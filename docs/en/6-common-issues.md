# 6. Common Issues

## Device Paths Changed

Run:

```bash
ls /dev/ttyACM* /dev/ttyUSB* /dev/video*
```

Update `config/hardware.toml`.

## LED Does Not Light

- Check `GST60A05-P1J` output.
- Check Pico WH serial port.
- Check TXS0108E direction and ground.
- Check the 330 ohm data resistor.
- Run `ailamp led-test`.

## Servo Does Not Move

- Check `GST120A12-P1J` output.
- Check Waveshare driver serial port.
- Check ST3215 IDs.
- Run `ailamp motor-test`.

## Camera Does Not Open

- Check `/dev/video0`.
- Reduce resolution to 640x480.
- Check Arducam UB0234 USB cable.

