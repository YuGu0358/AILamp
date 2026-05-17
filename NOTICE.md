# NOTICE

AILamp is an integration project derived from the open source LeLamp project by Human Computer Lab.

Copied source assets:

- 3D print files from `https://github.com/humancomputerlab/LeLamp`, local source commit `9650849`.
- MuJoCo / URDF simulation files from `https://github.com/humancomputerlab/LeLamp`, local source commit `9650849`.
- Motion recording CSV files from `https://github.com/humancomputerlab/lelamp_runtime`, local source commit `ee23699`.

The copied upstream project is licensed under GNU GPL v3. See `LICENSE`.

AILamp modifies the upstream simulation by adding `simulation/ailamp_scene.xml`, which includes a virtual person target, simulation camera, and simplified visual geoms for the AILamp hardware adapter kit while retaining the original LeLamp MJCF assets for reference.

AILamp-specific additions include:

- Jetson Orin Nano Super hardware configuration
- Jetson Nano 4GB API-hybrid hardware configuration
- Generated AILamp adapter kit in `3D/AILamp_Adapters/`
- Raspberry Pi Pico WH LED serial firmware
- Arducam UB0234 camera integration
- Seeed ReSpeaker XVF3800 audio integration
- Virtual vision simulation events
- AILamp runtime services and CLI
- Bilingual build documentation
