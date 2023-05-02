# SPWM motor control with L6234 driver

Smooth FOC like control with torque and speed control.

# Information

- [Manual](./resources/sinusoidal-pwm-manual.pdf)

# Install repo

```
git clone --recurse-submodules git@github.com:kaepek/spwm-motor-control.git
```

# Code

- [Arduino Code](./arduino-uno/arduino-uno.ino)
- [Teensy Code](./teensy-40/teensy-40.ino)

# Controller install

- cd kaepek-io
- npm install
- npm link

# IO Controller usage

## Run dualshock to serialport with torque-delay-direction profile (for testing with hardware).
```
kaepek-torque-delay-direction-dualshock-serial
```

## Run dualshock to console with torque-delay-direction profile (for testing without hardware).
```
kaepek-torque-delay-direction-dualshock-console
```

# Credits/Useful links:
- EE design and software enginnering - Jonathan Kelsey
- [Hack-a-day](https://hackaday.io/project/177958-low-power-bldc-driver-board-st-l6234#menu-details)
- [Electro-noobs](https://electronoobs.com/eng_arduino_tut176.php)
- [ST-l6234-three-phase-motor-drive](https://www.st.com/resource/en/application_note/cd00004062-l6234-three-phase-motor-driver-stmicroelectronics.pdf)