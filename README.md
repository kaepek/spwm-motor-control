# SPWM motor control with L6234 driver

Smooth FOC like control with torque and speed control.

# Information

- [Manual](./resources/sinusoidal-pwm-manual.pdf)

# Install repo

```
git clone --recurse-submodules git@github.com:kaepek/spwm-motor-control.git
npm run build
```

# Speed control:

## Open loop code (no feedback)

- [Arduino Code](./speed-control/open-loop/arduino-uno/arduino-uno.ino)
- [Teensy40 Code](./speed-control/open-loop/teensy-40/teensy-40.ino) (todo fix words)

## Closed loop (AS5147P rotary encoder , Teensy 4.0 microcontroller, l6234 motor driver)

- [Detailed instructions](./speed-control/closed-loop/AS5147P/teensy40/README.md)

# Position control:

## Open loop code (no feedback)
- [Teensy40 Code](./position-control/open-loop/teensy-40/teensy-40.ino) (todo fix words)

## Closed loop code (rotary encoder AS5147P)
- [Todo](./)

# Controller install

- cd external/kaepek-io
- npm install
- npm link

# Credits/Useful links:
- EE design and software enginnering - Jonathan Kelsey
- [Hack-a-day](https://hackaday.io/project/177958-low-power-bldc-driver-board-st-l6234#menu-details)
- [Electro-noobs](https://electronoobs.com/eng_arduino_tut176.php)
- [ST-l6234-three-phase-motor-drive](https://www.st.com/resource/en/application_note/cd00004062-l6234-three-phase-motor-driver-stmicroelectronics.pdf)