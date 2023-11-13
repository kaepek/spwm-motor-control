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

- [Arduino Code](./lib/peripheral/speed-control/open-loop/arduino-uno/arduino-uno.ino)
- [Teensy40 instructions](./lib/peripheral/speed-control/open-loop/teensy-40/README.md)

## Closed loop (AS5147P rotary encoder , Teensy 4.0 microcontroller, l6234 motor driver)

- [Detailed instructions](./lib/peripheral/speed-control/closed-loop/AS5147P/teensy40/README.md)

# Position control:

## Open loop code (no feedback)
- [Teensy40 instructions](./lib/peripheral/position-control/open-loop/teensy-40/README.md)

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
- [Anti-cogging-algorithm](https://www.modlabupenn.org/wp-content/uploads/piccoli_matthew_anticogging_torque_ripple_suppression_modeling_and_parameter_selection.pdf)

# How to prepare the Teensy40 platform:
- Install [Arduino IDE v1.8.19](https://www.arduino.cc/en/software)
- Install [Teensyduino v2.1.0](https://www.pjrc.com/teensy/teensyduino.html)
- Install [TeensyTimerTool library](https://github.com/luni64/TeensyTimerTool) by opening up Arduino IDE click the "Sketch" menu item, go down to "Include Library" and click "Manage Libraries...". Enter in the filter you search the string "TeensyTimerTool", select the correct version "Version 1.3.0" and click the install button.

# General dependancies:

- [Arduino.h](https://github.com/arduino/ArduinoCore-avr)
- [imxrt.h](https://github.com/PaulStoffregen/cores/tree/master)
- [TeensyTimerTool](https://github.com/luni64/TeensyTimerTool/blob/master/LICENSE)
- [RxJs](https://github.com/ReactiveX/rxjs/blob/master/LICENSE.txt)
- [TS-Node](https://github.com/TypeStrong/ts-node/blob/main/LICENSE)
- [Regression](https://github.com/tom-alexander/regression-js)
- [Regression types](https://www.npmjs.com/package/@types/regression)