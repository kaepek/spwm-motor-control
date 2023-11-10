# Closed loop code (rotary encoder AS5147P)

Current support platforms Teensy40 with a AS5147P digital rotary encoder with a L6234 driver circuit only.

## Choose an ESC to evaluate

### Sinsusoidal model ESC

- [ESC sinusoidal](./esc/README.md)
- [ESC sinusoidal PID](./esc-pid/README.md)
- [ESC sinusoidal Anti-Cogging](./esc-ac/README.md)
- [ESC sinusoidal Anti-Cogging PID](./esc-pid-ac/README.md)

### Direct fit model ESC

- [ESC direct](./esc-direct/README.md)
- [ESC direct PID](./esc-direct-ac/README.md)
- [ESC direct Anti-Cogging](./esc-direct-ac/README.md)
- [ESC direct Anti-Cogging PID](./esc-direct-pid-ac/README.md)

## Step Change Analysis

A program to perform a step change analysis and get PID values.

### Usage instructions

1. First load one of the following ESC's onto the Teensy 4.0 microcontroller. Note that for each of these a different step change analysis would be create and they are not interchangable. Also not that the analysis is dependant on the motor load (in the standard use case a fixed mass), if the motor load has changed the step-change analysis will no longer be relevant.
    - [ESC sinusoidal](./esc/README.md)
    - [ESC sinusoidal anti-cogging](./esc-ac/README.md)
    - [ESC direct](./esc-direct/README.md)
    - [ESC direct anti-cogging](./esc-direct-ac/README.md)
2. Configure the motor parameters as per the specific sections configuration instructions.
3. Start the director: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp network=localhost,9002,udp network=localhost,9003,udp`.
3. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs (can start and restart this program with alternative configs as you desire):
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/all_by_time.json`.
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_by_time.json`.
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json`.
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_hz_by_time_no_buffer.json`.
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`.
- Realtime graphing example command: `kaepek-io-graph -a localhost -p 9002 -c ./lib/host/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
- One could also run the [rotation stats analyser](#rotation-stats-analyser) with the following command `node ./dist/lib/host/speed-control/programs/rotation-stats.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_by_time.json`
4. Run the step change analysis program: `node ./dist/lib/host/speed-control/programs/step-change.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json --duty_cap_multiplier 0.3 --output_data_file ./calibration-data/<step-change-output-file-name>.json`.
- Example command: `node ./dist/lib/host/speed-control/programs/step-change.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json --duty_cap_multiplier 0.3 --output_data_file ./calibration-data/tarot-4006-esc-direct-step-change.json`.
5. After the step change program has finished graph the step change output: `kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.detections.cw.csv -c ./lib/host/speed-control/graph_configs/step_change.json && kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.detections.ccw.csv -c ./lib/host/speed-control/graph_configs/step_change.json`
- Example command: `kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-direct-step-change.detections.cw.csv -c ./lib/host/speed-control/graph_configs/step_change.json && kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-direct-step-change.detections.ccw.csv -c ./lib/host/speed-control/graph_configs/step_change.json`
6. Graph the step change regression analysis model fit: `kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.regression.cw.csv -c ./calibration-data/<step-change-output-file-name>.regression.cw.config.json && kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.regression.ccw.csv -c ./calibration-data/<step-change-output-file-name>.regression.ccw.config.json` One can find the best model (linear or power law supported) fit for a particular duty range (--duty_begin and --duty_end) by looking for the r^2 fitness values from the least squares regression, and read the values which can be inputted into the controller. For a power law model one can read off the `10^c` value for each direction which corresponds to the `power law set point divisor` words and one can read off the `m` value for each direction which corresponds to the `power law root` words. For the linear model one can read of the `1/m` value for each direction which corresponds to the `linear set point coefficient` words and one can read of the `-c/m` which corresponds to the `linear bias term`. These values can then be sent to a running controller by their respective words `powerlawrootccwf32 powerlawrootcwf32 powerlawsetpointdivisorccwf32 powerlawsetpointdivisorcwf32 linearbiascwf32 linearbiasccwf32 linearsetpointcoefficientcwF32 linearsetpointcoefficientccwF32` or can be programmed persistantly into a microcontroller by setting them in the `PID_CONFIG` struct within their respective ino file.
- Example command: `kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-direct-step-change.regression.cw.csv -c ./calibration-data/tarot-4006-esc-direct-step-change.regression.cw.config.json && kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-direct-step-change.regression.ccw.csv -c ./calibration-data/tarot-4006-esc-direct-step-change.regression.ccw.config.json`
7. Optionally inspect the output data files:
    - `./calibration-data/<step-change-output-file-name>.json`
    - `./calibration-data/<step-change-output-file-name>.stats.json`
    - `./calibration-data/<step-change-output-file-name>.detections.cw.csv`
    - `./calibration-data/<step-change-output-file-name>.detections.ccw.csv`
    - `./calibration-data/<step-change-output-file-name>.detections.cw.csv.html`
    - `./calibration-data/<step-change-output-file-name>.detections.ccw.csv.html`
    - `./calibration-data/<step-change-output-file-name>.steady.cw.csv`
    - `./calibration-data/<step-change-output-file-name>.steady.ccw.csv`
    - `./calibration-data/<step-change-output-file-name>.transition.cw.csv`
    - `./calibration-data/<step-change-output-file-name>.transition.ccw.csv`
    - `./calibration-data/<step-change-output-file-name>.regression.ccw.csv`
    - `./calibration-data/<step-change-output-file-name>.regression.cw.config.json`
    - `./calibration-data/<step-change-output-file-name>.regression.ccw.csv.html`
    - `./calibration-data/<step-change-output-file-name>.regression.cw.csv`
    - `./calibration-data/<step-change-output-file-name>.regression.ccw.config.json`
    - `./calibration-data/<step-change-output-file-name>.regression.cw.csv.html`

### Example of the whole chain of analysis and graphing

1. Start the director in one terminal session:
    - `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp network=localhost,9002,udp network=localhost,9003,udp`
2. Start the realtime graph and rotation analyser (need an additional two terminal sessions):
    - `kaepek-io-graph -a localhost -p 9002 -c ./lib/host/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
    - `node ./dist/lib/host/speed-control/programs/rotation-stats.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_by_time.json`
2. Run the logging and analysis (for the slow use case) in another additional terminal session:
    - `node ./dist/lib/host/speed-control/programs/step-change.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json --duty_cap_multiplier 0.3 --duty_begin idle --duty_end 400 --output_data_file ./calibration-data/<step-change-output-file-name>.json && kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.detections.cw.csv -c ./lib/host/speed-control/graph_configs/step_change.json && kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.detections.ccw.csv -c ./lib/host/speed-control/graph_configs/step_change.json && kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.regression.cw.csv -c ./calibration-data/<step-change-output-file-name>.regression.cw.config.json && kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.regression.ccw.csv -c ./calibration-data/<step-change-output-file-name>.regression.ccw.config.json`

### Note on different models

Depending on the usage differing model fits (power law or linear) maybe provide better or worse performance depending on the motors load and the power range in which it will be utalised. Assuming a fixed load a linear model may provide a better fit at low power ranges / speeds compared to the power law model which may provide a better fit given higher power ranges / speeds.

In order to get the best fit one may configure the step change program differently depending on the desired power range / speed.

- Low speed behaviour
    - For the best low speed model fitting it is important to have some idea of the maximum duty required for a given target speed. This can be experimentally determined by loading one of the following programs `esc sinusoidal`, `esc sinusoidal anti-cogging`, `esc direct`, `esc direct anti-cogging`, then by running the `rotation stats analyser` program with the following command `node ./dist/lib/host/speed-control/programs/rotation-stats.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_by_time.json` and the director with the following command `kaepek-io-director -i keyboard -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9003,udp`. Firstly get focus on the kaepek-io-director terminal session and start the motor controller by hitting the `space` key. One could then increment the thrustui16 value by using the `w` key, when the maximum operating speed expected is achieved (by inspecting the output of the rotation stats program) one could then convert the thrustui16 value to an actual duty (`<maximum-target-duty>`) by using the following formula `(<thrustui16-value>/65534)*<maximum-duty>` where `<maximum-duty>` is the maximum duty supported by the motor controller program, this can be calculated by taking the `PWM_WRITE_RESOLUTION` as read from the relevant esc ino file and calculating the duty as follows `2^PWM_WRITE_RESOLUTION - 1`. Typically the `PWM_WRITE_RESOLUTION` is set to '11' so the `<maximum-duty>` is '2047'.
    - When the `<maximum-target-duty>` is known we could configure the step change analysis program with the following example command `node ./dist/lib/host/speed-control/programs/step-change.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json --duty_cap_multiplier 0.3 --duty_begin idle --duty_end <maximum-target-duty> --output_data_file ./calibration-data/tarot-4006-esc-direct-step-change.json`. Make sure to restart the director with the following command example `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp network=localhost,9003,udp network=localhost,9002,udp`. The step change program would then sweep from the automatically determined idle duty to the user defined `<maximum-target-duty>` in a number of steps and perform the analysis. Typically if maximum speed targetted by the `<maximum-target-duty>` is low (below 2 [Hz]) the linear model is expected to perform best, but check the the `r^2` for each model, the one closest to 1.0 is the best model to choose. One can then load the model values either using the relevant command words or by configuring the `PID_CONFIG` in the relevant pid esc ino file (see the relevant PID esc section of your choice in this readme for additional instruction on how to do this).
- High speed behaviour
    - For the best high speed model fitting one would follow the normal instruction and leave the `--duty_begin` and `--duty_end` flags absent, they default to `--duty_begin start` and `--duty_end 2047` which implies that the step change analyse would speed from the minimum speed required to start a motor from it being motionless to the maximum operating range of the motor. Typically in this higher speed domain the power law model is expected to perform best, but check the the `r^2` for each model, the one closest to 1.0 is the best model to choose. One can then load the model values either using the relevant command words or by configuring the `PID_CONFIG` in the relevant pid esc ino file (see the relevant PID esc section of your choice in this readme for additional instruction on how to do this).

## Rotation stats analyser

This program computes the mean, standard deviation, min, max and range for the velocity and acceleration data coming from the motor controller, useful to understand some metrics of motor performance without having to rely on values read off the graphing system.

1. Run the command choosing the appropriate `--input_config_file`. By default the rotation stats analyser listens on the incoming protocol `upd`, with host address `localhost` and port `9003` so make sure the kaepek-io-direct program has the appropriate output sink flag for this program if you wish to use it, e.g. append `-o network=localhost,9003,upd`. You can configure angular frequency over which data is collected and their statistics analysed by changing the `--n_rotations` argument (which defaults to '1' rotation).

- For PID (with anti cogging or otherwise) use this for example: `node ./dist/lib/host/speed-control/programs/rotation-stats.js --input_config_file ./lib/host/speed-control/graph_configs/pid_by_time.json`
- For ESC-sinusoidal,ESC-direct,ESC sinudoidal anti-cogging and ESC direct anti-cogging use this for example: `node ./dist/lib/host/speed-control/programs/rotation-stats.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_by_time.json`

## [Parent Readme](../../../../../../README.md)