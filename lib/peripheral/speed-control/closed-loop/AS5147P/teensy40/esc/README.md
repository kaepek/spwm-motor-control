# ESC sinusoidal

- [ESC sinusoidal code](./esc.ino)

## Usage instructions

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/blob/main/README.md#spwm-procedure) see the ESC sinusoidal section. 
1. In the `ESC sinusodial code` ino update the `MOTOR_CONFIG` given the calibration fit read of a graph file `./[calibration-dir]/calibration-data/zc_reconstruction_[combined_id].png`, the following must be set:
    - `MOTOR_CONFIG_CW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_NUMBER_OF_POLES`
2. Load the `ESC sinusoidal` code onto the teensy40 microcontroller.
3. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9002,udp network=localhost,9003,udp`
4. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs (can start and restart this program with alternative configs as you desire):
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/all_by_time.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_by_time.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_hz_by_time_no_buffer.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
- Realtime graphing example command: `kaepek-io-graph -a localhost -p 9002 -c ./lib/host/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
- Also one could run the [rotation stats analyser](../README.md#rotation-stats-analyser) with the following command `node ./dist/lib/host/speed-control/programs/rotation-stats.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_by_time.json`
5. Use the keyboard (click the director program terminal session first), ps4 dualshock controller or netsend program to control the ESC.
    - keyboard usage: 
        - `[space]` to start.
        - `x` to stop.
        - `r` to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `w` to increase speed.
        - `s` to decrease speed.
        - `q` to change direction (note thrust must be zero to change direction).
    - dualshock ps4 controller usage: 
        - `options` button to start.
        - `share` button to stop.
        - `playstation` icon middle button to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `r2` to control thrust.
        - `triangle` to change direction (note thrust must be zero to change direction).
    - netsend usage:
        - `kaepek-io-netsend -w start` to start.
        - `kaepek-io-netsend -w stop` to stop.
        - `kaepek-io-netsend -w reset` to reset.
        - `kaepek-io-netsend -w thrustui16 -d <duty_value>` to set duty (`<duty_value>` ranges from 0 -> 65535 [min->max], converted to 0 -> `MAX_DUTY` inside the controller, depending on setting `PWM_WRITE_RESOLUTION` the `MAX_DUTY` is `2^PWM_WRITE_RESOLUTION - 1`, so if `PWM_WRITE_RESOLUTION` is set to 11 then `MAX_DUTY` would be 2047, if then you sent the following command `kaepek-io-netsend -w thrustui16 -d 65535` the controller would apply a duty cycle of 2047).
        - `kaepek-io-netsend -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.

## [Parent Readme](../README.md)