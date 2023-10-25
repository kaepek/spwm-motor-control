## Closed loop code (rotary encoder AS5147P)

Current support platforms Teensy40 with a AS5147P digital rotary encoder with a L6234 driver circuit only.

### ESC sinusoidal

- [ESC sinusoidal code](./esc/esc.ino)

#### Usage instructions

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/tree/FEATURES/new-documentation#spwm-procedure) see the ESC sinusoidal section. 
1. In the `ESC sinusodial code` ino update the `MOTOR_CONFIG` given the calibration fit read of a graph file `./[calibration-dir]/calibration-data/zc_reconstruction_[combined_id].png`, the following must be set:
    - `MOTOR_CONFIG_CW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_NUMBER_OF_POLES`
2. Load the `ESC sinusoidal` code onto the teensy40 microcontroller.
3. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9002,udp`
4. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs (can start and restart this program with alternative configs as you desire):
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/all_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time_no_buffer.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/`
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
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w start` to start.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w stop` to stop.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w reset` to reset.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w thrustui16 -d <duty_value>` to set duty (`<duty_value>` ranges from 0 -> 65535 [min->max], converted to 0 -> `MAX_DUTY` inside the controller, depending on setting `PWM_WRITE_RESOLUTION` the `MAX_DUTY` is `2^PWM_WRITE_RESOLUTION - 1`, so if `PWM_WRITE_RESOLUTION` is set to 11 then `MAX_DUTY` would be 2047, if then you sent the following command `kaepek-io-netsend -h localhost -p 9000 -n udp -w thrustui16 -d 65535` the controller would apply a duty cycle of 2047).
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.

### ESC sinusoidal pid

#### Usage instructions

- [ESC sinusoidal pid code](./esc-pid/esc-pid.ino)

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/tree/FEATURES/new-documentation#spwm-procedure) see the ESC sinusoidal section. 
1. In the `ESC sinusodial pid code` ino update the `MOTOR_CONFIG` given the calibration fit read of a graph file `./[calibration-dir]/calibration-data/zc_reconstruction_[combined_id].png`, the following must be set:
    - `MOTOR_CONFIG_CW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_NUMBER_OF_POLES`
2. Load the code onto the teensy40 microcontroller.
3. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c directionui8 start stop reset setpointf32 proportionalf32 integralf32 derivativef32 powerlawrootccwf32 powerlawrootcwf32 powerlawsetpointdivisorccwf32 powerlawsetpointdivisorcwf32 linearbiascwf32 linearbiasccwf32 linearsetpointcoefficientcwF32 linearsetpointcoefficientccwF32  -p serial console -o network=localhost,9002,udp`
4. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try the following config:
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/pid.json`
5. Use the keyboard (click the director program terminal session first), ps4 dualshock controller or netsend program to control the ESC. Note you must use the netsend program to update PID values.
    - keyboard usage: 
        - `[space]` to start.
        - `x` to stop.
        - `r` to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `q` to change direction (note thrust must be zero to change direction).
    - dualshock ps4 controller usage: 
        - `options` button to start.
        - `share` button to stop.
        - `playstation` icon middle button to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `triangle` to change direction (note thrust must be zero to change direction).
    - netsend usage:
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w start` to start.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w stop` to stop.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w reset` to reset.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w setpointf32 -d 1.3` to set the PID setpoint velocity to 1.3 `[Hz]`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d <proportional-coefficient>` to set the PID proportional-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d 1` sets the PID proportional-coefficient to a value of `1`.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w integralf32 -d <integral-coefficient>` to set the PID integral-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w integralf32 -d 20` sets the PID integral-coefficient to a value of `20`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w derivativef32 -d <derivative-coefficient>` to set the PID derivative-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w derivativef32 -d 0.04` sets the PID derivative-coefficient to a value of `0.04`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootcwf32 -d <power-law-root-cw>` to set the PID power law root as determined from the step change regression analysis (power law m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootcwf32 -d 1.358356`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootccwf32 -d <power-law-root-ccw>` to set the PID power law root as determined from the step change regression analysis (power law m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootccwf32 -d 1.378703`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorcwf32 -d <power-law-set-point-divisor-cw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorcwf32 -d 85.08715447157786`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorccwf32 -d <power-law-set-point-divisor-ccw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorccwf32 -d 87.27482768135731`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiascwf32 -d <linear-bias-cw>` to set the PID linear bias. As determined from the step change regression analysis (linear model -c/m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiascwf32 -d 0.022338094680920625`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiasccwf32 -d <linear-bias-ccw>` to set the PID linear bias as determined from the step change regression analysis (linear model -c/m value )for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiasccwf32 -d 0.022384353255695207`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientcwF32 -d <linear-setpoint-coefficient-cw>` to set the PID linear set point coefficient. As determined from the step change regression analysis (linear model 1/m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientcwF32 -d 0.02064702106741506`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientccwF32 -d <linear-setpoint-coefficient-ccw>` to set the PID linear set point coefficient. As determined from the step change regression analysis (linear model 1/m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientccwF32 -d 0.020612730310931287`

### ESC sinusoidal anti-cogging

- [ESC sinusoidal anti-cogging code](./esc-ac/esc-ac.ino)

#### Usage instructions

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/tree/FEATURES/new-documentation#spwm-procedure) see the ESC sinusoidal section. 
1. In the `ESC sinusodial code` ino update the `MOTOR_CONFIG` given the calibration fit read of a graph file `./[calibration-dir]/calibration-data/zc_reconstruction_[combined_id].png`, the following must be set:
    - `MOTOR_CONFIG_CW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_NUMBER_OF_POLES`
2. Load a configured `ESC sinusoidal` program onto the Teensy 4.0 microcontroller.
3. `cd` to the `./[spwm-root-directory]`
4. Optionally run a realtime graph to observe the anti cogging program's influence over the ESC: `kaepek-io-graph -a localhost -p 9002 -c ./lib/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
5. Run the director: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp`
6. Run the anti-cogging program: `node ./dist/lib/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --output_data_file ./calibration-data/<anti-cogging-output-file-name>.json --incoming_address localhost --incoming_port 9001 --incoming_protocol udp --outgoing_address localhost --outgoing_port 9002 --outgoing_protocol udp  --command_address localhost --command_port 9000 --command_protocol udp --angular_steps 16384 --angular_compression_ratio <angular-compression-ratio>`
    - You need to replace `<anti-cogging-output-file-name>` with a suitable name, perhaps if using a `tarot-4006` motor the following would be suitable `tarot-4006-esc-sinusoidal-ac.json`
    - `--angular_steps` defaults to `16384` so this is optional when using a AS5147P encoder.
    - You need to set `<angular-compression-ratio>` to the correct value, one can do this by taking the configured value of the `ENCODER_VALUE_COMPRESSION` found within the `ESC sinusodial code` e.g. `8`.
- Example command for the anti-cogging program: `node ./dist/lib/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --output_data_file ./calibration-data/tarot-4006-esc-sinusoidal-ac.json --incoming_address localhost --incoming_port 9001 --incoming_protocol udp --outgoing_address localhost --outgoing_port 9002 --outgoing_protocol udp  --command_address localhost --command_port 9000 --command_protocol udp --angular_steps 16384 --angular_compression_ratio 8`
7. When the anti-cogging program has finished, optionally graph the collected anti-cogging data with the following commands:
    1. Export the collected anti-cogging data to a csv: `node ./dist/lib/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/<anti-cogging-output-file-name>.json -c ./lib/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/<anti-cogging-output-graph-data-file-name>.csv` replace `<anti-cogging-output-file-name>.json` with the relevant name and rename `<anti-cogging-output-graph-data-file-name>.csv` with a name of your choosing.
    2. Use the graph-file program to render the csv to a graph file: `kaepek-io-graph-file -i ./calibration-data/<anti-cogging-output-graph-data-file-name>.csv -c ./lib/speed-control/graph_configs/ac_map_graphs.json`
    3. Inspect the outputed graph `./calibration-data/<anti-cogging-output-graph-data-file-name>.csv.html`
- Example combined command: `node ./dist/lib/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/tarot-4006-esc-sinusoidal-ac.json -c ./lib/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/tarot-4006-esc-sinusoidal-ac.json.csv && kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-sinusoidal-ac.json.csv -c ./lib/speed-control/graph_configs/ac_map_graphs.json`, outputed file for inspection would be `./calibration-data/tarot-4006-esc-sinusoidal-ac.json.csv.html`.
8. Replace the file in the directory `./speed-control/closed-loop/AS5147P/teensy40/esc-ac/calibration/ac.cpp` with the anti-cogging map cpp file found here: `./calibration-data/<anti-cogging-output-file-name>.cpp` make sure the file is named `ac.cpp`
9. Load the `ESC sinusoidal anti-cogging` ino onto the Teensy4.0 microcontroller.
10. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9002,udp`
11. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs (can start and restart this program with alternative configs as you desire):
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/all_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time_no_buffer.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/`
12. Use the keyboard (click the director program terminal session first), ps4 dualshock controller or netsend program to control the ESC.
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
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w start` to start.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w stop` to stop.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w reset` to reset.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w thrustui16 -d <duty_value>` to set duty (`<duty_value>` ranges from 0 -> 65535 [min->max], converted to 0 -> `MAX_DUTY` inside the controller, depending on setting `PWM_WRITE_RESOLUTION` the `MAX_DUTY` is `2^PWM_WRITE_RESOLUTION - 1`, so if `PWM_WRITE_RESOLUTION` is set to 11 then `MAX_DUTY` would be 2047, if then you sent the following command `kaepek-io-netsend -h localhost -p 9000 -n udp -w thrustui16 -d 65535` the controller would apply a duty cycle of 2047).
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.

### ESC sinusoidal anti-cogging pid

- [ESC sinusoidal anti-cogging pid code](./esc-pid-ac/esc-pid-ac.ino)

#### Usage instructions

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/tree/FEATURES/new-documentation#spwm-procedure) see the ESC sinusoidal section. 
1. In the `ESC sinusodial code` ino update the `MOTOR_CONFIG` given the calibration fit read of a graph file `./[calibration-dir]/calibration-data/zc_reconstruction_[combined_id].png`, the following must be set:
    - `MOTOR_CONFIG_CW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_ZERO_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_CCW_PHASE_DISPLACEMENT_DEG`
    - `MOTOR_CONFIG_NUMBER_OF_POLES`
2. Load a configured `ESC sinusoidal` program onto the Teensy 4.0 microcontroller.
3. `cd` to the `./[spwm-root-directory]`
4. Optionally run a realtime graph to observe the anti cogging program's influence over the ESC: `kaepek-io-graph -a localhost -p 9002 -c ./lib/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
5. Run the director: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp`
6. Run the anti-cogging program: `node ./dist/lib/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --output_data_file ./calibration-data/<anti-cogging-output-file-name>.json --incoming_address localhost --incoming_port 9001 --incoming_protocol udp --outgoing_address localhost --outgoing_port 9002 --outgoing_protocol udp  --command_address localhost --command_port 9000 --command_protocol udp --angular_steps 16384 --angular_compression_ratio <angular-compression-ratio>`
    - You need to replace `<anti-cogging-output-file-name>` with a suitable name, perhaps if using a `tarot-4006` motor the following would be suitable `tarot-4006-esc-sinusoidal-ac.json`
    - `--angular_steps` defaults to `16384` so this is optional when using a AS5147P encoder.
    - You need to set `<angular-compression-ratio>` to the correct value, one can do this by taking the configured value of the `ENCODER_VALUE_COMPRESSION` found within the `ESC sinusodial code` e.g. `8`.
- Example command for the anti-cogging program: `node ./dist/lib/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --output_data_file ./calibration-data/tarot-4006-esc-sinusoidal-ac.json --incoming_address localhost --incoming_port 9001 --incoming_protocol udp --outgoing_address localhost --outgoing_port 9002 --outgoing_protocol udp  --command_address localhost --command_port 9000 --command_protocol udp --angular_steps 16384 --angular_compression_ratio 8`
7. When the anti-cogging program has finished, optionally graph the collected anti-cogging data with the following commands:
    1. Export the collected anti-cogging data to a csv: `node ./dist/lib/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/<anti-cogging-output-file-name>.json -c ./lib/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/<anti-cogging-output-graph-data-file-name>.csv` replace `<anti-cogging-output-file-name>.json` with the relevant name and rename `<anti-cogging-output-graph-data-file-name>.csv` with a name of your choosing.
    2. Use the graph-file program to render the csv to a graph file: `kaepek-io-graph-file -i ./calibration-data/<anti-cogging-output-graph-data-file-name>.csv -c ./lib/speed-control/graph_configs/ac_map_graphs.json`
    3. Inspect the outputed graph `./calibration-data/<anti-cogging-output-graph-data-file-name>.csv.html`
- Example combined command: `node ./dist/lib/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/tarot-4006-esc-sinusoidal-ac.json -c ./lib/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/tarot-4006-esc-sinusoidal-ac.json.csv && kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-sinusoidal-ac.json.csv -c ./lib/speed-control/graph_configs/ac_map_graphs.json`, outputed file for inspection would be `./calibration-data/tarot-4006-esc-sinusoidal-ac.json.csv.html`.
8. Replace the file in the directory `./speed-control/closed-loop/AS5147P/teensy40/esc-ac/calibration/ac.cpp` with the anti-cogging map cpp file found here: `./calibration-data/<anti-cogging-output-file-name>.cpp` make sure the file is named `ac.cpp`
9. Load the `ESC sinusoidal anti-cogging pid` ino onto the Teensy4.0 microcontroller.
10. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c directionui8 start stop reset setpointf32 proportionalf32 integralf32 derivativef32 powerlawrootccwf32 powerlawrootcwf32 powerlawsetpointdivisorccwf32 powerlawsetpointdivisorcwf32 linearbiascwf32 linearbiasccwf32 linearsetpointcoefficientcwF32 linearsetpointcoefficientccwF32  -p serial console -o network=localhost,9002,udp`
11. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try the following config:
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/pid.json`
12. Use the keyboard (click the director program terminal session first), ps4 dualshock controller or netsend program to control the ESC. Note you must use the netsend program to update PID values.
    - keyboard usage: 
        - `[space]` to start.
        - `x` to stop.
        - `r` to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `q` to change direction (note thrust must be zero to change direction).
    - dualshock ps4 controller usage: 
        - `options` button to start.
        - `share` button to stop.
        - `playstation` icon middle button to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `triangle` to change direction (note thrust must be zero to change direction).
    - netsend usage:
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w start` to start.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w stop` to stop.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w reset` to reset.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w setpointf32 -d 1.3` to set the PID setpoint velocity to 1.3 `[Hz]`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d <proportional-coefficient>` to set the PID proportional-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d 1` sets the PID proportional-coefficient to a value of `1`.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w integralf32 -d <integral-coefficient>` to set the PID integral-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w integralf32 -d 20` sets the PID integral-coefficient to a value of `20`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w derivativef32 -d <derivative-coefficient>` to set the PID derivative-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w derivativef32 -d 0.04` sets the PID derivative-coefficient to a value of `0.04`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootcwf32 -d <power-law-root-cw>` to set the PID power law root as determined from the step change regression analysis (power law m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootcwf32 -d 1.358356`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootccwf32 -d <power-law-root-ccw>` to set the PID power law root as determined from the step change regression analysis (power law m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootccwf32 -d 1.378703`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorcwf32 -d <power-law-set-point-divisor-cw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorcwf32 -d 85.08715447157786`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorccwf32 -d <power-law-set-point-divisor-ccw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorccwf32 -d 87.27482768135731`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiascwf32 -d <linear-bias-cw>` to set the PID linear bias. As determined from the step change regression analysis (linear model -c/m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiascwf32 -d 0.022338094680920625`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiasccwf32 -d <linear-bias-ccw>` to set the PID linear bias as determined from the step change regression analysis (linear model -c/m value )for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiasccwf32 -d 0.022384353255695207`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientcwF32 -d <linear-setpoint-coefficient-cw>` to set the PID linear set point coefficient. As determined from the step change regression analysis (linear model 1/m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientcwF32 -d 0.02064702106741506`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientccwF32 -d <linear-setpoint-coefficient-ccw>` to set the PID linear set point coefficient. As determined from the step change regression analysis (linear model 1/m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientccwF32 -d 0.020612730310931287`

### ESC direct

- [ESC direct code](./esc-direct/esc-direct.ino)

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/tree/FEATURES/new-documentation#dpwm-procedure) see the ESC direct pwm section.
1. Copy the contents of the relevant cpp direct fit data e.g. `combination-direct-fit-ynitlldoqesyyvgyuwyg.cpp` to the following directory file `./speed-control/closed-loop/AS5147P/teensy40/esc-direct/calibration/voltage-map.cpp`.
2. Load the `ESC direct` code onto the teensy40 microcontroller.
3. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9002,udp`
4. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs (can start and restart this program with alternative configs as you desire):
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/all_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time_no_buffer.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/`
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
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w start` to start.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w stop` to stop.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w reset` to reset.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w thrustui16 -d <duty_value>` to set duty (`<duty_value>` ranges from 0 -> 65535 [min->max], converted to 0 -> `MAX_DUTY` inside the controller, depending on setting `PWM_WRITE_RESOLUTION` the `MAX_DUTY` is `2^PWM_WRITE_RESOLUTION - 1`, so if `PWM_WRITE_RESOLUTION` is set to 11 then `MAX_DUTY` would be 2047, if then you sent the following command `kaepek-io-netsend -h localhost -p 9000 -n udp -w thrustui16 -d 65535` the controller would apply a duty cycle of 2047).
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.

### ESC direct pid

- [ESC direct pid code](./esc-direct-pid/esc-direct-pid.ino)

#### Usage instructions

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/tree/FEATURES/new-documentation#dpwm-procedure) see the ESC direct PWM section.
1. Copy the contents of the relevant cpp direct fit data e.g. `combination-direct-fit-ynitlldoqesyyvgyuwyg.cpp` to the following directory file `./speed-control/closed-loop/AS5147P/teensy40/esc-direct-pid/calibration/voltage-map.cpp`.
2. Load the `ESC direct pid` code onto the teensy40 microcontroller.
3. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c directionui8 start stop reset setpointf32 proportionalf32 integralf32 derivativef32 powerlawrootccwf32 powerlawrootcwf32 powerlawsetpointdivisorccwf32 powerlawsetpointdivisorcwf32 linearbiascwf32 linearbiasccwf32 linearsetpointcoefficientcwF32 linearsetpointcoefficientccwF32  -p serial console -o network=localhost,9002,udp`
4. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try the following config:
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/pid.json`
5. Use the keyboard (click the director program terminal session first), ps4 dualshock controller or netsend program to control the ESC. Note you must use the netsend program to update PID values.
    - keyboard usage: 
        - `[space]` to start.
        - `x` to stop.
        - `r` to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `q` to change direction (note thrust must be zero to change direction).
    - dualshock ps4 controller usage: 
        - `options` button to start.
        - `share` button to stop.
        - `playstation` icon middle button to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `triangle` to change direction (note thrust must be zero to change direction).
    - netsend usage:
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w start` to start.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w stop` to stop.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w reset` to reset.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w setpointf32 -d 1.3` to set the PID setpoint velocity to 1.3 `[Hz]`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d <proportional-coefficient>` to set the PID proportional-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d 1` sets the PID proportional-coefficient to a value of `1`.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w integralf32 -d <integral-coefficient>` to set the PID integral-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w integralf32 -d 20` sets the PID integral-coefficient to a value of `20`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w derivativef32 -d <derivative-coefficient>` to set the PID derivative-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w derivativef32 -d 0.04` sets the PID derivative-coefficient to a value of `0.04`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootcwf32 -d <power-law-root-cw>` to set the PID power law root as determined from the step change regression analysis (power law m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootcwf32 -d 1.358356`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootccwf32 -d <power-law-root-ccw>` to set the PID power law root as determined from the step change regression analysis (power law m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootccwf32 -d 1.378703`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorcwf32 -d <power-law-set-point-divisor-cw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorcwf32 -d 85.08715447157786`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorccwf32 -d <power-law-set-point-divisor-ccw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorccwf32 -d 87.27482768135731`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiascwf32 -d <linear-bias-cw>` to set the PID linear bias. As determined from the step change regression analysis (linear model -c/m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiascwf32 -d 0.022338094680920625`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiasccwf32 -d <linear-bias-ccw>` to set the PID linear bias as determined from the step change regression analysis (linear model -c/m value )for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiasccwf32 -d 0.022384353255695207`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientcwF32 -d <linear-setpoint-coefficient-cw>` to set the PID linear set point coefficient. As determined from the step change regression analysis (linear model 1/m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientcwF32 -d 0.02064702106741506`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientccwF32 -d <linear-setpoint-coefficient-ccw>` to set the PID linear set point coefficient. As determined from the step change regression analysis (linear model 1/m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientccwF32 -d 0.020612730310931287`


### ESC direct anti-cogging

- [ESC direct anti-cogging code](./esc-direct-ac/esc-direct-ac.ino)

#### Usage instructions

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/tree/FEATURES/new-documentation#dpwm-procedure) see the ESC direct PWM section.
1. Copy the contents of the relevant cpp direct fit data e.g. `combination-direct-fit-ynitlldoqesyyvgyuwyg.cpp` to the following directory file `./speed-control/closed-loop/AS5147P/teensy40/esc-direct/calibration/voltage-map.cpp`.
2. Load the `ESC direct` code onto the teensy40 microcontroller.
3. `cd` to the `./[spwm-root-directory]`
4. Optionally run a realtime graph to observe the anti cogging program's influence over the ESC: `kaepek-io-graph -a localhost -p 9002 -c ./lib/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
5. Run the director: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp`
6. Run the anti-cogging program: `node ./dist/lib/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --output_data_file ./calibration-data/<anti-cogging-output-file-name>.json --incoming_address localhost --incoming_port 9001 --incoming_protocol udp --outgoing_address localhost --outgoing_port 9002 --outgoing_protocol udp  --command_address localhost --command_port 9000 --command_protocol udp --angular_steps 16384 --angular_compression_ratio <angular-compression-ratio>`
    - You need to replace `<anti-cogging-output-file-name>` with a suitable name, perhaps if using a `tarot-4006` motor the following would be suitable `tarot-4006-esc-direct-ac.json`
    - `--angular_steps` defaults to `16384` so this is optional when using a AS5147P encoder.
    - You need to set `<angular-compression-ratio>` to the correct value, one can do this by taking the configured value of the `ENCODER_VALUE_COMPRESSION` found within the `ESC direct code` e.g. `4`.
- Example command for the anti-cogging program: `node ./dist/lib/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --output_data_file ./calibration-data/tarot-4006-esc-direct-ac.json --incoming_address localhost --incoming_port 9001 --incoming_protocol udp --outgoing_address localhost --outgoing_port 9002 --outgoing_protocol udp  --command_address localhost --command_port 9000 --command_protocol udp --angular_steps 16384 --angular_compression_ratio 4`
7. When the anti-cogging program has finished, optionally graph the collected anti-cogging data with the following commands:
    1. Export the collected anti-cogging data to a csv: `node ./dist/lib/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/<anti-cogging-output-file-name>.json -c ./lib/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/<anti-cogging-output-graph-data-file-name>.csv` replace `<anti-cogging-output-file-name>.json` with the relevant name and rename `<anti-cogging-output-graph-data-file-name>.csv` with a name of your choosing.
    2. Use the graph-file program to render the csv to a graph file: `kaepek-io-graph-file -i ./calibration-data/<anti-cogging-output-graph-data-file-name>.csv -c ./lib/speed-control/graph_configs/ac_map_graphs.json`
    3. Inspect the outputed graph `./calibration-data/<anti-cogging-output-graph-data-file-name>.csv.html`
- Example combined command: `node ./dist/lib/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/tarot-4006-esc-direct-ac.json -c ./lib/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/tarot-4006-esc-direct-ac.json.csv && kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-direct-ac.json.csv -c ./lib/speed-control/graph_configs/ac_map_graphs.json`, outputed file for inspection would be `./calibration-data/tarot-4006-esc-direct-ac.json.csv.html`.
8. Replace the file in the directory `./speed-control/closed-loop/AS5147P/teensy40/esc-direct-ac/calibration/ac.cpp` with the anti-cogging map cpp file found here: `./calibration-data/<anti-cogging-output-file-name>.cpp` make sure the file is named `ac.cpp`
9. Load the `ESC direct anti-cogging` ino onto the Teensy4.0 microcontroller.
10. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9002,udp`
11. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs (can start and restart this program with alternative configs as you desire):
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/all_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time_no_buffer.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/`
12. Use the keyboard (click the director program terminal session first), ps4 dualshock controller or netsend program to control the ESC.
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
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w start` to start.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w stop` to stop.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w reset` to reset.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w thrustui16 -d <duty_value>` to set duty (`<duty_value>` ranges from 0 -> 65535 [min->max], converted to 0 -> `MAX_DUTY` inside the controller, depending on setting `PWM_WRITE_RESOLUTION` the `MAX_DUTY` is `2^PWM_WRITE_RESOLUTION - 1`, so if `PWM_WRITE_RESOLUTION` is set to 11 then `MAX_DUTY` would be 2047, if then you sent the following command `kaepek-io-netsend -h localhost -p 9000 -n udp -w thrustui16 -d 65535` the controller would apply a duty cycle of 2047).
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.

### ESC direct pid ac

- [ESC direct anti-cogging pid code](./esc-direct-pid-ac/esc-direct-pid-ac.ino)


#### Usage instructions

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/tree/FEATURES/new-documentation#dpwm-procedure) see the ESC direct PWM section.
1. Copy the contents of the relevant cpp direct fit data e.g. `combination-direct-fit-ynitlldoqesyyvgyuwyg.cpp` to the following directory file `./speed-control/closed-loop/AS5147P/teensy40/esc-direct/calibration/voltage-map.cpp`.
2. Load the `ESC direct` code onto the teensy40 microcontroller.
3. `cd` to the `./[spwm-root-directory]`
4. Optionally run a realtime graph to observe the anti cogging program's influence over the ESC: `kaepek-io-graph -a localhost -p 9002 -c ./lib/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
5. Run the director: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp`
6. Run the anti-cogging program: `node ./dist/lib/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --output_data_file ./calibration-data/<anti-cogging-output-file-name>.json --incoming_address localhost --incoming_port 9001 --incoming_protocol udp --outgoing_address localhost --outgoing_port 9002 --outgoing_protocol udp  --command_address localhost --command_port 9000 --command_protocol udp --angular_steps 16384 --angular_compression_ratio <angular-compression-ratio>`
    - You need to replace `<anti-cogging-output-file-name>` with a suitable name, perhaps if using a `tarot-4006` motor the following would be suitable `tarot-4006-esc-direct-ac.json`
    - `--angular_steps` defaults to `16384` so this is optional when using a AS5147P encoder.
    - You need to set `<angular-compression-ratio>` to the correct value, one can do this by taking the configured value of the `ENCODER_VALUE_COMPRESSION` found within the `ESC direct code` e.g. `4`.
- Example command for the anti-cogging program: `node ./dist/lib/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --output_data_file ./calibration-data/tarot-4006-esc-direct-ac.json --incoming_address localhost --incoming_port 9001 --incoming_protocol udp --outgoing_address localhost --outgoing_port 9002 --outgoing_protocol udp  --command_address localhost --command_port 9000 --command_protocol udp --angular_steps 16384 --angular_compression_ratio 4`
7. When the anti-cogging program has finished, optionally graph the collected anti-cogging data with the following commands:
    1. Export the collected anti-cogging data to a csv: `node ./dist/lib/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/<anti-cogging-output-file-name>.json -c ./lib/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/<anti-cogging-output-graph-data-file-name>.csv` replace `<anti-cogging-output-file-name>.json` with the relevant name and rename `<anti-cogging-output-graph-data-file-name>.csv` with a name of your choosing.
    2. Use the graph-file program to render the csv to a graph file: `kaepek-io-graph-file -i ./calibration-data/<anti-cogging-output-graph-data-file-name>.csv -c ./lib/speed-control/graph_configs/ac_map_graphs.json`
    3. Inspect the outputed graph `./calibration-data/<anti-cogging-output-graph-data-file-name>.csv.html`
- Example combined command: `node ./dist/lib/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/tarot-4006-esc-direct-ac.json -c ./lib/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/tarot-4006-esc-direct-ac.json.csv && kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-direct-ac.json.csv -c ./lib/speed-control/graph_configs/ac_map_graphs.json`, outputed file for inspection would be `./calibration-data/tarot-4006-esc-direct-ac.json.csv.html`.
8. Replace the file in the directory `./speed-control/closed-loop/AS5147P/teensy40/esc-direct-ac/calibration/ac.cpp` with the anti-cogging map cpp file found here: `./calibration-data/<anti-cogging-output-file-name>.cpp` make sure the file is named `ac.cpp`
9. Load the `ESC direct anti-cogging pid` ino onto the Teensy4.0 microcontroller.
10. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c directionui8 start stop reset setpointf32 proportionalf32 integralf32 derivativef32 powerlawrootccwf32 powerlawrootcwf32 powerlawsetpointdivisorccwf32 powerlawsetpointdivisorcwf32 linearbiascwf32 linearbiasccwf32 linearsetpointcoefficientcwF32 linearsetpointcoefficientccwF32  -p serial console -o network=localhost,9002,udp`
11. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try the following config:
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/pid.json`
12. Use the keyboard (click the director program terminal session first), ps4 dualshock controller or netsend program to control the ESC. Note you must use the netsend program to update PID values.
    - keyboard usage: 
        - `[space]` to start.
        - `x` to stop.
        - `r` to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `q` to change direction (note thrust must be zero to change direction).
    - dualshock ps4 controller usage: 
        - `options` button to start.
        - `share` button to stop.
        - `playstation` icon middle button to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `triangle` to change direction (note thrust must be zero to change direction).
    - netsend usage:
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w start` to start.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w stop` to stop.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w reset` to reset.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -h localhost -p 9000 -n udp -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w setpointf32 -d 1.3` to set the PID setpoint velocity to 1.3 `[Hz]`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d <proportional-coefficient>` to set the PID proportional-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d 1` sets the PID proportional-coefficient to a value of `1`.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w integralf32 -d <integral-coefficient>` to set the PID integral-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w integralf32 -d 20` sets the PID integral-coefficient to a value of `20`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w derivativef32 -d <derivative-coefficient>` to set the PID derivative-coefficient. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w derivativef32 -d 0.04` sets the PID derivative-coefficient to a value of `0.04`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootcwf32 -d <power-law-root-cw>` to set the PID power law root as determined from the step change regression analysis (power law m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootcwf32 -d 1.358356`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootccwf32 -d <power-law-root-ccw>` to set the PID power law root as determined from the step change regression analysis (power law m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawrootccwf32 -d 1.378703`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorcwf32 -d <power-law-set-point-divisor-cw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorcwf32 -d 85.08715447157786`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorccwf32 -d <power-law-set-point-divisor-ccw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w powerlawsetpointdivisorccwf32 -d 87.27482768135731`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiascwf32 -d <linear-bias-cw>` to set the PID linear bias. As determined from the step change regression analysis (linear model -c/m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiascwf32 -d 0.022338094680920625`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiasccwf32 -d <linear-bias-ccw>` to set the PID linear bias as determined from the step change regression analysis (linear model -c/m value )for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearbiasccwf32 -d 0.022384353255695207`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientcwF32 -d <linear-setpoint-coefficient-cw>` to set the PID linear set point coefficient. As determined from the step change regression analysis (linear model 1/m value) for the clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientcwF32 -d 0.02064702106741506`
        - `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientccwF32 -d <linear-setpoint-coefficient-ccw>` to set the PID linear set point coefficient. As determined from the step change regression analysis (linear model 1/m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -h localhost -p 9000 -n udp -w linearsetpointcoefficientccwF32 -d 0.020612730310931287`

### Step change analysis - PID auto configure.

A program to perform a step change analysis and get PID values.

1. First load one of the following ESC's onto the Teensy 4.0 microcontroller. Note that for each of these a different step change analysis would be create and they are not interchangable. Also not that the analysis is dependant on the motor load (in the standard use case a fixed mass), if the motor load has changed the step-change analysis will no longer be relevant.
    - ESC sinusoidal.
    - ESC sinusoidal anti-cogging.
    - ESC direct.
    - ESC direct anti-cogging.
2. Configure the motor parameters as per the specific sections configuration instructions.
3. Start the director: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp`.
3. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs (can start and restart this program with alternative configs as you desire):
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/all_by_time.json`.
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_by_time.json`.
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time.json`.
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_time_no_buffer.json`.
    - `./[spwm-root-directory]/lib/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`.
4. Run the step change analysis program: `node ./dist/lib/speed-control/programs/step-change.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --duty_cap_multiplier 0.3 --output_data_file ./calibration-data/<step-change-output-file-name>.json`.
- Example command: `node ./dist/lib/speed-control/programs/step-change.js --input_config_file ./lib/speed-control/graph_configs/control_kalman_hz_by_time.json --duty_cap_multiplier 0.3 --output_data_file ./calibration-data/tarot-4006-esc-direct-step-change.json`.
5. After the step change program has finished graph the step change output: `kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.detections.cw.csv -c ./lib/speed-control/graph_configs/step_change.json && kaepek-io-graph-file -i ./calibration-data/<step-change-output-file-name>.detections.ccw.csv -c ./lib/speed-control/graph_configs/step_change.json`
- Example command: `kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-direct-step-change.detections.cw.csv -c ./lib/speed-control/graph_configs/step_change.json && kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-direct-step-change.detections.ccw.csv -c ./lib/speed-control/graph_configs/step_change.json`
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

# [Home Readme](../../../../README.md)