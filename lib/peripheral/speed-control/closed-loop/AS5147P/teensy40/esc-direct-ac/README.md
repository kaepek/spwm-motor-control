# ESC direct anti-cogging

- [ESC direct anti-cogging code](./esc-direct-ac.ino)

## Usage instructions

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/tree/FEATURES/new-documentation#dpwm-procedure) see the ESC direct PWM section.
1. Copy the contents of the relevant cpp direct fit data e.g. `combination-direct-fit-ynitlldoqesyyvgyuwyg.cpp` to the following directory file `./[spwm-root-directory]/lib/peripheral/speed-control/closed-loop/AS5147P/teensy40/esc-direct/calibration/voltage-map.cpp`.
2. Load the `ESC direct` code onto the teensy40 microcontroller.
3. `cd` to the `./[spwm-root-directory]`
4. Optionally run a realtime graph to observe the anti cogging program's influence over the ESC: `kaepek-io-graph -a localhost -p 9002 -c ./lib/host/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json` also one could run the [rotation stats analyser](../README.md#rotation-stats-analyser) with the following command `node ./dist/lib/host/speed-control/programs/rotation-stats.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_by_time.json`
5. Run the director: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp network=localhost,9002,udp network=localhost,9003,udp`
6. Run the anti-cogging program: `node ./dist/lib/host/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json --angular_compression_ratio <angular-compression-ratio> --output_data_file ./calibration-data/<anti-cogging-output-file-name>.json`
    - You need to replace `<anti-cogging-output-file-name>` with a suitable name, perhaps if using a `tarot-4006` motor the following would be suitable `tarot-4006-esc-direct-ac.json`
    - `--angular_steps` defaults to `16384` so this is optional when using a AS5147P encoder.
    - You need to set `<angular-compression-ratio>` to the correct value, one can do this by taking the configured value of the `ENCODER_VALUE_COMPRESSION` found within the `ESC direct code` e.g. `4`.
- Example command for the anti-cogging program: `node ./dist/lib/host/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json --angular_compression_ratio 4 --output_data_file ./calibration-data/tarot-4006-esc-direct-ac.json`
7. When the anti-cogging program has finished, optionally graph the collected anti-cogging data with the following commands:
    1. Export the collected anti-cogging data to a csv: `node ./dist/lib/host/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/<anti-cogging-output-file-name>.json -c ./lib/host/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/<anti-cogging-output-file-name>.csv` replace `<anti-cogging-output-file-name>.json` with the relevant name.
    2. Use the graph-file program to render the csv to a graph file: `kaepek-io-graph-file -i ./calibration-data/<anti-cogging-output-file-name>.csv -c ./lib/host/speed-control/graph_configs/ac_map_graphs.json`
    3. Inspect the outputed graph `./calibration-data/<anti-cogging-output-file-name>.csv.html`
- Example combined command: `node ./dist/lib/host/speed-control/programs/anti-cogging-calibration.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json --angular_compression_ratio 4 --output_data_file ./calibration-data/tarot-4006-esc-direct-ac.json && node ./dist/lib/host/speed-control/programs/ac-map-visualiser.js -i ./calibration-data/tarot-4006-esc-direct-ac.json -c ./lib/host/speed-control/graph_configs/ac_map_graphs.json -o ./calibration-data/tarot-4006-esc-direct-ac.json.csv && kaepek-io-graph-file -i ./calibration-data/tarot-4006-esc-direct-ac.json.csv -c ./lib/host/speed-control/graph_configs/ac_map_graphs.json`, outputed file for inspection would be `./calibration-data/tarot-4006-esc-direct-ac.json.csv.html`.
8. Replace the file in the directory `./[spwm-root-directory]/lib/peripheral/speed-control/closed-loop/AS5147P/teensy40/esc-direct-ac/calibration/ac.cpp` with the anti-cogging map cpp file found here: `./calibration-data/<anti-cogging-output-file-name>.cpp` make sure the file is named `ac.cpp`
9. Load the `ESC direct anti-cogging` ino onto the Teensy4.0 microcontroller.
10. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9002,udp network=localhost,9003,udp`
11. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs (can start and restart this program with alternative configs as you desire):
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/all_by_time.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_by_time.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_hz_by_time.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_hz_by_time_no_buffer.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
- Realtime graphing example command: `kaepek-io-graph -a localhost -p 9002 -c ./lib/host/speed-control/graph_configs/control_kalman_hz_by_encoder_step.json`
- Also one could run the [rotation stats analyser](../README.md#rotation-stats-analyser) with the following command `node ./dist/lib/host/speed-control/programs/rotation-stats.js --input_config_file ./lib/host/speed-control/graph_configs/control_kalman_by_time.json`
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
        - `kaepek-io-netsend -w start` to start.
        - `kaepek-io-netsend -w stop` to stop.
        - `kaepek-io-netsend -w reset` to reset.
        - `kaepek-io-netsend -w thrustui16 -d <duty_value>` to set duty (`<duty_value>` ranges from 0 -> 65535 [min->max], converted to 0 -> `MAX_DUTY` inside the controller, depending on setting `PWM_WRITE_RESOLUTION` the `MAX_DUTY` is `2^PWM_WRITE_RESOLUTION - 1`, so if `PWM_WRITE_RESOLUTION` is set to 11 then `MAX_DUTY` would be 2047, if then you sent the following command `kaepek-io-netsend -w thrustui16 -d 65535` the controller would apply a duty cycle of 2047).
        - `kaepek-io-netsend -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.

## [Parent Readme](../README.md)