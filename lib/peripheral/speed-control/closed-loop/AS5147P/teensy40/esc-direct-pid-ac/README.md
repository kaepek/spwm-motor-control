# ESC direct pid ac

- [ESC direct anti-cogging pid code](./esc-direct-pid-ac.ino)

## Usage instructions

0. Perform motor calibration using the [Calibration library](https://github.com/kaepek/calibration/blob/main/README.md#dpwm-procedure) see the ESC direct PWM section.
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
9. Load the `ESC direct anti-cogging pid` ino onto the Teensy4.0 microcontroller.
10. Run the director program: `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c directionui8 start stop reset setpointf32 proportionalf32 integralf32 derivativef32 powerlawrootccwf32 powerlawrootcwf32 powerlawsetpointdivisorccwf32 powerlawsetpointdivisorcwf32 linearbiascwf32 linearbiasccwf32 linearsetpointcoefficientcwF32 linearsetpointcoefficientccwF32  -p serial console -o network=localhost,9002,udp network=localhost,9003,udp`
11. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs:
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/pid_by_time.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/pid_by_encoder_step.json`
    - `./[spwm-root-directory]/lib/host/speed-control/graph_configs/pid_voltages_by_encoder_step.json`
- Realtime graphing example command: `kaepek-io-graph -a localhost -p 9002 -c ./lib/host/speed-control/graph_configs/pid_by_encoder_step.json`
- One could also run the [rotation stats analyser](../README.md#rotation-stats-analyser) with the following command `node ./dist/lib/host/speed-control/programs/rotation-stats.js --input_config_file ./lib/host/speed-control/graph_configs/pid_by_encoder_step.json`
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
        - `kaepek-io-netsend -w start` to start.
        - `kaepek-io-netsend -w stop` to stop.
        - `kaepek-io-netsend -w reset` to reset.
        - `kaepek-io-netsend -w directionui8 -d <direction>`, direction can be 0 for clockwise and 1 for counter-clockwise. E.g. to set the direction to clockwise use the following `kaepek-io-netsend -w directionui8 -d 0`. Note that the thrust must be zero before you will be allowed to change direction.
        - `kaepek-io-netsend -w setpointf32 -d 1.3` to set the PID setpoint velocity to 1.3 `[Hz]`
        - `kaepek-io-netsend -w proportionalf32 -d <proportional-coefficient>` to set the PID proportional-coefficient. E.g. `kaepek-io-netsend -w proportionalf32 -d 1` sets the PID proportional-coefficient to a value of `1`.
        - `kaepek-io-netsend -w integralf32 -d <integral-coefficient>` to set the PID integral-coefficient. E.g. `kaepek-io-netsend -w integralf32 -d 20` sets the PID integral-coefficient to a value of `20`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -w derivativef32 -d <derivative-coefficient>` to set the PID derivative-coefficient. E.g. `kaepek-io-netsend -w derivativef32 -d 0.04` sets the PID derivative-coefficient to a value of `0.04`. Caution setting this to high can cause an unstable system, please start with a low value.
        - `kaepek-io-netsend -w powerlawrootcwf32 -d <power-law-root-cw>` to set the PID power law root as determined from the [step change regression analysis](../README.md#step-change-analysis) (power law m value) for the clockwise direction. E.g. `kaepek-io-netsend -w powerlawrootcwf32 -d 1.358356`
        - `kaepek-io-netsend -w powerlawrootccwf32 -d <power-law-root-ccw>` to set the PID power law root as determined from the [step change regression analysis](../README.md#step-change-analysis) (power law m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -w powerlawrootccwf32 -d 1.378703`
        - `kaepek-io-netsend -w powerlawsetpointdivisorcwf32 -d <power-law-set-point-divisor-cw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the clockwise direction. E.g. `kaepek-io-netsend -w powerlawsetpointdivisorcwf32 -d 85.08715447157786`
        - `kaepek-io-netsend -w powerlawsetpointdivisorccwf32 -d <power-law-set-point-divisor-ccw>` to set the PID power law set point divisor. As determined by the step change regression analysis (power law 10^c value) for the counter clockwise direction. E.g. `kaepek-io-netsend -w powerlawsetpointdivisorccwf32 -d 87.27482768135731`
        - `kaepek-io-netsend -w linearbiascwf32 -d <linear-bias-cw>` to set the PID linear bias. As determined from the [step change regression analysis](../README.md#step-change-analysis) (linear model -c/m value) for the clockwise direction. E.g. `kaepek-io-netsend -w linearbiascwf32 -d 0.022338094680920625`
        - `kaepek-io-netsend -w linearbiasccwf32 -d <linear-bias-ccw>` to set the PID linear bias as determined from the [step change regression analysis](../README.md#step-change-analysis) (linear model -c/m value )for the counter clockwise direction. E.g. `kaepek-io-netsend -w linearbiasccwf32 -d 0.022384353255695207`
        - `kaepek-io-netsend -w linearsetpointcoefficientcwF32 -d <linear-setpoint-coefficient-cw>` to set the PID linear set point coefficient. As determined from the [step change regression analysis](../README.md#step-change-analysis) (linear model 1/m value) for the clockwise direction. E.g. `kaepek-io-netsend -w linearsetpointcoefficientcwF32 -d 0.02064702106741506`
        - `kaepek-io-netsend -w linearsetpointcoefficientccwF32 -d <linear-setpoint-coefficient-ccw>` to set the PID linear set point coefficient. As determined from the [step change regression analysis](../README.md#step-change-analysis) (linear model 1/m value) for the counter clockwise direction. E.g. `kaepek-io-netsend -w linearsetpointcoefficientccwF32 -d 0.020612730310931287`

## [Parent Readme](../README.md)