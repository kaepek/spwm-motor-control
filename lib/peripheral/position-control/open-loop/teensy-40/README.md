# Open loop position control

## Usage instructions

- [Open loop position control code](./teensy-40.ino)

1. Load the code onto the teensy40 microcontroller.
2. Start the director `kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop reset offsetf32 torqueui16 -p serial console -o network=localhost,9002,udp network=localhost,9003,udp`
4. Run the realtime graphing program `kaepek-io-graph -a localhost -p 9002 -c <config-file-path>`, try one of the following configs:
    - `./[spwm-root-directory]/lib/host/position-control/graph_configs/open-loop.json`
- Realtime graphing example command: `kaepek-io-graph -a localhost -p 9002 -c ./lib/host/position-control/graph_configs/open-loop.json`
5. Use the keyboard (click the director program terminal session first), ps4 dualshock controller or netsend program to control the ESC. Note you must use the netsend program to update PID values.
    - keyboard usage: 
        - `[space]` to start.
        - `x` to stop.
        - `r` to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `u` to increase the torque value.
        - `j` to decrease the torque value.
    - dualshock ps4 controller usage: 
        - `options` button to start.
        - `share` button to stop.
        - `playstation` icon middle button to reset (if ESC led is flashing indicating a fault, note must stop first).
        - `up` button to increase the torque value.
        - `down` button to decrease the torque value.
        - `lStickX` use the left stick x position to rotate the motor clockwise (right) or counter - clockwise (left).
    - netsend usage:
        - `kaepek-io-netsend -w start` to start.
        - `kaepek-io-netsend -w stop` to stop.
        - `kaepek-io-netsend -w reset` to reset.
        - `kaepek-io-netsend -w torqueui16 -d 16384` to set the torque value to 16384 out of 65535 or about 25%
        - `kaepek-io-netsend -w offsetf32 -d -180` to set the offset value to -180, corresponds to -180/100.0 electrical degree decrements per 1000 microseconds.

## [Parent Readme](../../../../../README.md)