kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset -p serial console -o network=localhost,9001,udp

kaepek-io-graph -a localhost -p 9001 -c ./speed-control/closed-loop/AS5147P/teensy40/esc-pid/graph_configs/kalman_control_pid.json


so for control use netsend

- need to set set_point
- need suitable scale for p terms

