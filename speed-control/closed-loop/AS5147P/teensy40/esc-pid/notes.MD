kaepek-io-director -i keyboard network=localhost,9000,udp dualshock -c start stop thrustui16 directionui8 reset setpointf32 proportionalf32 integralf32 derivativef32 -p serial console -o network=localhost,9001,udp


kaepek-io-graph -a localhost -p 9001 -c ./speed-control/closed-loop/AS5147P/teensy40/esc-pid/graph_configs/kalman_control_pid.json


so for control use netsend

- need to set set_point
- need suitable scale for p terms

0.5 corresponds to about 22hz


set set_point to 10 hz

kaepek-io-netsend -h localhost -p 9000 -n udp -w setpointf32 -d 10.0


give it 10 hz want to get about 

p guess 0.022727273

0.022727273 * 10

kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d 0.022727273



 2175  kaepek-io-netsend -h localhost -p 9000 -n udp -w setpointf32 -d 1.0
 2176  kaepek-io-netsend -h localhost -p 9000 -n udp -w derivativef32 -d 0.011
 2177  kaepek-io-netsend -h localhost -p 9000 -n udp -w proportionalf32 -d 0.4
 2178  kaepek-io-netsend -h localhost -p 9000 -n udp -w integralf32 -d 3.6
