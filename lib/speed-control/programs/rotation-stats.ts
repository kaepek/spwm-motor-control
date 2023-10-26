import { ArgumentHandlers, CliArg, CliArgType, parse_args } from "../../../external/kaepek-io/lib/host/controller/utils/cli-args.js";
import { ASCIIParser } from "../../../external/kaepek-io/lib/host/controller/utils/ascii-parser.js"
import { ACMap } from "../tasks/collect-acceleration-data";
import fs from "fs";
import NetworkAdaptor from "../../../external/kaepek-io/lib/host/controller/utils/network.js";
import { rotation_detector } from "../../rotation-detector.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";

const cli_args: Array<CliArg> = [
    {
        name: "input_config_file",
        type: CliArgType.InputFilePath,
        short: "c",
        help: "Config file for microcontroller input format for data coming from kaepek-io-direction",
        required: true
    },
    {
        name: "incoming_address",
        type: CliArgType.String,
        short: "a",
        required: false,
        help: "The incoming address to indicate what host to connect to, to accept incoming data coming from the kaepek-io-director program.",
        default: "localhost",
        group: "incoming"
    },
    {
        name: "incoming_port",
        type: CliArgType.Number,
        short: "p",
        help: "The incoming port to indicate what host port to connect to, to accept incoming data coming from the kaepek-io-director program.",
        required: false,
        default: 9001,
        group: "incoming"
    },
    {
        name: "incoming_protocol",
        type: CliArgType.String,
        short: "n",
        help: "The incoming protocol to indicate what host connection protocol to connect via, to accept incoming data coming from the kaepek-io-director program.",
        required: false,
        default: "udp",
        group: "incoming"
    },
    {
        name: "outgoing_address",
        type: CliArgType.String,
        short: "s",
        required: false,
        help: "The outgoing address to indicate where this program will send data to. Useful for graphing via the kaepek-io-graph program.",
        default: "localhost",
        group: "outgoing"
    },
    {
        name: "outgoing_port",
        type: CliArgType.Number,
        short: "x",
        help: "The outgoing port to indicate what port this program will send data to. Useful for graphing via the kaepek-io-graph program.",
        default: 9002,
        required: false,
        group: "outgoing"
    },
    {
        name: "outgoing_protocol",
        type: CliArgType.String,
        short: "v",
        help: "The outgoing protocol to indicate what protocol this program will send data to. Useful for graphing via the kaepek-io-graph program.",
        default: "udp",
        required: false,
        group: "outgoing"
    },
    {
        name: "direction",
        type: CliArgType.String,
        short: "d",
        help: "Direction we are wanting to analyse, values are 'cw' (clockwise) or 'ccw' (counter-clockwise).",
        required: true,
    },
    {
        name: "n_rotations",
        type: CliArgType.Number,
        short: "w",
        help: "The number of rotations before statistics about rotation are updated",
        required: false,
        default: 1
    }
];

const parsed_args = parse_args("RotationStatistics", cli_args, ArgumentHandlers) as any;

const direction = parsed_args.direction;
let direction_bl = null;
if (direction === "cw") {
    direction_bl = true;
}
else if (direction === "ccw") {
    direction_bl = false;
}
else {
    throw "--direction not recognised should be 'cw' (clockwise) or 'ccw' (counter-clockwise)."
}

const adaptor = new NetworkAdaptor(parsed_args.incoming_address, parsed_args.incoming_port, parsed_args.incoming_protocol, parsed_args.input_config_file, ",", parsed_args.outgoing_address, parsed_args.outgoing_port, parsed_args.outgoing_protocol);

adaptor.outgoing_data_config = adaptor.incoming_data_config;

adaptor.incoming_data$.subscribe((line_data) => {
    adaptor.transmit_outgoing_data(line_data.parsed_data);
});


let initial_rotations = 0.0;
let velocity_values: Array<number> = [];
let acceleration_values: Array<number> = [];
const dp = 4;
// 4 decimal places?

const rotations$ = rotation_detector(adaptor.incoming_data$, direction_bl);

rotations$.subscribe((rotation_data) => {
    console.log("rotation_data", rotation_data);
    if (rotation_data.motion === false) {
        // reset
        initial_rotations = 0.0;
    }
    else {
        const net_rotation_since_last_stat = (rotation_data.rotations - initial_rotations);
        if (net_rotation_since_last_stat > parsed_args.n_rotations) {
            initial_rotations = rotation_data.rotations;

            // compute stats
            let min_velocity = Number.POSITIVE_INFINITY;
            let max_velocity  = Number.NEGATIVE_INFINITY;
            let min_acceleration = Number.POSITIVE_INFINITY;
            let max_acceleration = Number.NEGATIVE_INFINITY;

            const mean_velocity = (velocity_values.reduce((acc, velocity) => {
                if (velocity < min_velocity) {
                    min_velocity = velocity;
                }
                if (velocity > max_velocity) {
                    max_velocity = velocity;
                }
                acc += velocity;
                return acc;
            }, 0))/velocity_values.length;

            const std_velocity = Math.sqrt((velocity_values.reduce((acc, velocity)=> {
                acc += Math.pow(velocity - mean_velocity, 2);
                return acc;
            },0))/velocity_values.length);

            const mean_acceleration = (acceleration_values.reduce((acc, acceleration) => {
                if (acceleration < min_acceleration) {
                    min_acceleration = acceleration;
                }
                if (acceleration > max_acceleration) {
                    max_acceleration = acceleration;
                }
                acc += acceleration;
                return acc;
            }, 0))/acceleration_values.length;

            const std_acceleration = Math.sqrt((acceleration_values.reduce((acc, acceleration)=> {
                acc += Math.pow(acceleration - mean_acceleration, 2);
                return acc;
            },0))/acceleration_values.length);
            const now = new Date();
            const time = `${now.getHours()}:${now.getMinutes()}:${now.getSeconds()}`;
            console2.success(
`Time: ${time} -------------
Velocity  ------------------
Min: ${min_velocity.toFixed(dp)} [Hz]
Max: ${max_velocity.toFixed(dp)} [Hz]
Avg: ${mean_velocity.toFixed(dp)} [Hz]
Std: ${std_velocity.toFixed(dp)} [Hz]
Acceleration ---------------
MinA: ${min_acceleration.toFixed(dp)} [Hz^2]
MaxA: ${max_acceleration.toFixed(dp)} [Hz^2]
AvgA: ${mean_acceleration.toFixed(dp)} [Hz^2]
StdA: ${std_acceleration.toFixed(dp)} [Hz^2]
----------------------------`

);
            velocity_values = [];
            acceleration_values = [];
        }
        else {
            velocity_values.push(rotation_data.line_data.parsed_data.kalman_velocity);
            acceleration_values.push(rotation_data.line_data.parsed_data.kalman_acceleration);
        }
    }
});


adaptor.ready();