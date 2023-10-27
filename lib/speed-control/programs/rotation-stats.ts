import { ArgumentHandlers, CliArg, CliArgType, parse_args } from "../../../external/kaepek-io/lib/host/controller/utils/cli-args.js";
import { ASCIIParser } from "../../../external/kaepek-io/lib/host/controller/utils/ascii-parser.js"
import { ACMap } from "../tasks/collect-acceleration-data";
import fs from "fs";
import NetworkAdaptor from "../../../external/kaepek-io/lib/host/controller/utils/network.js";
import { rotation_detector } from "../../rotation-detector.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";

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
        default: 9003,
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
        default: undefined,
        group: "outgoing"
    },
    {
        name: "outgoing_port",
        type: CliArgType.Number,
        short: "x",
        help: "The outgoing port to indicate what port this program will send data to. Useful for graphing via the kaepek-io-graph program.",
        default: undefined,
        required: false,
        group: "outgoing"
    },
    {
        name: "outgoing_protocol",
        type: CliArgType.String,
        short: "v",
        help: "The outgoing protocol to indicate what protocol this program will send data to. Useful for graphing via the kaepek-io-graph program.",
        default: undefined,
        required: false,
        group: "outgoing"
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


const adaptor = new NetworkAdaptor(parsed_args.incoming_address, parsed_args.incoming_port, parsed_args.incoming_protocol, parsed_args.input_config_file, ",", parsed_args.outgoing_address, parsed_args.outgoing_port, parsed_args.outgoing_protocol);

adaptor.outgoing_data_config = adaptor.incoming_data_config;

adaptor.incoming_data$.subscribe((line_data) => {
    adaptor.transmit_outgoing_data(line_data.parsed_data);
});

const dp = 4; // 4 decimal places

const rotations_cw$ = rotation_detector(adaptor.incoming_data$, true);
const rotations_ccw$ = rotation_detector(adaptor.incoming_data$, false);

const pad_zeros = (value: number, required_digits = 2) => {
    let value_string = value.toString();
    let value_missing_digits = required_digits - value_string.length;
    while (value_missing_digits != 0) {
        value_string = "0" + value_string;
        value_missing_digits--;
    }
    return value_string;
}

const rotation_direction_stats_logger = (rotations$: Observable<{
    motion: boolean;
    rotations: number;
    line_data: any;
}>, direction_str: string) => {
    let initial_rotations = 0.0;
    let velocity_values: Array<number> = [];
    let acceleration_values: Array<number> = [];
    rotations$.subscribe((rotation_data) => {
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
                let max_velocity = Number.NEGATIVE_INFINITY;
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
                }, 0)) / velocity_values.length;

                const std_velocity = Math.sqrt((velocity_values.reduce((acc, velocity) => {
                    acc += Math.pow(velocity - mean_velocity, 2);
                    return acc;
                }, 0)) / velocity_values.length);

                const mean_acceleration = (acceleration_values.reduce((acc, acceleration) => {
                    if (acceleration < min_acceleration) {
                        min_acceleration = acceleration;
                    }
                    if (acceleration > max_acceleration) {
                        max_acceleration = acceleration;
                    }
                    acc += acceleration;
                    return acc;
                }, 0)) / acceleration_values.length;

                const std_acceleration = Math.sqrt((acceleration_values.reduce((acc, acceleration) => {
                    acc += Math.pow(acceleration - mean_acceleration, 2);
                    return acc;
                }, 0)) / acceleration_values.length);

                const velocity_range = Math.abs(max_velocity - min_velocity);
                const acceleration_range = Math.abs(max_acceleration - min_acceleration);

                const velocity_min_offset_percentage = ((mean_velocity - min_velocity) / mean_velocity) * 100;
                const velocity_max_offset_percentage = ((max_velocity - mean_velocity) / mean_velocity) * 100;

                const acceleration_min_offset_percentage = ((mean_acceleration - min_acceleration) / mean_acceleration) * 100;
                const acceleration_max_offset_percentage = ((max_acceleration - mean_acceleration) / mean_acceleration) * 100;


                const now = new Date();
                const time = `${pad_zeros(now.getHours())}:${pad_zeros(now.getMinutes())}:${pad_zeros(now.getSeconds())}:${pad_zeros(now.getMilliseconds(),3)}`;


                console2.success(
                    `Time: ${time} -------------
                    Direction: ${direction_str}
                    Velocity  ------------------------
                    Avg: ${mean_velocity.toFixed(dp)} [Hz]
                    Std: ${std_velocity.toFixed(dp)} [Hz]
                    Min: ${min_velocity.toFixed(dp)} [Hz] Avg ${velocity_min_offset_percentage.toFixed(2)}%
                    Max: ${max_velocity.toFixed(dp)} [Hz] Avg ${velocity_max_offset_percentage.toFixed(2)}%
                    Range: ${velocity_range.toFixed(dp)} [Hz]
                    Acceleration -------------------
                    Avg: ${mean_acceleration.toFixed(dp)} [Hz^2]
                    Std: ${std_acceleration.toFixed(dp)} [Hz^2]
                    Min: ${min_acceleration.toFixed(dp)} [Hz^2] Avg ${acceleration_min_offset_percentage.toFixed(2)}%
                    Max: ${max_acceleration.toFixed(dp)} [Hz^2] Avg ${acceleration_max_offset_percentage.toFixed(2)}%
                    Range: ${acceleration_range.toFixed(dp)} [Hz^2]
                    --------------------------------`.replace(/  +/g, '')
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
}

rotation_direction_stats_logger(rotations_cw$, "Clockwise");
rotation_direction_stats_logger(rotations_ccw$, "Counter Clockwise");


adaptor.ready();