import { Subject } from "rxjs";
import { parse_args, CliArg, ArgumentHandlers, CliArgType } from "../../../external/kaepek-io/lib/host/ts-adaptors/cli-args.js";
import NetworkAdaptor from "../../../external/kaepek-io/lib/host/ts-adaptors/network.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import fs from "fs";
import { StepChange } from "../tasks/step-change.js";

/**
 * Brief run the get start duty for both directions. Do this apriori.
 * 
 * We should start at zero duty / setpoint and call this stable.
 * Next we should go to the start duty.
 * We then proceed with n - 1 steps up until but including the max duty.
 * 
 * We start with zero thrust / setpoint and assume this to be a "stable segment" of the analysis.
 * We record for some number of seconds (stable time)
 * We then apply the next word and move to a "transtional segment"
 * 
 * To process the data we need to use the double buffer technique to get the error of the error of the acceleration.
 * Then we look for detections of the minimum of the error ^2 of the acceleration.
 * 
 * 
 * We define 
 */

const cli_args: Array<CliArg> = [
    {
        name: "input_config_file",
        type: CliArgType.InputFilePath, // InputJSONFilePathArgumentHandler
        short: "c",
        required: true
    },
    {
        name: "output_data_file",
        type: CliArgType.OutputFilePath,
        short: "o",
        required: false
    },
    {
        name: "incoming_address",
        type: CliArgType.String,
        short: "a",
        required: true,
        group: "incoming"
    },
    {
        name: "incoming_port",
        type: CliArgType.Number,
        short: "p",
        required: true,
        group: "incoming"
    },
    {
        name: "incoming_protocol",
        type: CliArgType.String,
        short: "n",
        required: true,
        group: "incoming"
    },
    {
        name: "outgoing_address",
        type: CliArgType.String,
        short: "s",
        required: false,
        group: "outgoing"
    },
    {
        name: "outgoing_port",
        type: CliArgType.Number,
        short: "x",
        required: false,
        group: "outgoing"
    },
    {
        name: "outgoing_protocol",
        type: CliArgType.String,
        short: "v",
        required: false,
        group: "outgoing"
    },
    {
        name: "command_address",
        type: CliArgType.String,
        short: "y",
        required: true,
        group: "command"
    },
    {
        name: "command_port",
        type: CliArgType.Number,
        short: "u",
        required: true,
        group: "command"
    },
    {
        name: "command_protocol",
        type: CliArgType.String,
        short: "m",
        required: true,
        group: "command"
    }
];


const parsed_args = parse_args("AntiCoggingCalibration", cli_args, ArgumentHandlers) as any;

const word_sender = new SendWord(parsed_args.command_address, parsed_args.command_port, parsed_args.command_protocol);

const adaptor = new NetworkAdaptor(parsed_args.incoming_address, parsed_args.incoming_port, parsed_args.incoming_protocol, parsed_args.input_config_file, ",", parsed_args.outgoing_address, parsed_args.outgoing_port, parsed_args.outgoing_protocol);

// extend the incoming data format with the extra fields we will add via analysis.
const outgoing_data_config = [
    ...parsed_args.input_config_file,
    { "name": "smoothed_future_kalman_acceleration", "position": 17 },
    { "name": "smoothed_future_kalman_jerk", "position": 18 },
    { "name": "smoothed_prior_kalman_acceleration", "position": 19 },
    { "name": "smoothed_prior_kalman_jerk", "position": 20 },
    { "name": "smoothed_central_kalman_acceleration", "position": 21 },
    { "name": "smoothed_central_kalman_jerk", "position": 22 },
    { "name": "smoothed_prior_kalman_acceleration_std", "position": 23 },
    { "name": "smoothed_prior_kalman_jerk_std", "position": 24 },
    { "name": "smoothed_future_kalman_acceleration_std", "position": 25 },
    { "name": "smoothed_future_kalman_jerk_std", "position": 26 },
    { "name": "smoothed_central_kalman_acceleration_std", "position": 27 },
    { "name": "smoothed_central_kalman_jerk_std", "position": 28 },
    { "name": "smoothed2_future_kalman_acceleration", "position": 29 },
    { "name": "smoothed2_future_kalman_jerk", "position": 30 },
    { "name": "smoothed2_prior_kalman_acceleration", "position": 31 },
    { "name": "smoothed2_prior_kalman_jerk", "position": 32 },
    { "name": "smoothed2_central_kalman_acceleration", "position": 33 },
    { "name": "smoothed2_central_kalman_jerk", "position": 22 },
    { "name": "smoothed2_prior_kalman_acceleration_std", "position": 34 },
    { "name": "smoothed2_prior_kalman_jerk_std", "position": 24 },
    { "name": "smoothed2_future_kalman_acceleration_std", "position": 35 },
    { "name": "smoothed2_future_kalman_jerk_std", "position": 36 },
    { "name": "smoothed2_central_kalman_acceleration_std", "position": 37 },
    { "name": "smoothed2_central_kalman_jerk_std", "position": 38 }
];

adaptor.outgoing_data_config = outgoing_data_config;
// adaptor.incoming_data$.

/*adaptor.incoming_data$.subscribe((line_data) => {
    adaptor.transmit_outgoing_data(line_data.parsed_data);
});*/

// const cw_get_start_duty_task = new GetStartDuty(cw_rotation$, word_sender, "cw");

// StepChange