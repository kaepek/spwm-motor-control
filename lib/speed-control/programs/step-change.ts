import { Subject } from "rxjs";
import { parse_args, CliArg, ArgumentHandlers, CliArgType } from "../../../external/kaepek-io/lib/host/ts-adaptors/cli-args.js";
import NetworkAdaptor from "../../../external/kaepek-io/lib/host/ts-adaptors/network.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import fs from "fs";
import { GetStepChange, SegmentWithStats } from "../tasks/step-change.js";
import { rotation_detector } from "../../rotation-detector.js";
import { GetStartDuty } from "../tasks/get-start-duty.js";
import { run_tasks } from "../../../external/kaepek-io/lib/host/ts-adaptors/task-runner.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";

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

const outgoing_data_config = parsed_args.input_config_file

adaptor.outgoing_data_config = outgoing_data_config;

adaptor.incoming_data$.subscribe((line_data) => {
    adaptor.transmit_outgoing_data(line_data.parsed_data);
});

const cw_rotation$ = rotation_detector(adaptor.incoming_data$, true);
const ccw_rotation$ = rotation_detector(adaptor.incoming_data$, false);

const cw_get_start_duty_task = new GetStartDuty(cw_rotation$, word_sender, "cw");
const cw_get_step_change_task = new GetStepChange(adaptor.incoming_data$, word_sender, "cw");

const tasks = [cw_get_start_duty_task, cw_get_step_change_task];

type StepChangeOuput = {
    cw: {
        segments: SegmentWithStats,
        start_duty: number
    },
    ccw: {
        segments: SegmentWithStats,
        start_duty: number
    }
}

run_tasks(tasks, adaptor).then((output: StepChangeOuput) => {
    console2.success("All finished, result:", JSON.stringify(output));
    // write file
    if (parsed_args.hasOwnProperty("output_data_file")) {
        // fs.writeFileSync(parsed_args.output_data_file, JSON.stringify(output));
        // fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".cpp"), cpp_ac_map);
    }
    process.exit(0);
}).catch((err) => {
    console2.error(err);
    if (err.hasOwnProperty("stack")) {console2.error(err.stack)};
    process.exit(1);
});
