import NetworkAdaptor from "../../../external/kaepek-io/lib/host/ts-adaptors/network.js";
import { parseArgs } from "node:util";
import fs from "fs";
import { rotation_detector } from "../../rotation-detector.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { GetStartDuty } from "../tasks/get-start-duty.js";
import { run_tasks } from "../../../external/kaepek-io/lib/host/ts-adaptors/task-runner.js";
import { parse_args, CliArg, ArgumentHandlers, CliArgType } from "../../../external/kaepek-io/lib/host/ts-adaptors/cli-args.js";
import { GetIdleDuty } from "../tasks/get-min-duty.js";

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

// console.log("about to init network adaptor", values);
const adaptor = new NetworkAdaptor(parsed_args.incoming_address, parsed_args.incoming_port, parsed_args.incoming_protocol, parsed_args.input_config_file, ",", parsed_args.outgoing_address, parsed_args.outgoing_port, parsed_args.outgoing_protocol);

const rotation$ = rotation_detector(adaptor.incoming_data$, true);

// what tasks do we need for this program
const get_start_duty_task = new GetStartDuty(rotation$, word_sender);
const get_idle_duty_task = new GetIdleDuty(rotation$, word_sender);

const tasks = [get_start_duty_task, get_idle_duty_task];

// need to parse state to each task when we are running it

run_tasks(tasks, adaptor).then(output => {
    console2.success("All finished, result:", JSON.stringify(output));
    process.exit(0);
}).catch((err) => {
    console2.error(err);
    process.exit(1);
});
