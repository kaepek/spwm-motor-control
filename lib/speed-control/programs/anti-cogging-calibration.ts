import NetworkAdaptor from "../../../external/kaepek-io/lib/host/ts-adaptors/network.js";
import { parseArgs } from "node:util";
import fs from "fs";
import { rotation_detector } from "../../rotation-detector.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { GetStartDuty } from "../tasks/get-start-duty.js";
import { run_tasks } from "../../../external/kaepek-io/lib/host/ts-adaptors/task-runner.js";
import { parse_args, CliArg, ArgumentHandlers, CliArgType } from "../../../external/kaepek-io/lib/host/ts-adaptors/cli-args.js";

const cli_args: Array<CliArg> = [
    {
        name: "input_config_file",
        type: CliArgType.InputJSONFilePath, // InputJSONFilePathArgumentHandler
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

const parse_options: any = {
    options: {
        input_config_file: {
            type: "string",
            short: "c"
        },
        output_data_file: {
            type: "string",
            short: "o"
        },
        incoming_address: {
            type: "string",
            short: "a"
        },
        incoming_port: {
            type: "string",
            short: "p"
        },
        incoming_protocol: {
            type: "string",
            short: "n"
        },
        outgoing_address: {
            type: "string",
            short: "o",
        },
        outgoing_port: {
            type: "string",
            short: "x"
        },
        outgoing_protocol: {
            type: "string",
            short: "v"
        },
        command_address: {
            type: "string",
            short: "y",
        },
        command_port: {
            type: "string",
            short: "u"
        },
        command_protocol: {
            type: "string",
            short: "m"
        },
    }
};

const parsed_args_2 = parse_args("AntiCoggingCalibration", cli_args, ArgumentHandlers);
console.log("parsed_args_2", parsed_args_2);
process.exit(1);

let parsed_options: any = { values: {}, positionals: [] };

try {
    parsed_options = parseArgs(parse_options as any);
}
catch (e: any) {
    console.error(`AntiCoggingCalibration argument parser error: ${e.message}`);
    process.exit(1);
}

const missing_options: Array<string> = [];
Object.keys(parse_options.options).forEach((option_name) => {
    if (!parsed_options.values[option_name] && ((option_name !== "outgoing_address") && (option_name !== "outgoing_port") && (option_name !== "outgoing_protocol"))) {
        missing_options.push(option_name);
    }
});

if (missing_options.length !== 0) {
    console.error(`AntiCoggingCalibration: Missing the following arguments ${missing_options.map(option_str => {
        const option = (parse_options.options)[option_str];
        return `--${option_str} or -${option.short}`
    }).join(", ")}`);
    process.exit(1);
}

const values = parsed_options.values;

// check if files exists
const cwd = process.cwd();

const full_output_data_path = `${cwd}/${values["output_data_file"]}`;
const output_data_path_exists = fs.existsSync(full_output_data_path);
if (output_data_path_exists) {
    console.error(`AntiCoggingCalibration: output_data_file ${full_output_data_path} exists already.`);
    process.exit(1);
}


const full_input_config_path = `${cwd}/${values["input_config_file"]}`;
const input_config_exists = fs.existsSync(full_input_config_path);
if (!input_config_exists) {
    console.error(`AntiCoggingCalibration: input_config_file ${full_input_config_path} does not exist`);
    process.exit(1);
}

const outgoing_address = values["outgoing_address"] || null;
let outgoing_port = values["outgoing_port"] || null;
if (outgoing_port !== null) {
    outgoing_port = parseFloat(outgoing_port);
}
const outgoing_protocol = values["outgoing_protocol"];

const command_address = values["command_address"];
const command_port = parseFloat(values["command_port"]);
const command_protocol = values["command_protocol"];

if (command_protocol !== "udp") {
    console.error(`AntiCoggingCalibration: unknown command_protocol ${command_protocol} expects 'upd'`);
    process.exit(1);
}

// console.log("VALUEs", values);


const word_sender = new SendWord(command_address, command_port, command_protocol);

// console.log("about to init network adaptor", values);
const adaptor = new NetworkAdaptor(values["incoming_address"], parseFloat(values["incoming_port"]), values["incoming_protocol"], values["input_config_file"], ",", outgoing_address, outgoing_port, outgoing_protocol);

const rotation$ = rotation_detector(adaptor.incoming_data$, true);

// what tasks do we need for this program
const get_start_duty_task = new GetStartDuty(rotation$, word_sender);

const tasks = [get_start_duty_task];

// need to parse state to each task when we are running it

run_tasks(tasks, adaptor).then(output => {
    console2.success("All finished, result:", JSON.stringify(output));
    process.exit(0);
}).catch((err) => {
    console2.error(err);
    process.exit(1);
});
