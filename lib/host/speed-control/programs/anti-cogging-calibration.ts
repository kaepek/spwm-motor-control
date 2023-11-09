import NetworkAdaptor from "../../../../external/kaepek-io/lib/host/controller/utils/network.js";
import fs from "fs";
import { rotation_detector } from "../../rotation-detector.js";
import { SendWord } from "../../../../external/kaepek-io/lib/host/controller/utils/send-word.js";
import { console2 } from "../../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { GetStartDuty } from "../tasks/get-start-duty.js";
import { run_tasks } from "../../../../external/kaepek-io/lib/host/controller/utils/task-runner.js";
import { parse_args, CliArg, ArgumentHandlers, CliArgType } from "../../../../external/kaepek-io/lib/host/controller/utils/cli-args.js";
import { GetIdleDuty } from "../tasks/get-min-duty.js";
import { CollectAccelerationData, ACMap } from "../tasks/collect-acceleration-data.js";
import { generate_ac_map_cpp } from "../utils/cpp-ac-state-map-generator.js";
import { delay } from "../utils/delay.js";
import { SetIdleDuty } from "../tasks/set-idle.js";

const cli_args: Array<CliArg> = [
    {
        name: "input_config_file",
        type: CliArgType.InputFilePath, // InputJSONFilePathArgumentHandler
        short: "c",
        help: "Relative file path to indicate which config file the program will use to parse the data coming from the microcontroller program via the kaepek-io-director",
        required: true
    },
    {
        name: "output_data_file",
        type: CliArgType.OutputFilePath,
        short: "o",
        help: "Relative file path to indicate where to output the analysis files from this program.",
        required: false
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
        name: "command_address",
        type: CliArgType.String,
        short: "y",
        help: "The command host address, indicates which host this program will send word commands to the kaepek-io-director program.",
        required: false,
        default: "localhost",
        group: "command"
    },
    {
        name: "command_port",
        type: CliArgType.Number,
        short: "u",
        help: "The command host port, indicates which port this program will use to send word commands to the kaepek-io-director program.",
        required: false,
        default: 9000,
        group: "command"
    },
    {
        name: "command_protocol",
        type: CliArgType.String,
        short: "m",
        help: "The command host protocol, indicates which protocol this program will use to send word commands to the kaepek-io-director program.",
        required: false,
        default: "udp",
        group: "command"
    },
    {
        name: "duty_max",
        type: CliArgType.Number,
        short: "d",
        help: "The maximum internally supported duty value of the microcontroller.",
        required: false,
        default: 2047
    },
    {
        name: "angular_steps",
        type: CliArgType.Number,
        short: "b",
        help: "The number of angular steps that the rotary encoder has.",
        default: 16384,
        required: false
    },
    {
        name: "angular_compression_ratio",
        type: CliArgType.Number,
        short: "r",
        help: "The angular compression ratio, as read from ENCODER_VALUE_COMPRESSION within the microcontroller.",
        required: true
    },
    {
        name: "bin_population_threshold",
        type: CliArgType.Number,
        short: "t",
        help: "The velocity data points needed per each compressed angular step required before data collection is completed.",
        default: 20,
        required: false
    }
];


const parsed_args = parse_args("AntiCoggingCalibration", cli_args, ArgumentHandlers) as any;

const duty_max = parsed_args.duty_max;
const angular_steps = parsed_args.angular_steps;
const bin_population_threshold = parsed_args.bin_population_threshold;

const word_sender = new SendWord(parsed_args.command_address, parsed_args.command_port, parsed_args.command_protocol);

// console.log("about to init network adaptor", values);
const adaptor = new NetworkAdaptor(parsed_args.incoming_address, parsed_args.incoming_port, parsed_args.incoming_protocol, parsed_args.input_config_file, ",", parsed_args.outgoing_address, parsed_args.outgoing_port, parsed_args.outgoing_protocol);

adaptor.outgoing_data_config = adaptor.incoming_data_config;

adaptor.incoming_data$.subscribe((line_data) => {
    adaptor.transmit_outgoing_data(line_data.parsed_data);
});

const cw_rotation$ = rotation_detector(adaptor.incoming_data$, true);
const ccw_rotation$ = rotation_detector(adaptor.incoming_data$, false);

// what tasks do we need for this program
const cw_get_start_duty_task1 = new GetStartDuty(cw_rotation$, word_sender, "cw");
const cw_get_start_duty_task2 = new GetStartDuty(cw_rotation$, word_sender, "cw");
const cw_get_idle_duty_task = new GetIdleDuty(cw_rotation$, word_sender, "cw");
const cw_set_idle_duty_task = new SetIdleDuty(ccw_rotation$, word_sender, "cw");
const cw_collect_acceleration_data = new CollectAccelerationData(cw_rotation$, word_sender, "cw", duty_max, angular_steps, parsed_args.angular_compression_ratio, bin_population_threshold);

const despin_task = new SetIdleDuty(ccw_rotation$, word_sender, "ccw", 0);

const ccw_get_start_duty_task1 = new GetStartDuty(ccw_rotation$, word_sender, "ccw");
const ccw_get_start_duty_task2 = new GetStartDuty(ccw_rotation$, word_sender, "ccw");
const ccw_get_idle_duty_task = new GetIdleDuty(ccw_rotation$, word_sender, "ccw");
const ccw_set_idle_duty_task = new SetIdleDuty(ccw_rotation$, word_sender, "ccw");
const ccw_collect_acceleration_data = new CollectAccelerationData(ccw_rotation$, word_sender, "ccw", duty_max, angular_steps, parsed_args.angular_compression_ratio, bin_population_threshold);

const tasks = [ccw_get_start_duty_task1, ccw_get_start_duty_task2, ccw_get_idle_duty_task, ccw_set_idle_duty_task, ccw_collect_acceleration_data, despin_task, cw_get_start_duty_task1, cw_get_start_duty_task2, cw_get_idle_duty_task, cw_set_idle_duty_task, cw_collect_acceleration_data];

// need to parse state to each task when we are running it
const start_time = Date.now();
run_tasks(tasks, adaptor).then(async (output: ACMap) => {
    await delay(300);
    await word_sender.send_word("thrustui16", 0);
    await delay(300);
    await word_sender.send_word("stop");
    const cpp_ac_map = generate_ac_map_cpp(output);
    console2.success("All finished");
    // write file
    if (parsed_args.hasOwnProperty("output_data_file")) {
        fs.writeFileSync(parsed_args.output_data_file, JSON.stringify(output));
        fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".cpp"), cpp_ac_map);
    }
    const end_time = Date.now();
    console2.info(`Finished in ${(end_time-start_time)/1000} [s]`);
    process.exit(0);
}).catch(async (err) => {
    await delay(300);
    await word_sender.send_word("thrustui16", 0);
    await delay(300);
    await word_sender.send_word("stop");
    console2.error(err);
    if (err.hasOwnProperty("stack")) {console2.error(err.stack)};
    process.exit(1);
});
