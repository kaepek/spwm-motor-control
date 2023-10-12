import NetworkAdaptor from "../../../external/kaepek-io/lib/host/controller/utils/network.js";
import { parseArgs } from "node:util";
import fs from "fs";
import { rotation_detector } from "../../rotation-detector.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/controller/utils/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { GetStartDuty } from "../tasks/get-start-duty.js";
import { run_tasks } from "../../../external/kaepek-io/lib/host/controller/utils/task-runner.js";
import { parse_args, CliArg, ArgumentHandlers, CliArgType } from "../../../external/kaepek-io/lib/host/controller/utils/cli-args.js";
import { GetIdleDuty } from "../tasks/get-min-duty.js";
import { CollectAccelerationData, ACMap } from "../tasks/collect-acceleration-data.js";
import { generate_ac_map_cpp } from "../utils/cpp-ac-state-map-generator.js";
import { delay } from "../utils/delay.js";

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

adaptor.outgoing_data_config = adaptor.incoming_data_config;

adaptor.incoming_data$.subscribe((line_data) => {
    adaptor.transmit_outgoing_data(line_data.parsed_data);
});

const cw_rotation$ = rotation_detector(adaptor.incoming_data$, true);
const ccw_rotation$ = rotation_detector(adaptor.incoming_data$, false);

// what tasks do we need for this program
const cw_get_start_duty_task = new GetStartDuty(cw_rotation$, word_sender, "cw");
const cw_get_idle_duty_task = new GetIdleDuty(cw_rotation$, word_sender, "cw");
const cw_collect_acceleration_data = new CollectAccelerationData(cw_rotation$, word_sender, "cw");
const ccw_get_start_duty_task = new GetStartDuty(ccw_rotation$, word_sender, "ccw");
const ccw_get_idle_duty_task = new GetIdleDuty(ccw_rotation$, word_sender, "ccw");
const ccw_collect_acceleration_data = new CollectAccelerationData(ccw_rotation$, word_sender, "ccw");

const tasks = [ccw_get_start_duty_task, ccw_get_idle_duty_task, ccw_collect_acceleration_data, cw_get_start_duty_task, cw_get_idle_duty_task, cw_collect_acceleration_data];

// need to parse state to each task when we are running it

run_tasks(tasks, adaptor).then((output: ACMap) => {
    const cpp_ac_map = generate_ac_map_cpp(output);
    console2.success("All finished, result:", JSON.stringify(output));
    // write file
    if (parsed_args.hasOwnProperty("output_data_file")) {
        fs.writeFileSync(parsed_args.output_data_file, JSON.stringify(output));
        fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".cpp"), cpp_ac_map);
    }
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
