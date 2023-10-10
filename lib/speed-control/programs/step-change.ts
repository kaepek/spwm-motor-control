import { Subject } from "rxjs";
import { parse_args, CliArg, ArgumentHandlers, CliArgType } from "../../../external/kaepek-io/lib/host/ts-adaptors/cli-args.js";
import NetworkAdaptor from "../../../external/kaepek-io/lib/host/ts-adaptors/network.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import fs from "fs";
import { GetStepChange, LineData, SegmentWithStats } from "../tasks/step-change.js";
import { rotation_detector } from "../../rotation-detector.js";
import { GetStartDuty } from "../tasks/get-start-duty.js";
import { run_tasks } from "../../../external/kaepek-io/lib/host/ts-adaptors/task-runner.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { ASCIIParser } from "../../../external/kaepek-io/lib/host/ts-adaptors/ascii-parser.js";

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


const parsed_args = parse_args("StepChange", cli_args, ArgumentHandlers) as any;

console2.log("parsed args", parsed_args);

const word_sender = new SendWord(parsed_args.command_address, parsed_args.command_port, parsed_args.command_protocol);

const adaptor = new NetworkAdaptor(parsed_args.incoming_address, parsed_args.incoming_port, parsed_args.incoming_protocol, parsed_args.input_config_file, ",", parsed_args.outgoing_address, parsed_args.outgoing_port, parsed_args.outgoing_protocol);

const input_config_file_data = JSON.parse(fs.readFileSync(parsed_args.input_config_file).toString());

const outgoing_data_config = input_config_file_data.inputs;

adaptor.outgoing_data_config = outgoing_data_config;

const outgoing_data_config_with_extensions = [
    ...outgoing_data_config,
    {"name": "steady_region1", "position":17},
    {"name": "transition_region1", "position":18},
    {"name": "steady_region2", "position":19},
    {"name": "transition_region2", "position":20}
];

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
        segments: SegmentWithStats[],
        start_duty: number
    },
    ccw: {
        segments: SegmentWithStats[],
        start_duty: number
    }
}


const steady_format = [
    {"name": "duty", "position": 0},
    {"name": "mean_velocity", "position": 1},
    {"name": "std_velocity", "position": 2}

];
const transition_format = [
    {"name": "duty", "position": 0},
    {"name": "transition_time", "position": 1}
];

const detections_parser = new ASCIIParser(outgoing_data_config_with_extensions, ",");
const steady_parser = new ASCIIParser(steady_format, ",");
const transition_parser = new ASCIIParser(transition_format, ",");

run_tasks(tasks, adaptor).then((output: StepChangeOuput) => {

    console.log(output);

    const output_flat: {cw: LineData[], ccw: LineData[]} = {cw: [], ccw:[]};

    output.cw.segments.forEach((segment) => {
        output_flat.cw = output_flat.cw.concat(segment.data);
    });

    if (output.ccw) output.ccw.segments.forEach((segment) => {
        output_flat.ccw = output_flat.ccw.concat(segment.data);
    });

    // now parse this to lines

    const output_lines: {cw: Array<string>, ccw: Array<string>} = {cw: [], ccw: []};
    output_flat.cw.forEach((line) => {
        const str_line = detections_parser.serialise(line);
        output_lines.cw.push(str_line);
    });
    output_flat.ccw.forEach((line) => {
        const str_line = detections_parser.serialise(line);
        output_lines.ccw.push(str_line);
    });

    const cw_lines_output = output_lines.cw.join("\n");
    const ccw_lines_output = output_lines.ccw.join("\n");

    // extract the stable/transition region stats.

    const stats: {cw: any, ccw: any} = {
        cw: [],
        ccw: []
    };

    output.cw.segments.forEach((segment) => {
        const type = segment.type;
        if (segment.type === "steady") {
            stats.cw.push({type, duty: segment.duty, max_velocity: segment.max_velocity, min_velocity: segment.min_velocity, mean_velocity: segment.mean_velocity, std_velocity: segment.std_velocity});
        }
        else {
            const last_segment_element = segment.data[segment.data.length - 1];
            const first_segment_element = segment.data[0];
            const max_time = last_segment_element.time;
            const min_time = first_segment_element.time;
            const transition_time = Math.abs(max_time - min_time);
            const duty = segment.duty;
            stats.cw.push({type, duty, max_time, min_time, transition_time});
        }
    });

    if (output.ccw) {
        output.ccw.segments.forEach((segment) => {
            const type = segment.type;
            if (segment.type === "steady") {
                stats.ccw.push({type, duty: segment.duty, max_velocity: segment.max_velocity, min_velocity: segment.min_velocity, mean_velocity: segment.mean_velocity, std_velocity: segment.std_velocity});
            }
            else {
                const last_segment_element = segment.data[segment.data.length - 1];
                const first_segment_element = segment.data[0];
                const duty = segment.duty;
                const max_time = last_segment_element.time;
                const min_time = first_segment_element.time;
                const transition_time = Math.abs(max_time - min_time);
                stats.ccw.push({type, duty, max_time, min_time, transition_time});
            }
        });
    }

    const charts: {cw: {steady: Array<string>, transition: Array<string>}, ccw: {steady: Array<string>, transition: Array<string>}} = {cw: {steady: [], transition: []}, ccw: {steady: [], transition: []}};

    /*
const steady_format = [
    {"name": "duty", "position": 0},
    {"name": "mean_velocity", "position": 1},
    {"name": "std_velocity", "position": 2}

];
const transition_format = [
    {"name": "duty", "position": 0},
    {"name": "transition_time", "position": 1}
];
    */

    stats.cw.forEach((stat: any) => {
        if (stat.type === "steady") {
            charts.cw.steady.push(steady_parser.serialise(stat));
        }   
        else {
            charts.cw.transition.push(transition_parser.serialise(stat));
        }
    });
    stats.ccw.forEach((stat: any) => {
        if (stat.type === "steady") {
            charts.ccw.steady.push(steady_parser.serialise(stat));
        }   
        else {
            charts.ccw.transition.push(transition_parser.serialise(stat));
        }
    });

    console2.success("All finished, result:", JSON.stringify(output));
    // write file
    if (parsed_args.hasOwnProperty("output_data_file")) {
        fs.writeFileSync(parsed_args.output_data_file, JSON.stringify(output, null, 4));
        fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".detections.cw.csv"), cw_lines_output);
        fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".detections.ccw.csv"), ccw_lines_output);
        // write stats
        fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".stats.json"), JSON.stringify(stats, null, 4));
        // write charts
        fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".transition.cw.csv"), charts.cw.transition.join("\n"));
        fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".transition.ccw.csv"), charts.ccw.transition.join("\n"));
        fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".steady.cw.csv"), charts.cw.steady.join("\n"));
        fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, ".steady.ccw.csv"), charts.ccw.steady.join("\n"));
    }
    process.exit(0);
}).catch((err) => {
    console2.error(err);
    if (err.hasOwnProperty("stack")) {console2.error(err.stack)};
    process.exit(1);
});
