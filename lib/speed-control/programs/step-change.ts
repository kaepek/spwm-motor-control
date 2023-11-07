import { parse_args, CliArg, ArgumentHandlers, CliArgType } from "../../../external/kaepek-io/lib/host/controller/utils/cli-args.js";
import NetworkAdaptor from "../../../external/kaepek-io/lib/host/controller/utils/network.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/controller/utils/send-word.js";
import fs from "fs";
import { GetStepChange, LineData, SegmentWithStats, SteadySegmentWithStats } from "../tasks/step-change.js";
import { rotation_detector } from "../../rotation-detector.js";
import { GetStartDuty } from "../tasks/get-start-duty.js";
import { run_tasks } from "../../../external/kaepek-io/lib/host/controller/utils/task-runner.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { ASCIIParser } from "../../../external/kaepek-io/lib/host/controller/utils/ascii-parser.js";
import { delay } from "../utils/delay.js";
import { GetIdleDuty } from "../tasks/get-min-duty.js";
import { SetIdleDuty } from "../tasks/set-idle.js";
import regression from 'regression';

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
        name: "duty_cap_multiplier",
        type: CliArgType.Number,
        short: "d",
        help: "The duty cap multiplier, represents the microcontrollers internal duty cap on the with a range 0.0 -> 1.0, 1.0 represents 100% of the possible duty authority.",
        required: true
    },
    {
        name: "duty_max",
        type: CliArgType.Number,
        short: "q",
        help: "The maximum internally supported duty value of the microcontroller.",
        required: false,
        default: 2047
    },
    {
        name: "duty_end",
        type: CliArgType.Number,
        short: "e",
        help: "The duty to end the step change program with.",
        required: false,
        default: 2047
    },
    {
        name: "duty_begin",
        type: CliArgType.StringOrNumber,
        short: "b",
        required: false,
        help: "The duty to start the step change program with. Several possible values: either 'idle' which represents the lowest possible speed the motor will reliably turn, 'start' which represents the startup speed required to overcome sticktion etc, or a user provided numerical value e.g. 10. Note the program will bring the motor up to the idle speed before applying the user selected values.",
        default: "idle"
    },
    {
        name: "stable_region_tolerance_percentage",
        type: CliArgType.Number,
        short: "t",
        required: false,
        help: "The percentage tolerance allowing values to be permitted into the stable region from the transition region. Aka if 1, values in the transition region which are +/-1% will be allowed into the stable region, the first failure of out of bounds marks the the boundary of the transition/stable region.",
        default: 1
    },
    {
        name: "wait_time",
        type: CliArgType.Number,
        help: "During a transition. The [wait_time] is the number of milliseconds to wait between while not seeing a lower acceleration before we assume we are in a stable velocity region and the transition has concluded.",
        short: "w",
        required: false,
        default: 3000
    },
    {
        name: "number_duty_steps",
        type: CliArgType.Number,
        help: "The number of duty steps to apply between (inclusive) the duty_begin and duty_end values.",
        short: "f",
        required: false,
        default: 10
    }
];
  
const parsed_args = parse_args("StepChange", cli_args, ArgumentHandlers) as any;

const duty_multiplier = parsed_args.duty_cap_multiplier;

const word_sender = new SendWord(parsed_args.command_address, parsed_args.command_port, parsed_args.command_protocol);

const adaptor = new NetworkAdaptor(parsed_args.incoming_address, parsed_args.incoming_port, parsed_args.incoming_protocol, parsed_args.input_config_file, ",", parsed_args.outgoing_address, parsed_args.outgoing_port, parsed_args.outgoing_protocol);

const input_config_file_data = JSON.parse(fs.readFileSync(parsed_args.input_config_file).toString());

const outgoing_data_config = input_config_file_data.inputs;

adaptor.outgoing_data_config = outgoing_data_config;

const outgoing_data_config_with_extensions = [
    ...outgoing_data_config,
    { "name": "steady_region1", "position": 17 },
    { "name": "transition_region1", "position": 18 },
    { "name": "steady_region2", "position": 19 },
    { "name": "transition_region2", "position": 20 },
    { "name": "dead_region", "position": 21 }
];

adaptor.incoming_data$.subscribe((line_data) => {
    adaptor.transmit_outgoing_data(line_data.parsed_data);
});

const cw_rotation$ = rotation_detector(adaptor.incoming_data$, true);
const ccw_rotation$ = rotation_detector(adaptor.incoming_data$, false);

const cw_get_start_duty_task1 = new GetStartDuty(cw_rotation$, word_sender, "cw");
const cw_get_start_duty_task2 = new GetStartDuty(cw_rotation$, word_sender, "cw");
const cw_get_idle_duty_task = new GetIdleDuty(cw_rotation$, word_sender, "cw");
const cw_set_idle_duty_task = new SetIdleDuty(cw_rotation$, word_sender, "cw");
const cw_get_step_change_task = new GetStepChange(adaptor.incoming_data$, word_sender, "cw", parsed_args.duty_max, parsed_args.number_duty_steps, parsed_args.wait_time, parsed_args.stable_region_tolerance_percentage, parsed_args.duty_end, parsed_args.duty_begin);

const despin_task = new SetIdleDuty(cw_rotation$, word_sender, "cw", 0);
// this might not work if motor is stalled.... need to fixme this.

const ccw_get_start_duty_task1 = new GetStartDuty(ccw_rotation$, word_sender, "ccw");
const ccw_get_start_duty_task2 = new GetStartDuty(ccw_rotation$, word_sender, "ccw");

const ccw_get_idle_duty_task = new GetIdleDuty(ccw_rotation$, word_sender, "ccw");
const ccw_set_idle_duty_task = new SetIdleDuty(ccw_rotation$, word_sender, "ccw");
const ccw_get_step_change_task = new GetStepChange(adaptor.incoming_data$, word_sender, "ccw",  parsed_args.duty_max, parsed_args.number_duty_steps, parsed_args.wait_time, parsed_args.stable_region_tolerance_percentage, parsed_args.duty_end, parsed_args.duty_begin);

const tasks = [cw_get_start_duty_task1, cw_get_start_duty_task2, cw_get_idle_duty_task, cw_set_idle_duty_task, cw_get_step_change_task, despin_task, ccw_get_start_duty_task1, ccw_get_start_duty_task2, ccw_get_idle_duty_task, ccw_set_idle_duty_task, ccw_get_step_change_task];

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

type SteadyStat = {
    type: "steady"
    duty: number,
    max_velocity: number,
    min_velocity: number,
    mean_velocity: number,
    std_velocity: number
};

type TransitionStat = {
    type: "transition",
    duty_prior: number,
    duty: number,
    duty_change: number,
    velocity_prior: number,
    velocity_next: number,
    velocity_change: number,
    max_time: number,
    min_time: number,
    transition_time: number,
    dead_time: number
}

type RegressionLines = {
    Duty: number,
    "Speed [Hz]": number,
    "Speed STD": number,
    "Duty percentage / 100%": number,
    "LOG10(Duty percentage / 100%)": number,
    "LOG10(Speed [Hz])": number,
    "Linear model Speed [Hz]": number,
    "Linear Model Speed Error [ΔHz]": number,
    "Power law model LOG10(Speed [Hz])": number, // 
    "Power law model Speed [Hz]": number,
    "Power law model Speed Error [ΔHz]": number,
    "Linear model Duty percentage / 100%": number,
    "Power law model Duty percentage / 100%": number
};

const steady_format = [
    { "name": "duty", "position": 0 },
    { "name": "mean_velocity", "position": 1 },
    { "name": "std_velocity", "position": 2 }

];
const transition_format = [
    { "name": "duty", "position": 0 },
    { "name": "transition_time", "position": 1 },
    { "name": "dead_time", "position": 2 }
];

const detections_parser = new ASCIIParser(outgoing_data_config_with_extensions, ",");
const steady_parser = new ASCIIParser(steady_format, ",");
const transition_parser = new ASCIIParser(transition_format, ",");
const max_possible_duty = 2047;

const start_time = Date.now();
run_tasks(tasks, adaptor).then(async (output: StepChangeOuput) => {
    await delay(300);
    await word_sender.send_word("thrustui16", 0);
    await delay(300);
    await word_sender.send_word("stop");

    const output_flat: { cw: LineData[], ccw: LineData[] } = { cw: [], ccw: [] };

    output.cw.segments.forEach((segment) => {
        output_flat.cw = output_flat.cw.concat(segment.data);
    });

    if (output.ccw) output.ccw.segments.forEach((segment) => {
        output_flat.ccw = output_flat.ccw.concat(segment.data);
    });

    // now parse this to lines

    const output_lines: { cw: Array<string>, ccw: Array<string> } = { cw: [], ccw: [] };
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

    const stats: { cw: Array<SteadyStat | TransitionStat>, ccw: Array<SteadyStat | TransitionStat> } = {
        cw: [],
        ccw: []
    };

    output.cw.segments.forEach((segment, idx) => {

        const type = segment.type;
        if (segment.type === "steady") {
            stats.cw.push({ type, duty: segment.duty, max_velocity: segment.max_velocity, min_velocity: segment.min_velocity, mean_velocity: segment.mean_velocity, std_velocity: segment.std_velocity } as SteadyStat);
        }
        else {
            const last_segment_element = segment.data[segment.data.length - 1];
            const first_segment_element = segment.data[0];
            const max_time = last_segment_element.time;
            const min_time = first_segment_element.time;
            const transition_time = Math.abs(max_time - min_time);
            const dead_time = segment.dead_time;
            const duty = segment.duty;
            const duty_prior = (output.cw.segments[idx - 1] as SteadySegmentWithStats).duty;
            const duty_change = duty - duty_prior;
            const velocity_prior = (output.cw.segments[idx - 1] as SteadySegmentWithStats).mean_velocity;
            const velocity_next = (output.cw.segments[idx + 1] as SteadySegmentWithStats).mean_velocity;
            const velocity_change = velocity_next - velocity_prior;
            stats.cw.push({ type, duty_prior, duty, duty_change, velocity_prior, velocity_next, velocity_change, max_time, min_time, transition_time, dead_time } as TransitionStat);
        }
    });

    if (output.ccw) {
        output.ccw.segments.forEach((segment, idx) => {
            const type = segment.type;
            if (segment.type === "steady") {
                stats.ccw.push({ type, duty: segment.duty, max_velocity: segment.max_velocity, min_velocity: segment.min_velocity, mean_velocity: segment.mean_velocity, std_velocity: segment.std_velocity } as SteadyStat);
            }
            else {
                const last_segment_element = segment.data[segment.data.length - 1];
                const first_segment_element = segment.data[0];
                const duty = segment.duty;
                const max_time = last_segment_element.time;
                const min_time = first_segment_element.time;
                const transition_time = Math.abs(max_time - min_time);
                const dead_time = segment.dead_time;
                const duty_prior = (output.ccw.segments[idx - 1] as SteadySegmentWithStats).duty;
                const duty_change = duty - duty_prior;
                const velocity_prior = (output.ccw.segments[idx - 1] as SteadySegmentWithStats).mean_velocity;
                const velocity_next = (output.ccw.segments[idx + 1] as SteadySegmentWithStats).mean_velocity;
                const velocity_change = velocity_next - velocity_prior;
                stats.ccw.push({ type, duty_prior, duty, duty_change, velocity_prior, velocity_next, velocity_change, max_time, min_time, transition_time, dead_time } as TransitionStat);
            }
        });
    }

    const charts: { cw: { steady: Array<string>, transition: Array<string> }, ccw: { steady: Array<string>, transition: Array<string> } } = { cw: { steady: [], transition: [] }, ccw: { steady: [], transition: [] } };

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

    // ok now do regression
    Object.keys(stats).forEach((direction: string) => {
        const direction_str = direction as "cw" | "ccw";
        const raw_duty_velocity_pairs = [] as Array<Array<number>>;

        // get duty velocity pairs and modify with abs to get speed and normalise duty so that the maximum possible yields a duty percentage of the duty_cap_multiplier
        stats[direction_str].forEach((stat) => {
            if (stat.type === "steady") {
                raw_duty_velocity_pairs.push([stat.duty, stat.mean_velocity, stat.std_velocity]);
            }
        });
        const raw_duty_speed_pairs = raw_duty_velocity_pairs.map((pair) => {
            return [pair[0], Math.abs(pair[1]), pair[2]];
        });
        // max_possible_duty 2047
        // duty_multiplier 0.3
        // so when duty is at 2047 (duty/2047) = 0.3 real duty percentage
        // but (duty/2047) = 1
        // so ((duty/2047) * 0.3) = 0.3 
        const raw_duty_multiplier = ((1 / max_possible_duty) * duty_multiplier);

        // get linear duty speed pairs
        const linear_pairs = raw_duty_speed_pairs.map(pair => {
            return [raw_duty_multiplier * pair[0], pair[1]];
        });

        // get log log duty speed pairs
        const loglog_pairs = linear_pairs.map((pair) => {
            return [Math.log10(pair[0]), Math.log10(pair[1])];
        });

        const linear_result = regression.linear(linear_pairs as any, {precision: 6});
        const linear_result_gradient = linear_result.equation[0]; // m
        const linear_result_y_intercept = linear_result.equation[1]; // c
        const linear_result_gradient_reciprocal = 1 / linear_result_gradient; // 1/m
        const linear_result_neg_intercept_over_gradient = - (linear_result_y_intercept / linear_result_gradient); // -c/m
        const linear_result_r2 = linear_result.r2;
        console.log("linear_result_r2", direction_str, linear_result_r2);
        const linear_raw_equation = "y = mx + c";
        const linear_equation = "speed = m(duty) + c";
        // m value c value
        const linear_equation_rearraged = "duty = (speed/m) - (c/m)";
        // 1/m value -c/m value
        const power_law_result = regression.linear(loglog_pairs as any, {precision: 6});
        const power_law_result_gradient = power_law_result.equation[0]; // m
        const power_law_result_gradient_reciprocal = 1 / power_law_result_gradient;
        const power_law_result_y_intercept = power_law_result.equation[1]; // c
        const power_law_divisor = Math.pow(10.0, power_law_result_y_intercept); //10^c
        const power_law_result_r2 = power_law_result.r2;
        console.log("power_law_result_r2", direction_str, power_law_result_r2);
        const power_law_raw_equation = "log10(y) = m(log10(x)) + log10(c)"
        const power_law_equation = "speed = (10^c)(duty)^m";
        const power_law_equation_rearranged = "duty = (speed/(10^c))^(1/m)";
        // 10^c value m value

        // write csv config file
        // what chars format do we want
        /*
                {"name":"Duty percentage / 100%", "position": 0},
                {"name":"Speed [Hz]", "position": 1},
                {"name":"Speed STD", "position": 2},
                {"name":"Duty percentage / 100%", "position": 3},
                {"name":"LOG10(Duty percentage / 100%)", "position": 4},
                {"name":"LOG10(Speed [Hz])", "position": 5},
                {"name":"Linear model Speed [Hz]", "position": 6},
                {"name":"Linear Model Speed Error [ΔHz]", "position": 7},
                {"name":"Power law model LOG10(Speed [Hz])", "position": 8},
                {"name":"Power law model Speed [Hz]", "position": 9},
                {"name":"Power law model Speed Error [ΔHz]", "position": 10},
                {"name":"Linear model Duty percentage / 100%", "position": 11},
                {"name":"Power law model Duty percentage / 100%", "position": 12}
         */

        // what plots do we want

        // create chart for linear duty with best line fit.
        // create chart for log log duty with best line fit.
        // create linear model vs real speed fit chart
        // create create for power law model vs real speed fit.

        const linear_fit_title = `(Duty percentage / 100%) vs Speed [Hz] and Linear model Speed [Hz] and Linear Model Speed Error [ΔHz]. Raw Equation: ${linear_raw_equation}. Equation: ${linear_equation} m=${linear_result_gradient}, c=${linear_result_y_intercept}, r^2=${linear_result_r2}.`;
        const linear_fit = {
            "name": linear_fit_title,
            "independant_column": "Duty percentage / 100%",
            "dependant_columns": [
                { "name": "Speed [Hz]", "color": "black", "line": false },
                { "name": "Linear model Speed [Hz]", "color": "blue", "scatter": false },
                { "name": "Linear Model Speed Error [ΔHz]", "color": "red", "axis": { "location": "right" } }
            ]
        };

        const linear_fit_rearranged_title = `Speed [Hz] vs (Duty percentage / 100%) and (Linear model Duty percentage / 100%). Rearranged Equation ${linear_equation_rearraged} 1/m=${linear_result_gradient_reciprocal}, -c/m=${linear_result_neg_intercept_over_gradient}, r^2=${linear_result_r2}`;
        const linear_fit_rearranged = {
            "name": linear_fit_rearranged_title,
            "independant_column": "Speed [Hz]",
            "dependant_columns": [
                { "name": "Duty percentage / 100%", "color": "black", "line": false },
                { "name": "Linear model Duty percentage / 100%", "color": "blue", "scatter": false }
            ]
        };

        const power_law_fit_title = `LOG10(Duty percentage / 100%) vs LOG10(Speed [Hz]) and Power law model LOG10(Speed [Hz]). Raw Equation: ${power_law_raw_equation}. Equation: ${power_law_equation} m=${power_law_result_gradient}, c=${power_law_result_y_intercept}, r^2=${power_law_result_r2}.`;
        const power_law_fit = {
            "name": power_law_fit_title,
            "independant_column": "LOG10(Duty percentage / 100%)",
            "dependant_columns": [
                { "name": "LOG10(Speed [Hz])", "color": "black", "line": false },
                { "name": "Power law model LOG10(Speed [Hz])", "color": "blue", "scatter": false },
            ]
        };

        const power_law_fit_model_title = `Duty percentage / 100% vs Speed [Hz] and Power law model Speed [Hz] and Power law model Speed Error [ΔHz]. Raw Equation: ${power_law_raw_equation}. Equation: ${power_law_equation} m=${power_law_result_gradient}, c=${power_law_result_y_intercept}, r^2=${power_law_result_r2}.`;
        const power_law_fit_model = {
            "name": power_law_fit_model_title,
            "independant_column": "Duty percentage / 100%",
            "dependant_columns": [
                { "name": "Speed [Hz]", "color": "black", "line": false },
                { "name": "Power law model Speed [Hz]", "color": "blue", "scatter": false },
                { "name": "Power law model Speed Error [ΔHz]", "color": "red", "axis": { "location": "right" } }
            ]
        };

        const power_law_fit_rearranged_title = `Speed [Hz] vs (Duty percentage / 100%) and (Power law model Duty percentage / 100%). Rearranged Equation ${power_law_equation_rearranged} m=${power_law_result_gradient}, 10^c=${power_law_divisor}, r^2=${power_law_result_r2}.`;
        const power_law_fit_rearranged = {
            "name": power_law_fit_rearranged_title,
            "independant_column": "Speed [Hz]",
            "dependant_columns": [
                { "name": "Duty percentage / 100%", "color": "black", "line": false },
                { "name": "Power law model Duty percentage / 100%", "color": "blue", "scatter": false }
            ]
        };

        const regression_config = {
            inputs: [
                { "name": "Duty", "position": 0 },
                { "name": "Speed [Hz]", "position": 1 },
                { "name": "Speed STD", "position": 2 },
                { "name": "Duty percentage / 100%", "position": 3 },
                { "name": "LOG10(Duty percentage / 100%)", "position": 4 },
                { "name": "LOG10(Speed [Hz])", "position": 5 },
                { "name": "Linear model Speed [Hz]", "position": 6 },
                { "name": "Linear Model Speed Error [ΔHz]", "position": 7 },
                { "name": "Power law model LOG10(Speed [Hz])", "position": 8 },
                { "name": "Power law model Speed [Hz]", "position": 9 },
                { "name": "Power law model Speed Error [ΔHz]", "position": 10 },
                { "name": "Linear model Duty percentage / 100%", "position": 11 },
                { "name": "Power law model Duty percentage / 100%", "position": 12 }
            ],
            plots: [
                {
                    "name": "Models vs Experimental data comparison: (Duty percentage / 100%) vs Speed [Hz] and Linear model Speed [Hz] and Power law model Speed [Hz]",
                    "independant_column": "Duty percentage / 100%",
                    "dependant_columns": [
                        { "name": "Speed [Hz]", "color": "black", "line": false },
                        { "name": "Linear model Speed [Hz]", "color": "blue", "scatter": false },
                        { "name": "Power law model Speed [Hz]", "color": "red", "scatter": false }
                    ]
                },
                linear_fit,
                linear_fit_rearranged,
                power_law_fit,
                power_law_fit_model,
                power_law_fit_rearranged
            ]
        };

        // Now need to build the RegressionLines data needed to fulfil the needed graphs: 
        const regression_lines: Array<RegressionLines> = [];

        raw_duty_speed_pairs.forEach((value, idx) => {
            const duty = value[0];
            const speed = value[1];
            const speed_std = value[2];
            const duty_div_100pc = linear_pairs[idx][0];
            const log10_duty_div_100pc = loglog_pairs[idx][0];
            const log10_speed = loglog_pairs[idx][1];
            const linear_model_speed = linear_result_gradient * duty_div_100pc + linear_result_y_intercept;
            const linear_model_speed_error = speed - linear_model_speed;
            const power_law_model_speed = power_law_divisor * Math.pow(duty_div_100pc, power_law_result_gradient);
            const power_law_model_log10_speed = Math.log10(power_law_model_speed);
            const power_law_model_speed_error = speed - power_law_model_speed;
            const linear_model_duty_div_100pc = (speed * linear_result_gradient_reciprocal) + (linear_result_neg_intercept_over_gradient);
            const power_model_duty_div_100pc = Math.pow((speed / power_law_divisor), power_law_result_gradient_reciprocal); // all of these are zeros!

            regression_lines.push(
                {
                    Duty: duty,
                    "Speed [Hz]": speed,
                    "Speed STD": speed_std,
                    "Duty percentage / 100%": duty_div_100pc,
                    "LOG10(Duty percentage / 100%)": log10_duty_div_100pc,
                    "LOG10(Speed [Hz])": log10_speed,
                    "Linear model Speed [Hz]": linear_model_speed,
                    "Linear Model Speed Error [ΔHz]": linear_model_speed_error,
                    "Power law model LOG10(Speed [Hz])": power_law_model_log10_speed, // 
                    "Power law model Speed [Hz]": power_law_model_speed,
                    "Power law model Speed Error [ΔHz]": power_law_model_speed_error,
                    "Linear model Duty percentage / 100%": linear_model_duty_div_100pc,
                    "Power law model Duty percentage / 100%": power_model_duty_div_100pc
                }
            );
        });

        // now save config and plot data
        const regression_parser = new ASCIIParser(regression_config.inputs, ",");

        const regression_data_flat_lines = regression_lines.map((regression_line) => regression_parser.serialise(regression_line)).join("\n"); // regression_lines

        if (parsed_args.hasOwnProperty("output_data_file")) {
            fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, `.regression.${direction_str}.csv`), regression_data_flat_lines);
            fs.writeFileSync(`${parsed_args.output_data_file}`.replaceAll(/.json/g, `.regression.${direction_str}.config.json`), JSON.stringify(regression_config, null, 2));
        }

    });

    console2.success("All finished.");
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
    const end_time = Date.now();
    console2.info(`Finished in ${(end_time-start_time)/1000} [s]`);
    process.exit(0);
}).catch(async (err) => {
    await delay(300);
    await word_sender.send_word("thrustui16", 0);
    await delay(300);
    await word_sender.send_word("stop");
    console2.error(err);
    if (err.hasOwnProperty("stack")) { console2.error(err.stack) };
    process.exit(1);
});
