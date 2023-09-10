import { ASCIIParser } from "../../../../../external/kaepek-io/lib/host/adaptors/ascii-parser.js";
import { parseArgs } from "node:util";
import fs from "fs";

const parse_options = {
    options: {
        input_data_file: {
            type: "string",
            short: "f"
        },
        input_config_file: {
            type: "string",
            short: "c"
        },
        output_data_file: {
            type: "string",
            short: "o"
        }
    }
};

let parsed_options = { values: {}, positionals: [] };

try {
    parsed_options = parseArgs(parse_options);
}
catch (e) {
    console.error(`KalmanHigerDerivativesSmoother: ${e.message}`);
    process.exit(1);
}

const missing_options = [];
Object.keys(parse_options.options).forEach((option_name) => {
    if (!parsed_options.values[option_name]) {
        missing_options.push(option_name);
    }
});

if (missing_options.length !== 0) {
    console.error(`KalmanHigerDerivativesSmoother: Missing the following arguments ${missing_options.map(option_str => {
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
    console.error(`StepChangeDetector: output_data_file ${full_output_data_path} exists already.`);
    process.exit(1);
}


const full_input_config_path = `${cwd}/${values["input_config_file"]}`;
const input_config_exists = fs.existsSync(full_input_config_path);
if (!input_config_exists) {
    console.error(`StepChangeDetector: input_config_file ${full_input_config_path} does not exist`);
    process.exit(1);
}

let file_input_config_json = null;
try {
    const file_input_config_str = fs.readFileSync(full_input_config_path);
    file_input_config_json = JSON.parse(file_input_config_str);
}
catch (e) {
    console.error(`StepChangeDetector: error parsing input_config_file ${full_input_config_path} ${e.stack}`);
    process.exit(1);
}

const full_input_data_path = `${cwd}/${values["input_data_file"]}`;
const input_data_file_exists = fs.existsSync(full_input_data_path);
if (!input_data_file_exists) {
    console.error(`StepChangeDetector: input_data_file_exists ${full_input_data_path} does not exist`);
    process.exit(1);
}

let file_input_data_str = null;
try {
    file_input_data_str = fs.readFileSync(full_input_data_path).toString();
}
catch (e) {
    console.error(`StepChangeDetector: error parsing input_data_file ${full_input_data_path} ${e.stack}`);
    process.exit(1);
}

// so at this point we have the ascii data and the input config file.

const parser = new ASCIIParser(file_input_config_json.inputs, ",");

const data_lines = file_input_data_str.split("\n");

let com_thrust = -1;

[{ type: "steady" }, { type: "transition" }, { type: "steady" }];

let segments = [];

let e2v_std_min = Number.POSITIVE_INFINITY;
let first_e2v_value_for_transition = Number.POSITIVE_INFINITY;
let tmp_buffer = [];
let index = 0;
let min_index = -1;
let check_for_escape = false;
let highest_velocity = -1;

data_lines.forEach((line) => {
    const line_data = parser.deserialise(line);
    const thrust = line_data["com_thrust_percentage"];
    const velocity = line_data["kalman_velocity"];
    if (velocity > highest_velocity) highest_velocity = velocity;
    if (com_thrust === -1) { // first stage (assume 0)
        // console.log("here steady start");
        // create first segment
        segments.push({ type: "steady", data: [line_data] });
        com_thrust = thrust;
    }
    else if (com_thrust != thrust) { // we have a signal indicating the start of the transition
        // console.log("here transition start", thrust);
        // create transition segment
        segments.push({ type: "transition", data: [] });
        e2v_std_min = line_data["smoothed2_future_kalman_acceleration_std"];
        first_e2v_value_for_transition = line_data["smoothed2_future_kalman_acceleration_std"];
        tmp_buffer = [line_data];
        index = 0;
        check_for_escape = false;
        com_thrust = thrust;
    }
    else { // check for the end of the transition
        const latest_segment_index = segments.length - 1;
        const latest_segment = segments[latest_segment_index];
        if (latest_segment.type === "steady") {
            // console.log("here steady continue");
            latest_segment.data.push(line_data);
        }
        else if (latest_segment.type === "transition") {
            // console.log("here transition continue");
            /*
            Here we need some careful logic...
            We need to travel down the first hump finding the first minimum value as we go.
            But need to be careful to not climb up the second hump.

            So the escape condition could be a value of e2v greater than say first_e2v_value_for_transition / 2
            at this point we accept values from up to the minimum found as the transition
            and we reject the rest adding to a new steady state segment

            on the last transition there is 2nd hump so 

            // need temporary buffer for this segment as it contains both the transition and the steady state.

            // worried about finiding late minimum arbitrarily late....
            new idea do this in two rounds use the technique above to find a "provisional" steady state region.
            Then find the mean and stdev of the steady state segment and then create some threshold so when you are within 
            say one sigma you are included. the expand the steady state region based on the condition.
            Could just do max and min for this region as bounds too.
            */

            // append this data to the output data so we can plot these transitions as a sanity check.

            // see if we have ended the transition

            index++;
            const e2v_std = line_data["smoothed2_future_kalman_acceleration_std"];
            if (e2v_std < e2v_std_min) {
                // console.log("New transition minumum discovered");
                // we have a new minimum
                e2v_std_min = e2v_std;
                min_index = index;
            }

            tmp_buffer.push(line_data);

            if (check_for_escape && (e2v_std > (first_e2v_value_for_transition / 2))) {
                // console.log("ESCAPE transition", e2v_std, first_e2v_value_for_transition / 2);
                // escape 
                // take buffer from start to where min_index is.... this defines the transition segment
                // the remaining section of the buffer is steady state

                // buffer until min_index
                const transition_data = tmp_buffer.slice(0, min_index);

                // buffer after min_index
                const steady_state_data = tmp_buffer.slice(min_index); // to end

                latest_segment.data = transition_data;
                segments.push({ type: "steady", data: steady_state_data });
            }

            if (e2v_std < (first_e2v_value_for_transition / 2)) {
                check_for_escape = true;
            }
        }
    }
});

console.log("remaining buffer", tmp_buffer);

segments = segments.filter((seg) => {
    return seg.data.length;
});
console.log("segments...",segments.map(seg => seg));


// there is the remaining transition / steady state data as the escape condition does not work for the last transition
/*const latest_segment_index = segments.length - 1;
const latest_segment = segments[latest_segment_index];

// buffer up to min_index
const transition_data = tmp_buffer.slice(0, min_index);

// buffer after min_index
const steady_state_data = tmp_buffer.slice(min_index); // to end
latest_segment.data = transition_data;
segments.push({type:"steady", data:[steady_state_data]});*/

// ok so now rebuild data and when we are exactly at a transition then set a transition value of maximum velocity
let output_lines = [];
let was_in_transition = false;

segments.forEach((segment) => {
    const type = segment.type;
    if (type === "steady") {
        if (was_in_transition == true) {
            was_in_transition = false;
            segment.data[0]["transition_p"] = highest_velocity;
        }
        output_lines = output_lines.concat(segment.data);
    }
    else {
        // console.log("transition non steady");
        // we have a transition period
        if (segment.data.length) {
            segment.data[0]["transition_p"] = highest_velocity;
            output_lines = output_lines.concat(segment.data);
            was_in_transition = true;
        }
        // console.log("segment.data[0]", segment.data[0]);
    }
});

output_lines.forEach((line) => {
    if (!line.hasOwnProperty("transition_p")) {
        line["transition_p"] = 0;
    }
});

/* 

segments... [
  'steady',     'transition',
  'steady',     'transition',
  'steady',     'transition',
  'steady',     'transition',
  'steady',     'transition',
  'transition'
]
*/

// ok now need to do this all over again but use 2nd strategy..... define min and max from stable region and extend 

// reverse output lines.
// assume steady state.
const reversed_lines = output_lines.reverse();
let min_velocity = Number.POSITIVE_INFINITY;
let max_velocity = 0;

const output2_segments = [{ type: "steady", data: [] }]; // {type:"steady", data:steady_state_data}

let passed_transition = false;

reversed_lines.forEach((line, idx) => {
    console.log("line time, transition_p", line["time"], line["transition_p"]);
    const latest_segment_index = output2_segments.length - 1;
    const latest_segment = output2_segments[latest_segment_index];
    const velocity = line["kalman_velocity"];

    if (latest_segment.type === "steady") {
        
        if (line["transition_p"] == 0 && passed_transition == false ) { // no transition yet
            console.log("continuing in stready region");
            // collect max and min from array up until non zero transition_p.
            if (velocity < min_velocity) {
                min_velocity = velocity;
            }
            if (velocity > max_velocity) {
                max_velocity = velocity;
            }
            latest_segment.data.push(line);
        }
        else {
            passed_transition = true;
            // we have reached the minimum e2v we now could check the contraints before continuing
            //             if (velocity >= (min_velocity * 0.995) && velocity <= (max_velocity * 1.005)) {
            if (velocity >= (min_velocity * 1) && velocity <= (max_velocity * 1)) {
                console.log("continuing in transition");
                latest_segment.data.push(line);
            }
            else { // violated a constraint we have exited the steady region
                console.log("exiting steady region vel,min,max,time", velocity, min_velocity, max_velocity, line["time"]);
                min_velocity = Number.POSITIVE_INFINITY;
                max_velocity = 0;
                line["transition_q"] = highest_velocity;
                passed_transition = false;
                output2_segments.push({ "type": "transition", data: [line] });
            }

        }
    }
    else { // within a transition
        // console.log("continuing with transition region");
        if (line["transition_p"] == 0) { // continuing the transition
            latest_segment.data.push(line);
        }
        else {
            console.log("exiting transition region vel,time", velocity, line["time"]);
            line["transition_q"] = highest_velocity;
            output2_segments.push({ "type": "steady", data: [line] }); // transition_p is non zero we have exited the transition
        }
    }

});

// finally need to create std and mean of the stable section, and find time delta of transitions

const stats = output2_segments.reduce((acc, segment) => {
    console.log("segment--------------", segment);

    if (segment.type === "steady") {
        // find mean and standard deviation / duty
        let min_velocity = Number.POSITIVE_INFINITY;
        let max_velocity = 0;
        const mean_velocity = segment.data.reduce((acc, segment_data) => {
            if (min_velocity > segment_data["kalman_velocity"]) {
                min_velocity = segment_data["kalman_velocity"];
            }
            if (max_velocity < segment_data["kalman_velocity"]) {
                max_velocity = segment_data["kalman_velocity"];
            }
            acc += segment_data["kalman_velocity"];
            return acc;
        }, 0) / segment.data.length;

        const std_velocity = Math.sqrt(segment.data.reduce((acc, segment_data) => {
            acc += Math.pow(segment_data["kalman_velocity"] - mean_velocity, 2);
            return acc;
        }, 0) / segment.data.length);

        const duty = segment.data[segment.data.length - 1]["com_thrust_percentage"];

        acc.push({type:"steady", duty, mean_velocity, std_velocity, min_velocity, max_velocity });
        
    }
    else { // within a transition
        const duty = segment.data[segment.data.length - 1]["com_thrust_percentage"];
        const min_time = segment.data[segment.data.length - 1]["time"];
        const max_time = segment.data[0]["time"];
        const transition_time = Math.abs(max_time - min_time);
        acc.push({type: "transition", duty, min_time, max_time, transition_time});
    }
    return acc;
}, []);

console.log("stats -------------------------");
console.log(stats);

// ok so now outputs csvs of the following

const steady_format = [
    {"name": "duty", "position": 0},
    {"name": "mean_velocity", "position": 1},
    {"name": "std_velocity", "position": 2}

];
// steady duty vs mean_velocity, std_velocity
const steady_serialiser = new ASCIIParser(steady_format, ",");
const steady_out = stats.filter((stat) => stat.type == "steady").map(line => steady_serialiser.serialise(line)).join("\n");
const full_steady_output_data_path = full_output_data_path + ".steady.csv"
fs.writeFileSync(full_steady_output_data_path, steady_out);

const transition_format = [
    {"name": "duty", "position": 0},
    {"name": "transition_time", "position": 1}
];
// transition duty vs transition_time
const transition_serialiser = new ASCIIParser(transition_format, ",");
const transition_out = stats.filter((stat) => stat.type == "transition").map(line => transition_serialiser.serialise(line)).join("\n");
const full_transition_output_data_path = full_output_data_path + ".transition.csv"
fs.writeFileSync(full_transition_output_data_path, transition_out);

// so for each segment. for the transition, look at the <next> section mean and std find when do we exit the stable region.

output2_segments.forEach((segment, idx) => {
    if (segment.type === "transition") {
        console.log("transition idx", idx);
        let escaped_region = false;
        const next_stable_idx = idx + 1;
        const next_segment_stats = stats[next_stable_idx];
        const next_min_vel = next_segment_stats.min_velocity;
        const next_max_vel = next_segment_stats.max_velocity;
        console.log("next min max", next_min_vel, next_max_vel);
        const data_reversed = segment.data.reverse();
        data_reversed.forEach((line) => {
            const velocity = line["kalman_velocity"];
            console.log("velocity", velocity);
            if (next_segment_stats.duty != 0) 
            {
                if ((velocity > next_max_vel || velocity < next_min_vel) && escaped_region == false) {
                    // we have left the previous stable regions constraints.
                    console.log("escape");
                    line["transition_s"] = highest_velocity;
                    escaped_region = true
                }
            }
            else {
                if ((velocity > (next_segment_stats.mean_velocity + next_segment_stats.std_velocity) || velocity < (next_segment_stats.mean_velocity - next_segment_stats.std_velocity)) && escaped_region == false) {
                    // we have left the previous stable regions constraints.
                    console.log("escape");
                    line["transition_s"] = highest_velocity;
                    escaped_region = true
                } 
            }

        });
        segment.data = data_reversed.reverse();
    }
});


console.log("output2_segments", output2_segments);
// reverse and reconstruct
const output2 = output2_segments.reduce((acc, segment) => {
    acc = acc.concat(segment.data);
    return acc;
}, []).reverse();

output2.forEach((line) => {
    if (!line.hasOwnProperty("transition_q")) {
        line["transition_q"] = 0;
    }
    if (!line.hasOwnProperty("transition_s")) {
        line["transition_s"] = 0;
    }
});

// console.log("output_lines", JSON.stringify(output_lines));

const out_config = [...file_input_config_json.inputs, { "name": "transition_p", "position": 39 }, { "name": "transition_q", "position": 40 }, { "name": "transition_s", "position": 41 }];
const serialiser = new ASCIIParser(out_config, ",");
const output_str = output2.map(line => serialiser.serialise(line)).join("\n");
fs.writeFileSync(full_output_data_path, output_str);


/*

Perhaps an easier detector would be red smoothed 2 prior. which seems to peak when the transition is finished...
trouble is there is quite a bit of up and down until red starts to rise property.... so perhaps create a cap so we dont start looking
for the peak until the red line has exceeded the blue lines first value.

// depends on window size too much... cannot tell if this is really a detector....

future error 2 minimum seems better but trouble is finding when to actually pick, falls to minimum region fairly quickly but then
osciallateds near zero (but positive) for a while

- need to escape if start rising if we exceed half the height of the blue line where its value started when com fired.


 */


// read input file

/*



start of transition dictated by the com signal rising. {"name":"com_thrust_percentage", "position": 3},
end of transition detector { "name": "smoothed2_future_kalman_acceleration_std", "position": 35 } reaching a minimum AFTER com thrust rise.


*/
