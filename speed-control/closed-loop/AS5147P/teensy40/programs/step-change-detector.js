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
    console.error(`StepChangeDetector: output_data_file ${full_input_config_path} exists already.`);
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

[{type: "steady"}, {type:"transition"}, {type: "steady"}];

const segments = [];

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
        console.log("here steady start");
        // create first segment
        segments.push({type:"steady", data:[line_data]});
        com_thrust = thrust;
    }
    else if (com_thrust != thrust) { // we have a signal indicating the start of the transition
        console.log("here transition start", thrust);
        // create transition segment
        segments.push({type: "transition", data: []});
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
            console.log("here steady continue");
            latest_segment.data.push(line_data);
        }
        else if (latest_segment.type === "transition") {
            console.log("here transition continue");
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
            if (e2v_std < e2v_std_min ) {
                console.log("New transition minumum discovered");
                // we have a new minimum
                e2v_std_min = e2v_std;
                min_index = index;
            }
            
            tmp_buffer.push(line_data);

            if (check_for_escape && (e2v_std > (first_e2v_value_for_transition/2))) {
                console.log("ESCAPE transition", e2v_std, first_e2v_value_for_transition/2);
                // escape 
                // take buffer from start to where min_index is.... this defines the transition segment
                // the remaining section of the buffer is steady state

                // buffer until min_index
                const transition_data = tmp_buffer.slice(0, min_index);

                // buffer after min_index
                const steady_state_data = tmp_buffer.slice(min_index); // to end

                latest_segment.data = transition_data;
                segments.push({type:"steady", data:steady_state_data});
            }

            if (e2v_std < (first_e2v_value_for_transition/2)) {
                check_for_escape = true;
            }
        }
    }
});

// there is the remaining transition / steady state data as the escape condition does not work for the last transition
/*const latest_segment_index = segments.length - 1;
const latest_segment = segments[latest_segment_index];

// buffer up to min_index
const transition_data = tmp_buffer.slice(0, min_index);

// buffer after min_index
const steady_state_data = tmp_buffer.slice(min_index); // to end
latest_segment.data = transition_data;
segments.push({type:"steady", data:[steady_state_data]});*/


console.log("segments", segments);
console.log("tmp_buffer", tmp_buffer);
console.log("highest_velocity", highest_velocity);

// ok so now rebuild data and when we are exactly at a transition then set a transition value of maximum velocity
let output_lines = [];

segments.forEach((segment) => {
    const type = segment.type;
    if (type === "steady") {
        output_lines = output_lines.concat(segment.data);
    }
    else {
        // console.log("transition non steady");
        // we have a transition period
        if (segment.data.length) {
            segment.data[0]["transition_p"] = highest_velocity;
            output_lines = output_lines.concat(segment.data);
        }
        // console.log("segment.data[0]", segment.data[0]);
    }
});

output_lines.forEach((line) => {
    if (!line.hasOwnProperty("transition_p")) {
        line["transition_p"] = 0;
    }
})

// console.log("output_lines", JSON.stringify(output_lines));

const out_config = [...file_input_config_json.inputs, {"name": "transition_p", "position": 39}];

console.log("out_config", out_config);

const serialiser = new ASCIIParser(out_config, ",");

const output_str = output_lines.map(line=>serialiser.serialise(line)).join("\n");

console.log("output_str", output_str);

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
