import NetworkAdaptor from "../../../../../external/kaepek-io/lib/host/adaptors/network.js";
import { parseArgs } from "node:util";

/**
 * class KalmanHigerDerivativesSmoother
 * Class to take incoming network data from a sink and to process it.
 * Smoothing consists of a set of window frames, with the current target as the start or end item within the frame.
 * The idea is to take average before and after the point in question allowing one to work out deviations from average behaviour.
 */
class KalmanHigerDerivativesSmoother extends NetworkAdaptor {
    buffer = [];
    window_length = 10;

    set_window_length(window_length) {
        this.window_length = window_length;
    }

    async ready() {
        this.outgoing_data_config = [
            ...this.incoming_data_config,
            { name: "smoothed_future_kalman_acceleration", position: 17 },
            { name: "smoothed_future_kalman_jerk", position: 18 },
            { name: "smoothed_prior_kalman_acceleration", position: 19 },
            { name: "smoothed_prior_kalman_jerk", position: 20 },
            { "name": "smoothed_central_kalman_acceleration", "position": 21 },
            { "name": "smoothed_central_kalman_jerk", "position": 22 },
            {"name": "smoothed_prior_kalman_acceleration_std", "position": 23},
            {"name": "smoothed_prior_kalman_jerk_std", "position": 24},
            {"name": "smoothed_future_kalman_acceleration_std", "position": 25},
            {"name": "smoothed_future_kalman_jerk_std", "position": 26},
            {"name": "smoothed_central_kalman_acceleration_std", "position": 27},
            {"name": "smoothed_central_kalman_jerk_std", "position": 28},
        ];
        await super.ready();
        return "KalmanHigerDerivativesSmoother ready."
    }
    incoming_data_callback(message_obj, info) {

        // kalman_acceleration and kalman_jerk are the targets for smoothing.

        // deal with priors;
        const data_before = this.buffer.slice(this.buffer.length - this.window_length, this.buffer.length);
        const acceleration_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["kalman_acceleration"], 0) / this.window_length;
        const jerk_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["kalman_jerk"], 0) / this.window_length;
        message_obj["smoothed_prior_kalman_acceleration"] = acceleration_prior_avg;
        message_obj["smoothed_prior_kalman_jerk"] = jerk_prior_avg;
        // stdev
        const acceleration_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_acceleration"] - acceleration_prior_avg, 2), 0) / this.window_length);
        const jerk_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_jerk"] - jerk_prior_avg, 2), 0) / this.window_length);
        message_obj["smoothed_prior_kalman_acceleration_std"] = acceleration_prior_std;
        message_obj["smoothed_prior_kalman_jerk_std"] = jerk_prior_std;
        this.buffer.push(message_obj);

        // deal with futures;

        if (this.buffer.length >= this.window_length) {
            const index_to_update = this.buffer.length - this.window_length;
            const data_after = this.buffer.slice(index_to_update);
            const acceleration_future_avg = data_after.reduce((acc, data_item) => acc += data_item["kalman_acceleration"], 0) / this.window_length;
            const jerk_future_avg = data_after.reduce((acc, data_item) => acc += data_item["kalman_jerk"], 0) / this.window_length;
            this.buffer[index_to_update]["smoothed_future_kalman_acceleration"] = acceleration_future_avg;
            this.buffer[index_to_update]["smoothed_future_kalman_jerk"] = jerk_future_avg;
            // std
            const acceleration_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_acceleration"] - acceleration_future_avg, 2), 0) / this.window_length);
            const jerk_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_jerk"] - jerk_future_avg, 2), 0) / this.window_length);
            this.buffer[index_to_update]["smoothed_future_kalman_acceleration_std"] = acceleration_future_std;
            this.buffer[index_to_update]["smoothed_future_kalman_jerk_std"] = jerk_future_std;

            // create central smoothing
            const half_data_after = this.buffer.slice(index_to_update, index_to_update + ((this.window_length - 1) / 2));
            const half_data_before = this.buffer.slice(index_to_update - ((this.window_length - 1) / 2), index_to_update);
            const central_data = half_data_before.concat(half_data_after);
            const acceleration_central_avg = central_data.reduce((acc, data_item) => acc += data_item["kalman_acceleration"], 0) / this.window_length;
            const jerk_central_avg = central_data.reduce((acc, data_item) => acc += data_item["kalman_jerk"], 0) / this.window_length;
            this.buffer[index_to_update]["smoothed_central_kalman_acceleration"] = acceleration_central_avg;
            this.buffer[index_to_update]["smoothed_central_kalman_jerk"] = jerk_central_avg;
            // std
            const acceleration_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_acceleration"] - acceleration_central_avg, 2), 0) / this.window_length);
            const jerk_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_jerk"] - jerk_central_avg, 2), 0) / this.window_length);
            this.buffer[index_to_update]["smoothed_central_kalman_acceleration_std"] = acceleration_central_std;
            this.buffer[index_to_update]["smoothed_central_kalman_jerk_std"] = jerk_central_std;

            // this buffer index value is no ready for emissions.
            this.transmit_outgoing_data(this.buffer[index_to_update]);
        }
    }
}


const parse_options = {
    options: {
        incoming_address: {
            type: "string",
            short: "a"
        },
        incoming_port: {
            type: "string",
            short: "p",
        },
        incoming_protocol: {
            type: "string",
            short: "n"
        },
        incoming_config: {
            type: "string",
            short: "c"
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
        window_length: {
            type: "string",
            short: "w"
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
    if (!parsed_options.values[option_name] && option_name !== "data") {
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

const smoother = new KalmanHigerDerivativesSmoother(values["incoming_address"], parseFloat(values["incoming_port"]), values["incoming_protocol"], values["outgoing_address"], parseFloat(values["outgoing_port"]), values["outgoing_protocol"], values["incoming_config"], ",");
smoother.set_window_length(parseFloat(values["window_length"]));
smoother.ready().then(console.log).catch(console.error);