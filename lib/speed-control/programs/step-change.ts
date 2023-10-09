import { Subject } from "rxjs";
import { parse_args, CliArg, ArgumentHandlers, CliArgType } from "../../../external/kaepek-io/lib/host/ts-adaptors/cli-args.js";
import NetworkAdaptor from "../../../external/kaepek-io/lib/host/ts-adaptors/network.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import fs from "fs";

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
 * To process the data we need to use the double buffer technique to get the error of the error of the acceleration.
 * Then we look for detections of the minimum of the error ^2 of the acceleration.
 * 
 * 
 * We define 
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

// extend the incoming data format with the extra fields we will add via analysis.
const outgoing_data_config = [
    ...parsed_args.input_config_file,
    { "name": "smoothed_future_kalman_acceleration", "position": 17 },
    { "name": "smoothed_future_kalman_jerk", "position": 18 },
    { "name": "smoothed_prior_kalman_acceleration", "position": 19 },
    { "name": "smoothed_prior_kalman_jerk", "position": 20 },
    { "name": "smoothed_central_kalman_acceleration", "position": 21 },
    { "name": "smoothed_central_kalman_jerk", "position": 22 },
    { "name": "smoothed_prior_kalman_acceleration_std", "position": 23 },
    { "name": "smoothed_prior_kalman_jerk_std", "position": 24 },
    { "name": "smoothed_future_kalman_acceleration_std", "position": 25 },
    { "name": "smoothed_future_kalman_jerk_std", "position": 26 },
    { "name": "smoothed_central_kalman_acceleration_std", "position": 27 },
    { "name": "smoothed_central_kalman_jerk_std", "position": 28 },
    { "name": "smoothed2_future_kalman_acceleration", "position": 29 },
    { "name": "smoothed2_future_kalman_jerk", "position": 30 },
    { "name": "smoothed2_prior_kalman_acceleration", "position": 31 },
    { "name": "smoothed2_prior_kalman_jerk", "position": 32 },
    { "name": "smoothed2_central_kalman_acceleration", "position": 33 },
    { "name": "smoothed2_central_kalman_jerk", "position": 22 },
    { "name": "smoothed2_prior_kalman_acceleration_std", "position": 34 },
    { "name": "smoothed2_prior_kalman_jerk_std", "position": 24 },
    { "name": "smoothed2_future_kalman_acceleration_std", "position": 35 },
    { "name": "smoothed2_future_kalman_jerk_std", "position": 36 },
    { "name": "smoothed2_central_kalman_acceleration_std", "position": 37 },
    { "name": "smoothed2_central_kalman_jerk_std", "position": 38 }
];

adaptor.outgoing_data_config = outgoing_data_config;

const buffer1: Array<any> = [];
const buffer1_subject: Subject<any> = new Subject<any>;
const buffer1$ = buffer1_subject.asObservable();
const buffer2: Array<any> = [];
const buffer2_subject: Subject<any> = new Subject<any>;
const buffer2$ = buffer2_subject.asObservable();
const window_length = 10;


adaptor.incoming_data$.subscribe((line_data) => {

    // kalman_acceleration and kalman_jerk are the targets for smoothing.

    // deal with priors;
    const data_before = buffer1.slice(buffer1.length - window_length, buffer1.length);
    const acceleration_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["kalman_acceleration"], 0) / window_length;
    const jerk_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["kalman_jerk"], 0) / window_length;
    line_data["smoothed_prior_kalman_acceleration"] = acceleration_prior_avg;
    line_data["smoothed_prior_kalman_jerk"] = jerk_prior_avg;
    // stdev
    const acceleration_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_acceleration"] - acceleration_prior_avg, 2), 0) / window_length);
    const jerk_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_jerk"] - jerk_prior_avg, 2), 0) / window_length);
    line_data["smoothed_prior_kalman_acceleration_std"] = acceleration_prior_std;
    line_data["smoothed_prior_kalman_jerk_std"] = jerk_prior_std;
    buffer1.push(line_data);

    // deal with futures;

    if (buffer1.length >= window_length) {
        const index_to_update = buffer1.length - window_length;
        const data_after = buffer1.slice(index_to_update);
        const acceleration_future_avg = data_after.reduce((acc, data_item) => acc += data_item["kalman_acceleration"], 0) / window_length;
        const jerk_future_avg = data_after.reduce((acc, data_item) => acc += data_item["kalman_jerk"], 0) / window_length;
        buffer1[index_to_update]["smoothed_future_kalman_acceleration"] = acceleration_future_avg;
        buffer1[index_to_update]["smoothed_future_kalman_jerk"] = jerk_future_avg;
        // std
        const acceleration_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_acceleration"] - acceleration_future_avg, 2), 0) / window_length);
        const jerk_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_jerk"] - jerk_future_avg, 2), 0) / window_length);
        buffer1[index_to_update]["smoothed_future_kalman_acceleration_std"] = acceleration_future_std;
        buffer1[index_to_update]["smoothed_future_kalman_jerk_std"] = jerk_future_std;

        // create central smoothing
        const half_data_after = buffer1.slice(index_to_update, index_to_update + ((window_length - 1) / 2));
        const half_data_before = buffer1.slice(index_to_update - ((window_length - 1) / 2), index_to_update);
        const central_data = half_data_before.concat(half_data_after);
        const acceleration_central_avg = central_data.reduce((acc, data_item) => acc += data_item["kalman_acceleration"], 0) / window_length;
        const jerk_central_avg = central_data.reduce((acc, data_item) => acc += data_item["kalman_jerk"], 0) / window_length;
        buffer1[index_to_update]["smoothed_central_kalman_acceleration"] = acceleration_central_avg;
        buffer1[index_to_update]["smoothed_central_kalman_jerk"] = jerk_central_avg;
        // std
        const acceleration_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_acceleration"] - acceleration_central_avg, 2), 0) / window_length);
        const jerk_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_jerk"] - jerk_central_avg, 2), 0) / window_length);
        buffer1[index_to_update]["smoothed_central_kalman_acceleration_std"] = acceleration_central_std;
        buffer1[index_to_update]["smoothed_central_kalman_jerk_std"] = jerk_central_std;

        // this buffer index value is no ready for emissions.
        buffer1_subject.next(buffer1[index_to_update]);
    }

});

buffer1$.subscribe((line_data) => {
        // deal with priors;
        const half_window_length = parseInt(((window_length) / 2).toString());
        const data_before = buffer2.slice(buffer2.length - half_window_length, buffer2.length);
        const acceleration_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["smoothed_prior_kalman_acceleration"], 0) / half_window_length;
        const jerk_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["smoothed_prior_kalman_jerk"], 0) / half_window_length;
        line_data["smoothed2_prior_kalman_acceleration"] = acceleration_prior_avg;
        line_data["smoothed2_prior_kalman_jerk"] = jerk_prior_avg;
        // stdev
        const acceleration_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_prior_kalman_acceleration"] - acceleration_prior_avg, 2), 0) / half_window_length);
        const jerk_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_prior_kalman_jerk"] - jerk_prior_avg, 2), 0) / half_window_length);
        line_data["smoothed2_prior_kalman_acceleration_std"] = acceleration_prior_std;
        line_data["smoothed2_prior_kalman_jerk_std"] = jerk_prior_std;
        buffer2.push(line_data);

        // deal with futures;

        if (buffer2.length >= half_window_length) {
            const index_to_update = buffer2.length - half_window_length;
            const data_after = buffer2.slice(index_to_update);
            const acceleration_future_avg = data_after.reduce((acc, data_item) => acc += data_item["smoothed_future_kalman_acceleration"], 0) / half_window_length;
            const jerk_future_avg = data_after.reduce((acc, data_item) => acc += data_item["smoothed_future_kalman_jerk"], 0) / half_window_length;
            buffer2[index_to_update]["smoothed2_future_kalman_acceleration"] = acceleration_future_avg;
            buffer2[index_to_update]["smoothed2_future_kalman_jerk"] = jerk_future_avg;
            // std
            const acceleration_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_future_kalman_acceleration"] - acceleration_future_avg, 2), 0) / half_window_length);
            const jerk_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_future_kalman_jerk"] - jerk_future_avg, 2), 0) / half_window_length);
            buffer2[index_to_update]["smoothed2_future_kalman_acceleration_std"] = acceleration_future_std;
            buffer2[index_to_update]["smoothed2_future_kalman_jerk_std"] = jerk_future_std;

            // create central smoothing
            const half_data_after = buffer2.slice(index_to_update, index_to_update + ((half_window_length - 1) / 2));
            const half_data_before = buffer2.slice(index_to_update - ((half_window_length - 1) / 2), index_to_update);
            const central_data = half_data_before.concat(half_data_after);
            const acceleration_central_avg = central_data.reduce((acc, data_item) => acc += data_item["smoothed_central_kalman_acceleration"], 0) / half_window_length;
            const jerk_central_avg = central_data.reduce((acc, data_item) => acc += data_item["smoothed_central_kalman_jerk"], 0) / half_window_length;
            buffer2[index_to_update]["smoothed2_central_kalman_acceleration"] = acceleration_central_avg;
            buffer2[index_to_update]["smoothed2_central_kalman_jerk"] = jerk_central_avg;
            // std
            const acceleration_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_central_kalman_acceleration"] - acceleration_central_avg, 2), 0) / half_window_length);
            const jerk_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_central_kalman_jerk"] - jerk_central_avg, 2), 0) / half_window_length);
            buffer2[index_to_update]["smoothed2_central_kalman_acceleration_std"] = acceleration_central_std;
            buffer2[index_to_update]["smoothed2_central_kalman_jerk_std"] = jerk_central_std;

            // this buffer2 index value is now ready for emissions.
            buffer2_subject.next(buffer2[index_to_update]);
        }
});





























// adaptor.incoming_data$.

/*adaptor.incoming_data$.subscribe((line_data) => {
    adaptor.transmit_outgoing_data(line_data.parsed_data);
});*/













/**
 * class KalmanHigerDerivativesSmoother
 * Class to take incoming network data from a sink and to process it.
 * Smoothing consists of a set of window frames, with the current target as the start or end item within the frame.
 * The idea is to take average before and after the point in question allowing one to work out deviations from average behaviour.
 */
class KalmanHigerDerivativesSmoother extends NetworkAdaptor {
    buffer: Array<any> = [];
    buffer2: Array<any> = [];
    window_length = 10;
    output_file_path: string | null = null;

    set_window_length(window_length: number) {
        this.window_length = window_length;
    }

    set_output_file(path: string) {
        this.output_file_path = path;
    }

    async append_to_output_file(data_str: string) {
        return new Promise<void>((resolve, reject) => {
            if (this.output_file_path !== null) {
                fs.appendFile(this.output_file_path, data_str, (err) => {
                    if (err) return reject(err);
                    return resolve();
                });
            }
            return resolve();
        });
    }

    async ready() {
        this.outgoing_data_config = [
            ...this.incoming_data_config,
            { "name": "smoothed_future_kalman_acceleration", "position": 17 },
            { "name": "smoothed_future_kalman_jerk", "position": 18 },
            { "name": "smoothed_prior_kalman_acceleration", "position": 19 },
            { "name": "smoothed_prior_kalman_jerk", "position": 20 },
            { "name": "smoothed_central_kalman_acceleration", "position": 21 },
            { "name": "smoothed_central_kalman_jerk", "position": 22 },
            { "name": "smoothed_prior_kalman_acceleration_std", "position": 23 },
            { "name": "smoothed_prior_kalman_jerk_std", "position": 24 },
            { "name": "smoothed_future_kalman_acceleration_std", "position": 25 },
            { "name": "smoothed_future_kalman_jerk_std", "position": 26 },
            { "name": "smoothed_central_kalman_acceleration_std", "position": 27 },
            { "name": "smoothed_central_kalman_jerk_std", "position": 28 },
            { "name": "smoothed2_future_kalman_acceleration", "position": 29 },
            { "name": "smoothed2_future_kalman_jerk", "position": 30 },
            { "name": "smoothed2_prior_kalman_acceleration", "position": 31 },
            { "name": "smoothed2_prior_kalman_jerk", "position": 32 },
            { "name": "smoothed2_central_kalman_acceleration", "position": 33 },
            { "name": "smoothed2_central_kalman_jerk", "position": 22 },
            { "name": "smoothed2_prior_kalman_acceleration_std", "position": 34 },
            { "name": "smoothed2_prior_kalman_jerk_std", "position": 24 },
            { "name": "smoothed2_future_kalman_acceleration_std", "position": 35 },
            { "name": "smoothed2_future_kalman_jerk_std", "position": 36 },
            { "name": "smoothed2_central_kalman_acceleration_std", "position": 37 },
            { "name": "smoothed2_central_kalman_jerk_std", "position": 38 },
        ];
        await super.ready();
        // return "KalmanHigerDerivativesSmoother ready."
    }

    buffer2_callback(message_obj: any) {


        // deal with priors;
        const half_window_length = parseInt(((this.window_length) / 2).toString());
        const data_before = this.buffer2.slice(this.buffer2.length - half_window_length, this.buffer2.length);
        const acceleration_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["smoothed_prior_kalman_acceleration"], 0) / half_window_length;
        const jerk_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["smoothed_prior_kalman_jerk"], 0) / half_window_length;
        message_obj["smoothed2_prior_kalman_acceleration"] = acceleration_prior_avg;
        message_obj["smoothed2_prior_kalman_jerk"] = jerk_prior_avg;
        // stdev
        const acceleration_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_prior_kalman_acceleration"] - acceleration_prior_avg, 2), 0) / half_window_length);
        const jerk_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_prior_kalman_jerk"] - jerk_prior_avg, 2), 0) / half_window_length);
        message_obj["smoothed2_prior_kalman_acceleration_std"] = acceleration_prior_std;
        message_obj["smoothed2_prior_kalman_jerk_std"] = jerk_prior_std;
        this.buffer2.push(message_obj);

        // deal with futures;

        if (this.buffer2.length >= half_window_length) {
            const index_to_update = this.buffer2.length - half_window_length;
            const data_after = this.buffer2.slice(index_to_update);
            const acceleration_future_avg = data_after.reduce((acc, data_item) => acc += data_item["smoothed_future_kalman_acceleration"], 0) / half_window_length;
            const jerk_future_avg = data_after.reduce((acc, data_item) => acc += data_item["smoothed_future_kalman_jerk"], 0) / half_window_length;
            this.buffer2[index_to_update]["smoothed2_future_kalman_acceleration"] = acceleration_future_avg;
            this.buffer2[index_to_update]["smoothed2_future_kalman_jerk"] = jerk_future_avg;
            // std
            const acceleration_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_future_kalman_acceleration"] - acceleration_future_avg, 2), 0) / half_window_length);
            const jerk_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_future_kalman_jerk"] - jerk_future_avg, 2), 0) / half_window_length);
            this.buffer2[index_to_update]["smoothed2_future_kalman_acceleration_std"] = acceleration_future_std;
            this.buffer2[index_to_update]["smoothed2_future_kalman_jerk_std"] = jerk_future_std;

            // create central smoothing
            const half_data_after = this.buffer2.slice(index_to_update, index_to_update + ((half_window_length - 1) / 2));
            const half_data_before = this.buffer2.slice(index_to_update - ((half_window_length - 1) / 2), index_to_update);
            const central_data = half_data_before.concat(half_data_after);
            const acceleration_central_avg = central_data.reduce((acc, data_item) => acc += data_item["smoothed_central_kalman_acceleration"], 0) / half_window_length;
            const jerk_central_avg = central_data.reduce((acc, data_item) => acc += data_item["smoothed_central_kalman_jerk"], 0) / half_window_length;
            this.buffer2[index_to_update]["smoothed2_central_kalman_acceleration"] = acceleration_central_avg;
            this.buffer2[index_to_update]["smoothed2_central_kalman_jerk"] = jerk_central_avg;
            // std
            const acceleration_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_central_kalman_acceleration"] - acceleration_central_avg, 2), 0) / half_window_length);
            const jerk_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["smoothed_central_kalman_jerk"] - jerk_central_avg, 2), 0) / half_window_length);
            this.buffer2[index_to_update]["smoothed2_central_kalman_acceleration_std"] = acceleration_central_std;
            this.buffer2[index_to_update]["smoothed2_central_kalman_jerk_std"] = jerk_central_std;

            // this buffer2 index value is no ready for emissions.
            this.transmit_outgoing_data(this.buffer2[index_to_update]);
            // write to file
            if (this.output_file_path !== null) {
                this.append_to_output_file(`${this.outgoing_data_serialiser(this.buffer2[index_to_update])}\n`);
            }
        }
    }

    incoming_data_callback(message_obj: any, info: any) {

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
            this.buffer2_callback(this.buffer[index_to_update]);
        }
    }
}