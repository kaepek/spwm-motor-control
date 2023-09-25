
import { Task } from "../../../external/kaepek-io/lib/host/ts-adaptors/task.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";
import { ESCParsedLineData, RotationDetector } from "../../rotation-detector.js";
import { delay } from "../utils/delay.js";

export interface AngularBins {
    [angle_bin: string]: number[];
}

export interface AngularBinMeans {
    [bin_angle: string]: number;
}

export interface AngularBinsStd {
    [angle_bin: string]: number;
}

export interface TransformedAngularBins {
    [angle_bin: string]: number;
}

export interface ACState {
    angular_bins: AngularBins;
    mean: AngularBinMeans;
    stdev: AngularBinsStd;
    smallest_mean: number;
    largest_mean: number;
    largest_abs_mean: number;
    mean_of_means: number;
    stdev_of_means: number;
    transformed_angular_bins: TransformedAngularBins;
    norm_idle_duty: number; 
    norm_start_duty: number;
}

export interface ACMap {
    cw: ACState;
    ccw: ACState;
}

export class CollectAccelerationData extends Task<RotationDetector<ESCParsedLineData>> {
    max_duty: number;
    wait_time = 50;
    word_sender: SendWord;
    incoming_data: any;
    wait_timeout: any;
    start_duty: number | null = null;
    idle_duty: number | null = null;
    max_angular_bins: number;
    all_finished = false;
    percentage_minimum: number = -1;
    percentage_filled: number = -1;
    angular_bins: {[angle_bin: string]: Array<number>} = {};
    bin_population_threshold: number;
    max_angular_steps: number;
    direction = 0;
    direction_str = "cw";

    async run(state: any) {
        this.start_duty = state[this.direction_str].start_duty;
        this.idle_duty = state[this.direction_str].idle_duty;
        // parseInt(((65534 / this.max_duty) * (this.current_duty as number)).toString())
        // 20425 / 65534
        console2.info(`CollectAccelerationData program running`);
        console2.info("Sending word thrustui16", 0);
        await this.word_sender.send_word("thrustui16", 0);
        await this.word_sender.send_word("directionui8", this.direction);
        await delay(100);
        console2.info("Sending word thrustui16", this.start_duty);
        await this.word_sender.send_word("thrustui16", this.start_duty as number);
        await delay(100);
        await this.word_sender.send_word("reset");
        await this.word_sender.send_word("start");
        await delay(1000);
        console2.info("Sending word thrustui16", this.idle_duty);
        await this.word_sender.send_word("thrustui16", this.idle_duty as number);
        await delay(1000);
        return super.run(); // tick will now run every time the device outputs a line.
    }

    async tick(incoming_data: RotationDetector<ESCParsedLineData>) {
        const acc = incoming_data.line_data.parsed_data.kalman_acceleration;
        const angle_raw = incoming_data.line_data.parsed_data.encoder_raw_displacement;
        // angle could be 16383... 16383 -> max_angular_bins - 1 (aka 2047)
        // 16383 * (2047/16383)
        const compressed_angle = Math.round(angle_raw * ((this.max_angular_bins - 1) / (this.max_angular_steps - 1)));
        this.angular_bins[compressed_angle].push(acc);
        const [progress, completed] = this.bin_population_count();
        const percentage_minimum = progress / this.bin_population_threshold;
        const percentage_filled = completed / this.max_angular_bins;

        // console.log("filled", completed, this.max_angular_bins, percentage_filled);
        if (this.percentage_minimum != percentage_minimum) {
            console2.info(`Collected bin population minimum completion ${percentage_minimum*100}%`);
            this.percentage_minimum = percentage_minimum;
        }

        if (this.percentage_filled != percentage_filled) {
            console2.info(`Collected bin population completely filled ${percentage_filled*100}%`);
            this.percentage_filled = percentage_filled;
        }

        if (progress === this.bin_population_threshold) {
            return this.return_promise_resolver();
        }
    }

    async done() {
        // this will be called after return_promise_resolver is called.
        // turn off motor
        await this.word_sender.send_word("thrustui16", 0);
        await this.word_sender.send_word("stop");

        let smallest_mean = Infinity;
        let largest_mean = -Infinity;

        const thrust_to_duty_factor = (this.max_duty / 65533);

        // compute stats for angular_bins
        const mean = Object.keys(this.angular_bins).reduce((acc: any, bin_angle) => {
            const values = this.angular_bins[bin_angle];
            const _mean = values.reduce((acc,value) => {
                acc += value;
                return acc;
            }, 0) / values.length;
            acc[bin_angle] = _mean;
            if (_mean > largest_mean) {
                largest_mean = _mean;
            }
            if (_mean < smallest_mean) {
                smallest_mean = _mean;
            }
            return acc;
        }, {}) as {[bin_angle: string]: number};

        const stdev = Object.keys(this.angular_bins).reduce((acc: {[angle_bin: string]: number}, bin_angle) => {
            const values = this.angular_bins[bin_angle];
            const _mean = mean[bin_angle];
            const values_neg_mean_square = values.map((value) => Math.pow(value-_mean,2));
            const sum_values_neg_mean_square = values_neg_mean_square.reduce((acc, it) => {
                acc += it;
                return acc;
            },0);
            const std = Math.sqrt(sum_values_neg_mean_square / values.length);
            acc[bin_angle] = std;
            return acc;
        }, {});

        // calculate angular bins norm

        // calculate mean of all means

        const mean_of_means = Object.keys(mean).reduce((acc: number, angle) => {
            acc += mean[angle];
            return acc;
        }, 0) / this.max_angular_bins;

        const stdev_of_means = Math.sqrt(Object.keys(mean).reduce((acc: number, angle) => {
            acc += Math.pow(mean[angle] - mean_of_means, 2);
            return acc;
        },0) / this.max_angular_bins);

        const abs_small = Math.abs(smallest_mean);
        const abs_max = Math.abs(largest_mean);

        const largest_abs_mean = Math.max(...[abs_small, abs_max]);

        const norm_idle_duty = Math.round(thrust_to_duty_factor*(this.idle_duty as any) as number);
        const norm_start_duty = Math.round(thrust_to_duty_factor*(this.start_duty as any) as number);

        // normalise the angular bins with the algorithm...

        const transformed_angular_bins = Object.keys(mean).reduce((acc: {[angle_bin: string]: number}, angle) => {
            const bin_value = mean[angle];
            const modified_bin_value = bin_value / largest_abs_mean;
            acc[angle] = Math.round(modified_bin_value * -1 * (norm_idle_duty as any) as number);
            return acc;
        }, {})

        console2.info(`CollectAccelerationData program finished`);
        return { [this.direction_str]: {angular_bins: this.angular_bins, mean, stdev, smallest_mean, largest_mean, largest_abs_mean, mean_of_means, stdev_of_means, transformed_angular_bins, norm_idle_duty, norm_start_duty}};
    }

    bin_population_count() {
        let min_population: number | null = null;
        let completed = 0;
        Object.keys(this.angular_bins).forEach((angle_bin) => {
            const counts = this.angular_bins[angle_bin].length;
            if (min_population === null) {
                min_population = counts;
            }
            else if (counts < min_population) {
                min_population = counts;
            }

            if (counts >= this.bin_population_threshold) {
                completed++;
            }
        });
        return [min_population as any as number, completed];
    }
    
    constructor(input$: Observable<any>, word_sender: SendWord, direction_str = "cw", max_duty = 2047, max_angular_steps = 16384, angular_compression_ratio = 8, bin_population_threshold = 10) {
        super(input$);
        this.max_duty = max_duty;
        this.word_sender = word_sender;
        this.max_angular_bins = Math.round(max_angular_steps / angular_compression_ratio);
        for (let i = 0; i < this.max_angular_bins; i++) {
            this.angular_bins[i] = [];
        }
        this.bin_population_threshold = bin_population_threshold;
        this.max_angular_steps = max_angular_steps;
        this.direction_str = direction_str;
        if (direction_str === "cw") {
            this.direction = 0;
        }
        else if (direction_str === "ccw") {
            this.direction = 1;
        }
    }
}
