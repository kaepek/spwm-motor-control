
import { Task } from "../../../external/kaepek-io/lib/host/controller/utils/task.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/controller/utils/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";
import { ESCParsedLineData } from "../../rotation-detector.js";
import { delay } from "../utils/delay.js";

/**
 * Brief
 * Go through a range of duties. Starting with zero and then the determined start duty and iterate N - 2 steps until max duty.
 * For each sent word we need to take the acceleration data and work out if we have entered a stable region, this is defined to be
 * when we have not seen a lower acceleration value for a set period of time determined by the wait_time (in milliseconds).
 * 
 * Brief 2
 * Go through a range of dutites. Start by finding the start speed and the idle speed in both directions. Then go from the start speed and
 * set the idle speed. Once we are idleing succesfully, go from the idle speed to the maximum set duty in N - 1 steps.
 */

export type LineData = {
    kalman_hz: number,
    sample_hz: number,
    time: number,
    com_thrust_percentage: number,
    com_direction: number,
    eular_displacement: number,
    eular_velocity: number,
    eular_acceleration: number,
    eular_jerk: number,
    kalman_displacement: number,
    kalman_velocity: number,
    kalman_acceleration: number,
    kalman_jerk: number,
    voltage_phase_a: number,
    voltage_phase_b: number,
    voltage_phase_c: number,
    encoder_raw_displacement: number
};

type SteadySegment = {
    type: "steady",
    data: Array<LineData>,
    duty: number 
}

type TransitionSegment = {
    type: "transition",
    data: Array<LineData>,
    duty: number 
}

type TransitionSegmentWithDeadTime = TransitionSegment & {dead_time: number};

type Segment = SteadySegment | TransitionSegment;

type SteadySegmentWithMinAndMax = SteadySegment & {
    min_velocity: number;
    max_velocity: number;
};

export type SteadySegmentWithStats = (SteadySegmentWithMinAndMax & {
    mean_velocity: number;
    std_velocity: number;
});

export type SegmentWithStats = TransitionSegmentWithDeadTime | SteadySegmentWithStats;

export class GetStepChange extends Task<ESCParsedLineData> {
    max_duty: number;
    wait_time: number;
    current_duty: number | null = null;
    word_sender: SendWord;
    wait_timeout: any;
    start_duty: number | null = null;
    n_duty_steps: number;

    async create_timeout() {
        if (this.wait_timeout) clearTimeout(this.wait_timeout);
        this.wait_timeout = setTimeout(async () => {
            // we are now in a steady region
            this.segments.push({ type: "steady", data: [], duty: this.current_duty as number });
            await delay(this.wait_time);
            this.send_next_word();
        }, this.wait_time);
    }

    send_next_word() {
        if (this.duties_to_apply.length) {
            const duty_to_apply = this.duties_to_apply.shift();
            const mapped_duty = parseInt(((65534 / this.max_duty) * (duty_to_apply as number)).toString());
            this.current_duty = duty_to_apply as number;
            console2.info("Sending word thrustui16", mapped_duty);
            this.segments.push({ type: "transition", data: [], duty: duty_to_apply as number });
            this.word_sender.send_word("thrustui16", mapped_duty);
            this.create_timeout();
            this.smallest_acc = Number.POSITIVE_INFINITY;
            this.largest_vel = Number.NEGATIVE_INFINITY;
        }
        else {
            clearTimeout(this.wait_timeout);
            this.return_promise_resolver();
        }
    }

    duties_to_apply: Array<number> = [];
    async run(state: any) {
        const idle_duty = state[this.direction_str].idle_duty;
        const start_duty = state[this.direction_str].start_duty;

        let begin_duty = 0;

        if (this.start_duty_override === "idle") {
            begin_duty = parseInt((idle_duty as number * (this.max_duty / 65534)).toString());
        }
        else if (this.start_duty_override === "start"){
            begin_duty = parseInt((start_duty as number * (this.max_duty / 65534)).toString());
        }
        else if (this.start_duty_override !== undefined) {
            begin_duty = parseInt((this.start_duty_override as number * (this.max_duty / 65534)).toString());
        }

        const steps_remaining = this.n_duty_steps - 1;
        const iter = (this.end_duty - begin_duty) / steps_remaining;
        const remaining_range: Array<number> = [];
        let c_value = begin_duty;
        for (let i = 1; i <= steps_remaining; i++) { // 1,2,3,4,5,6,7,8, 9
            c_value += iter;
            remaining_range.push(c_value);
        }
        const range = [begin_duty, ...remaining_range];
        const rounded_range = range.map(Math.round);
        this.duties_to_apply = rounded_range;

        this.current_duty = begin_duty;
        console2.info(`GetStepChange program is initalising, duty range is ${JSON.stringify(this.duties_to_apply)}`);
        await delay(100);
        console2.info("GetStepChange program is now running");
        this.send_next_word();
        return super.run(); // tick will now run every time the device outputs a line.
    }

    smallest_acc: number = Number.POSITIVE_INFINITY;
    largest_vel: number = Number.NEGATIVE_INFINITY;
    segments: Array<Segment> = [];

    async tick(incoming_data: ESCParsedLineData) {

        if (this.segments.length) {
            // add data to correct segment
            const latest_segment_idx = this.segments.length - 1;
            this.segments[latest_segment_idx].data.push(incoming_data.parsed_data);
            const acc = incoming_data.parsed_data.kalman_acceleration * this.direction_sign;
            const vel = incoming_data.parsed_data.kalman_velocity * this.direction_sign;
            if (acc < this.smallest_acc) {
                this.smallest_acc = acc;
                this.create_timeout(); // reset the next/exit procedure timeout.
            }
            if (vel > this.largest_vel) {
                this.largest_vel = vel;
                this.create_timeout(); // reset the next/exit procedure timeout.
            }
        }

    }

    async done() {
        // this will be called after return_promise_resolver is called.
        // turn off motor
        console2.info(`StepChange program, turning motor off.`);
        await delay(300);
        await this.word_sender.send_word("thrustui16", 0);
        await delay(300);
        // await this.word_sender.send_word("stop");
        // await delay(1000); // todo parameterise this as a cooldown time. also this should only need to be applied if this is the 1st direction

        // now process the segments
        this.segments.forEach((segment, idx) => {
            // filter segments to make sure we exclude data which has not had the correct duty applied in data yet. Can happen due to latency.
            this.segments[idx].data.filter((line) => {
                const line_duty = Math.round(line.com_thrust_percentage * this.max_duty)
                return line_duty === segment.duty;
            })
        });

        // remove the first transitional region because we know that it is in fact stable when thrust is zero.
        this.segments.shift();

        /** 
         * now this is a little too permissive in the transition region because we have to wait some time before we confirm the region is stable.
         * So we need to extend the stable region by some amount and steal data from the transitional region so long as it falls within the bounds
         * set by the stable region.
        */

        const reversed_segments = this.segments.reverse() as SteadySegmentWithMinAndMax[];

        let segment_velocity_min = Number.POSITIVE_INFINITY;
        let segment_velocity_max = Number.NEGATIVE_INFINITY;

        reversed_segments.forEach((segment, reversed_segment_idx) => {
            if (segment.type === "steady") {
                // annotate first line
                (segment.data[0] as any).steady_region1 = 1.0;
                // calculate min and max of this region
                segment_velocity_min = Number.POSITIVE_INFINITY;
                segment_velocity_max = Number.NEGATIVE_INFINITY;
                segment.data.forEach((line) => {
                    const kalman_velocity_pos = line.kalman_velocity * this.direction_sign;
                    if (kalman_velocity_pos > segment_velocity_max) {
                        segment_velocity_max = kalman_velocity_pos;
                    }
                    if (kalman_velocity_pos < segment_velocity_min) {
                        segment_velocity_min = kalman_velocity_pos;
                    }
                });
                // should add min and max vel to these segments
                reversed_segments[reversed_segment_idx].min_velocity = segment_velocity_min * this.direction_sign;
                reversed_segments[reversed_segment_idx].max_velocity = segment_velocity_max * this.direction_sign;
            }
            else { // transition

                
                // annotate first line
                (segment.data[0] as any).transition_region1 = 1.0;

                const reversed_segment_data = [...segment.data].reverse();
                const transition_segment_max_index = segment.data.length - 1;

                // now go from the end of the transition segment to the start, if the transition line velocity data falls within the bounds
                // set but the future stable region, then it should be included in this future stable region and not the transition.
                let reversed_transition_idx_to_include_in_stable_segment = -1;

                for (let idx = 0; idx < reversed_segment_data.length; idx++) {
                    const line_data = reversed_segment_data[idx];
                    const kalman_velocity_pos = line_data.kalman_velocity * this.direction_sign;
                    if (kalman_velocity_pos <= (segment_velocity_max * this.max_stability_tolerance) && kalman_velocity_pos >= (segment_velocity_min * this.min_stability_tolerance) ) {
                        reversed_transition_idx_to_include_in_stable_segment = idx;
                    }
                    else {
                        break;
                    }
                }

                if (reversed_transition_idx_to_include_in_stable_segment === -1) {
                    // nothing changes leave the segments alone
                    return;
                }
                else {
                    // get the index is the non reversed direction
                    const transition_idx_to_slice =  transition_segment_max_index - reversed_transition_idx_to_include_in_stable_segment;
                    const transition_segment_data = segment.data.slice(0, transition_idx_to_slice);
                    const extra_stable_segment_data = segment.data.slice(transition_idx_to_slice);
                    reversed_segments[reversed_segment_idx].data = transition_segment_data;
                    reversed_segments[reversed_segment_idx - 1].data = [...extra_stable_segment_data, ...reversed_segments[reversed_segment_idx - 1].data];
                }

            }
        });

        const segments = reversed_segments.reverse();

        // compute stats for the stable regions
        const segments_with_stats = segments.map((segment: Segment) => {
            const segment_with_stats = segment as SegmentWithStats;
            if (segment.type === "steady") {
                const mean_velocity = segment.data.reduce((acc, line_data) => {
                    acc += line_data.kalman_velocity;
                    return acc;
                }, 0) / segment.data.length;

                const std_velocity = Math.sqrt(segment.data.reduce((acc, line_data) => {
                    acc += Math.pow(line_data.kalman_velocity - mean_velocity, 2);
                    return acc;
                }, 0) / segment.data.length);
                (segment_with_stats as SteadySegmentWithStats).mean_velocity = mean_velocity;
                (segment_with_stats as SteadySegmentWithStats).std_velocity = std_velocity;
            }
            return segment_with_stats;
        });

        segments_with_stats.forEach((segment, idx) => {
            if (segment.type === "steady") {
                segments_with_stats[idx].data.forEach((data_point, point_idx) => {
                    if (point_idx === 0.0) {
                        (segments_with_stats[idx].data[point_idx] as any).steady_region2 = 1.0;
                    }
                    else {
                        (segments_with_stats[idx].data[point_idx] as any).steady_region2 = 0.0;
                    }
                });
            }
            else {

                segments_with_stats[idx].data.forEach((data_point, point_idx) => {
                    if (point_idx === 0.0) {
                        (segments_with_stats[idx].data[point_idx] as any).transition_region2 = 1.0;
                    }
                    else {
                        (segments_with_stats[idx].data[point_idx] as any).transition_region2 = 0.0;
                    }
                });
            }
        });

        // last step compute dead times
        segments_with_stats.forEach((segment, segment_idx) => {
            if (segment.type === "steady") {
                segment_velocity_min = segment.min_velocity * this.direction_sign;
                segment_velocity_max = segment.max_velocity * this.direction_sign;
            }
            else {
                // iterate through transition when we exceed a threshold then we can assume the deadtime is over
                let dead_time_index = 0;
                for (let idx = 0; idx < segment.data.length - 1; idx++){
                    const line = segment.data[idx];
                    const kalman_velocity_pos = line.kalman_velocity * this.direction_sign;
                    if (kalman_velocity_pos > (segment_velocity_max * this.max_stability_tolerance) || kalman_velocity_pos < (segment_velocity_min * this.min_stability_tolerance) ) {
                        dead_time_index = idx;
                        break;
                    }
                }
                (segments_with_stats[segment_idx].data[dead_time_index] as any).dead_region = 1.0;
                const start_time = segment.data[0].time;
                const end_time = segment.data[dead_time_index].time;
                const dead_time = end_time - start_time;
                (segments_with_stats[segment_idx] as TransitionSegmentWithDeadTime).dead_time = dead_time;
            }
        });

        console2.info(`StepChange program finished`);

        return { [this.direction_str]: { "segments": segments_with_stats} }
    }

    direction = 0;
    direction_str = "cw";
    direction_sign = 1.0;
    max_stability_tolerance = 1.0;
    min_stability_tolerance = 1.0;
    end_duty = 2047;
    start_duty_override: number | string | undefined;

    constructor(input$: Observable<any>, word_sender: SendWord, direction_str = "cw", max_duty = 2047, n_duty_steps = 10, wait_time = 3000, stable_region_tolerance_percentage = 1, end_duty = 464, start_duty_override: number | string | undefined = "idle") {
        super(input$);
        this.n_duty_steps = n_duty_steps;
        this.max_duty = max_duty;
        this.word_sender = word_sender;
        this.direction_str = direction_str;
        this.end_duty = end_duty;
        this.wait_time = wait_time;
        this.max_stability_tolerance = 1.0 + (stable_region_tolerance_percentage / 100);
        this.min_stability_tolerance = 1.0 - (stable_region_tolerance_percentage / 100);
        this.start_duty_override = start_duty_override;
        if (direction_str === "cw") {
            this.direction = 0;
        }
        else if (direction_str === "ccw") {
            this.direction = 1;
            this.direction_sign = -1.0;
        }
        else {
            throw `Unrecognised direction ${direction_str} should be 'cw' or 'ccw'`;
        }
    }
}
