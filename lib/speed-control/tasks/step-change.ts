
import { Task } from "../../../external/kaepek-io/lib/host/ts-adaptors/task.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable, Subject } from "rxjs";
import { ESCParsedLineData } from "../../rotation-detector.js";

export type Buffer1Element = {
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
    encoder_raw_displacement: number,
    smoothed_prior_kalman_acceleration: number;
    smoothed_prior_kalman_jerk: number;
    smoothed_prior_kalman_acceleration_std: number;
    smoothed_prior_kalman_jerk_std: number;
    smoothed_future_kalman_acceleration: number;
    smoothed_future_kalman_jerk: number;
    smoothed_future_kalman_acceleration_std: number;
    smoothed_future_kalman_jerk_std: number;
    smoothed_central_kalman_acceleration: number;
    smoothed_central_kalman_jerk: number;
    smoothed_central_kalman_acceleration_std: number;
    smoothed_central_kalman_jerk_std: number;
}

type Buffer2Element = {
    smoothed2_prior_kalman_acceleration: number;
    smoothed2_prior_kalman_jerk: number;
    smoothed2_prior_kalman_acceleration_std: number;
    smoothed2_prior_kalman_jerk_std: number;
    smoothed2_future_kalman_acceleration: number;
    smoothed2_future_kalman_jerk: number;
    smoothed2_future_kalman_acceleration_std: number;
    smoothed2_future_kalman_jerk_std: number;
    smoothed2_central_kalman_acceleration: number;
    smoothed2_central_kalman_jerk: number;
    smoothed2_central_kalman_acceleration_std: number;
    smoothed2_central_kalman_jerk_std: number;
 } & Buffer1Element;


export class StepChange extends Task<ESCParsedLineData> {
    max_duty: number;
    initial_duty = 0;
    wait_time = 6;
    current_duty: number | null = null;
    word_sender: SendWord;
    incoming_data: any;
    wait_timeout: any;
    start_duty: number | null = null;

    buffer1: Array<Buffer1Element> = [];
    buffer1_subject: Subject<Buffer1Element> = new Subject<Buffer1Element>;
    buffer1$ = this.buffer1_subject.asObservable();
    buffer2: Array<Buffer2Element> = [];
    buffer2_subject: Subject<Buffer2Element> = new Subject<Buffer2Element>;
    public buffer2$ = this.buffer2_subject.asObservable();
    window_length = 10;


    async send_next_word() {
        (this.current_duty as number)++;
        let finished = false;
        if (this.current_duty as number > this.max_duty) {
            this.current_duty = this.max_duty;
            finished = true;
        }
        const mapped_duty = parseInt(((65534 / this.max_duty) * (this.current_duty as number)).toString());
        console2.info("Sending word thrustui16", mapped_duty);
        this.word_sender.send_word("thrustui16", mapped_duty);
        this.start_duty = mapped_duty;
        return finished;
    }


    async run(state: any) {
        this.current_duty = 0;
        console2.info(`GetStartDuty program running`);
        await this.word_sender.send_word("thrustui16", 0);
        await this.word_sender.send_word("directionui8", this.direction);
        await this.word_sender.send_word("thrustui16", this.current_duty as number);
        await this.word_sender.send_word("reset");
        await this.word_sender.send_word("start");
        this.buffer1$.subscribe((data: Buffer1Element) => this.buffer1_tick(data));
        this.buffer2$.subscribe((data) => this.buffer2_tick(data));
        return super.run(); // tick will now run every time the device outputs a line.
    }

    async tick(incoming_data: ESCParsedLineData) {
        // kalman_acceleration and kalman_jerk are the targets for smoothing.
        const message_obj = incoming_data.parsed_data as Buffer1Element;

        // deal with priors;
        const data_before = this.buffer1.slice(this.buffer1.length - this.window_length, this.buffer1.length);
        const acceleration_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["kalman_acceleration"], 0) / this.window_length;
        const jerk_prior_avg = data_before.reduce((acc, data_item) => acc += data_item["kalman_jerk"], 0) / this.window_length;
        message_obj["smoothed_prior_kalman_acceleration"] = acceleration_prior_avg;
        message_obj["smoothed_prior_kalman_jerk"] = jerk_prior_avg;
        // stdev
        const acceleration_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_acceleration"] - acceleration_prior_avg, 2), 0) / this.window_length);
        const jerk_prior_std = Math.sqrt(data_before.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_jerk"] - jerk_prior_avg, 2), 0) / this.window_length);
        message_obj["smoothed_prior_kalman_acceleration_std"] = acceleration_prior_std;
        message_obj["smoothed_prior_kalman_jerk_std"] = jerk_prior_std;
        this.buffer1.push(message_obj);

        // deal with futures;

        if (this.buffer1.length >= this.window_length) {
            const index_to_update = this.buffer1.length - this.window_length;
            const data_after = this.buffer1.slice(index_to_update);
            const acceleration_future_avg = data_after.reduce((acc, data_item) => acc += data_item["kalman_acceleration"], 0) / this.window_length;
            const jerk_future_avg = data_after.reduce((acc, data_item) => acc += data_item["kalman_jerk"], 0) / this.window_length;
            this.buffer1[index_to_update]["smoothed_future_kalman_acceleration"] = acceleration_future_avg;
            this.buffer1[index_to_update]["smoothed_future_kalman_jerk"] = jerk_future_avg;
            // std
            const acceleration_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_acceleration"] - acceleration_future_avg, 2), 0) / this.window_length);
            const jerk_future_std = Math.sqrt(data_after.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_jerk"] - jerk_future_avg, 2), 0) / this.window_length);
            this.buffer1[index_to_update]["smoothed_future_kalman_acceleration_std"] = acceleration_future_std;
            this.buffer1[index_to_update]["smoothed_future_kalman_jerk_std"] = jerk_future_std;

            // create central smoothing
            const half_data_after = this.buffer1.slice(index_to_update, index_to_update + ((this.window_length - 1) / 2));
            const half_data_before = this.buffer1.slice(index_to_update - ((this.window_length - 1) / 2), index_to_update);
            const central_data = half_data_before.concat(half_data_after);
            const acceleration_central_avg = central_data.reduce((acc, data_item) => acc += data_item["kalman_acceleration"], 0) / this.window_length;
            const jerk_central_avg = central_data.reduce((acc, data_item) => acc += data_item["kalman_jerk"], 0) / this.window_length;
            this.buffer1[index_to_update]["smoothed_central_kalman_acceleration"] = acceleration_central_avg;
            this.buffer1[index_to_update]["smoothed_central_kalman_jerk"] = jerk_central_avg;
            // std
            const acceleration_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_acceleration"] - acceleration_central_avg, 2), 0) / this.window_length);
            const jerk_central_std = Math.sqrt(central_data.reduce((acc, data_item) => acc += Math.pow(data_item["kalman_jerk"] - jerk_central_avg, 2), 0) / this.window_length);
            this.buffer1[index_to_update]["smoothed_central_kalman_acceleration_std"] = acceleration_central_std;
            this.buffer1[index_to_update]["smoothed_central_kalman_jerk_std"] = jerk_central_std;

            // this buffer index value is no ready for emissions.
            this.buffer1_subject.next(this.buffer1[index_to_update]);
        }
    }

    buffer1_tick(incoming_data: Buffer1Element) {
        const message_obj = incoming_data as Buffer2Element;

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
            this.buffer2_subject.next(this.buffer2[index_to_update]);
        }
    }

    com_thrust = -1;
    segments:Array<{type: string, data: Array<Buffer2Element>}> = []; // e.g. [{ type: "steady" }, { type: "transition" }, { type: "steady" }];
    e2v_std_min = Number.POSITIVE_INFINITY;
    first_e2v_value_for_transition = Number.POSITIVE_INFINITY;
    tmp_buffer: Array<Buffer2Element> = [];
    index = 0;
    min_index = -1;
    check_for_escape = false;
    highest_velocity = -1;

    buffer2_tick(incoming_data: Buffer2Element) {
        const thrust = incoming_data["com_thrust_percentage"];
        const velocity = incoming_data["kalman_velocity"];
        if (velocity > this.highest_velocity) this.highest_velocity = velocity;
        if (this.com_thrust === -1) { // first stage (assume 0)
            // create first segment
            this.segments.push({ type: "steady", data: [incoming_data] });
            this.com_thrust = thrust;
        }
        else if (this.com_thrust != thrust) { // we have a signal indicating the start of the transition
            // create transition segment
            this.segments.push({ type: "transition", data: [] });
            this.e2v_std_min = incoming_data["smoothed2_future_kalman_acceleration_std"];
            this.first_e2v_value_for_transition = incoming_data["smoothed2_future_kalman_acceleration_std"];
            this.tmp_buffer = [incoming_data];
            this.index = 0;
            this.check_for_escape = false;
            this.com_thrust = thrust;
        }
        else { // check for the end of the transition
            const latest_segment_index = this.segments.length - 1;
            const latest_segment = this.segments[latest_segment_index];
            if (latest_segment.type === "steady") {
                latest_segment.data.push(incoming_data);
            }
            else if (latest_segment.type === "transition") {
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
    
                this.index++;
                const e2v_std = incoming_data["smoothed2_future_kalman_acceleration_std"];
                if (e2v_std < this.e2v_std_min) {
                    // console.log("New transition minumum discovered");
                    // we have a new minimum
                    this.e2v_std_min = e2v_std;
                    this.min_index = this.index;
                }
    
                this.tmp_buffer.push(incoming_data);
    
                if (this.check_for_escape && (e2v_std > (this.first_e2v_value_for_transition / 2))) {
                    // take buffer from start to where min_index is.... this defines the transition segment
                    // the remaining section of the buffer is steady state
    
                    // buffer until min_index
                    const transition_data = this.tmp_buffer.slice(0, this.min_index);
    
                    // buffer after min_index
                    const steady_state_data = this.tmp_buffer.slice(this.min_index); // to end
    
                    latest_segment.data = transition_data;
                    this.segments.push({ type: "steady", data: steady_state_data });
                }
    
                if (e2v_std < (this.first_e2v_value_for_transition / 2)) {
                    this.check_for_escape = true;
                }
            }
        }
    }

    async done() {
        // this will be called after return_promise_resolver is called.
        // turn off motor
        await this.word_sender.send_word("thrustui16", 0);
        await this.word_sender.send_word("stop");
        console2.info(`StepChange program finished`);
    }

    direction = 0;
    direction_str = "cw";
    constructor(input$: Observable<any>, word_sender: SendWord, direction_str = "cw", max_duty = 2047) {
        super(input$);
        this.max_duty = max_duty;
        this.word_sender = word_sender;
        this.direction_str = direction_str;
        if (direction_str === "cw") {
            this.direction = 0;
        }
        else if (direction_str === "ccw") {
            this.direction = 1;
        }
    }
}
