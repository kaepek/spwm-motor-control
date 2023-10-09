
import { Task } from "../../../external/kaepek-io/lib/host/ts-adaptors/task.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable, Subject } from "rxjs";
import { ESCParsedLineData, RotationDetector } from "../../rotation-detector.js";

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
