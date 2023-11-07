
import { Task } from "../../../external/kaepek-io/lib/host/controller/utils/task.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/controller/utils/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";
import { ESCParsedLineData, RotationDetector } from "../../rotation-detector.js";
import { delay } from "../utils/delay.js";

export class GetStartDuty extends Task<RotationDetector<ESCParsedLineData>> {
    max_duty: number;
    initial_duty = 0;
    wait_time = 6;
    current_duty: number | null = null;
    word_sender: SendWord;
    incoming_data: any;
    wait_timeout: any;
    start_duty: number | null = null;
    start_time: Date | null = null;
    end_time: Date | null = null;
    spin_up_down_wait_timeout: any;
    spin_up_down_wait_time = 3000;
    direction = 0;
    direction_str = "cw";
    direction_sign = 1.0;
    timeout_run = false;
    escape_duty = false;
    state: any = null;
    all_finished = false;
    min_speed: number = Number.POSITIVE_INFINITY;
    max_speed: number = Number.NEGATIVE_INFINITY;
    max_speed_detected = false;
    zero_speed_detected = false;
    running_spin_up_routine = false;
    running_spin_down_routine = false;
    exit_routine = false;

    async send_next_word() {
        (this.current_duty as number)++;
        let finished = false;
        if (this.current_duty as number > this.max_duty) {
            this.current_duty = this.max_duty;
            finished = true;
        }
        const mapped_duty = parseInt(((65534 / this.max_duty) * (this.current_duty as number)).toString());
        console2.info("Sending word thrustui16", mapped_duty);
        this.start_time = new Date();
        this.word_sender.send_word("thrustui16", mapped_duty);
        this.start_duty = mapped_duty;
        this.create_wait_logic();
        return finished;
    }

    create_wait_logic() {
        this.escape_duty = false;
        this.timeout_run = false;
        clearTimeout(this.wait_timeout);
        this.wait_timeout = setTimeout(() => {
            // incoming data should have motion
            if (!this.incoming_data) {
                return this.return_promise_rejector(`First message from peripheral device not received in ${this.wait_time} milliseconds`);
            }
            else {
                if (this.incoming_data.motion == false) {
                    // escape to next duty
                    this.escape_duty = true;
                }
                this.timeout_run = true;
            }
            this.timeout_run = true;
        }, this.wait_time);
    }

    async run(state: any) {
        this.state = state;
        this.current_duty = 0;
        console2.info(`GetStartDuty program running`);
        await delay(300);
        await this.word_sender.send_word("thrustui16", 0);
        await delay(300);
        await this.word_sender.send_word("directionui8", this.direction);
        await delay(300);
        await this.word_sender.send_word("thrustui16", this.current_duty as number);
        await delay(300);
        await this.word_sender.send_word("reset");
        await delay(300);
        await this.word_sender.send_word("start");
        await delay(300);
        this.create_wait_logic();
        return super.run(); // tick will now run every time the device outputs a line.
    }

    check_spin_up_status(current_speed: number) {
        if (this.running_spin_up_routine === false) {
            this.running_spin_up_routine = true;
            console2.warn("Waiting for spin up.");
        }
        if (this.max_speed < current_speed) {
            this.max_speed = current_speed;
            if (this.spin_up_down_wait_timeout) clearTimeout(this.spin_up_down_wait_timeout);
            this.spin_up_down_wait_timeout = setTimeout(()=>{
                console2.success("Detected spin up.");
                this.max_speed_detected = true;
                this.end_time = new Date();
                this.word_sender.send_word("thrustui16", 0);
            },this.spin_up_down_wait_time);
        }
    }

    check_spin_down_status(current_speed: number) {
        if (this.running_spin_down_routine === false) {
            this.running_spin_down_routine = true;
            console2.warn("Waiting for spin down.");
        }
        if (this.min_speed > current_speed) {
            this.min_speed = current_speed;
            if (this.spin_up_down_wait_timeout) clearTimeout(this.spin_up_down_wait_timeout);
            this.spin_up_down_wait_timeout = setTimeout(()=>{
                console2.success("Detected spin down.");
                this.zero_speed_detected = true;
                this.return_promise_resolver();
            },this.spin_up_down_wait_time);
        }
    }

    async tick(incoming_data: RotationDetector<ESCParsedLineData>) {
        if (this.all_finished) return;
        const current_speed = incoming_data.line_data.parsed_data.kalman_velocity * this.direction_sign;
        this.incoming_data = incoming_data;
        if (this.timeout_run === false) {
            // wait for timout atleast.
            return;
        }
        else if (this.escape_duty === true) { // && this.exit_routine == false
            if (this.current_duty === this.max_duty) {
                this.all_finished = true;
                // await new Promise((resolve) => setTimeout(resolve, 1000));
                await this.word_sender.send_word("thrustui16", 0);
                await this.word_sender.send_word("stop");
                return this.return_promise_rejector("Reached max duty an rotation was not detected");
            }
            // timeout detect lack of motion move on
            this.send_next_word();
        }
        else if (this.escape_duty === false) {
            if (incoming_data["motion"] === false && this.exit_routine == false) {
                // stop motion
                this.send_next_word();
            }
            else {
                
                // we still have motion
                if ((incoming_data["rotations"] as number) > 5) { // need to compute startup time in ms
                    this.exit_routine = true;
                    // we have 5 complete rotations
                    if (this.max_speed_detected === false) {
                        this.check_spin_up_status(current_speed);
                    }
                    else if (this.zero_speed_detected === false) {
                        this.check_spin_down_status(current_speed);
                    }
                }
            }
        }
    }

    async done() {
        // this will be called after return_promise_resolver is called.
        // turn off motor
        await delay(300);
        await this.word_sender.send_word("thrustui16", 0);
        await delay(300);
        await this.word_sender.send_word("stop");
        console2.info(`GetStartDuty program finished`);
        const start_duty_with_excess = parseInt((this.start_duty as number * 1.0).toString());
        const actual_start_duty = Math.round(start_duty_with_excess as number * (this.max_duty / 65534));
        console2.success(`Found start duty ${actual_start_duty}`);
        const delta_time = (this.end_time as Date).getTime() - (this.start_time as Date).getTime();
        console2.success(`Found start time ${delta_time} [ms]`);

        // if we have an existing start duty for this direction then emit the maximum.
        if (this.state.hasOwnProperty(this.direction_str) && this.state[this.direction_str].hasOwnProperty("start_duty")) {
            // check existing start duty emit max
            if (start_duty_with_excess > this.state[this.direction_str].start_duty) {
                return { [this.direction_str]: { "start_duty": start_duty_with_excess, "start_time": delta_time } };
            }
            else {
                return { [this.direction_str]: { "start_duty": this.state[this.direction_str].start_duty, "start_time": this.state[this.direction_str].start_time } };
            }
        }

        return { [this.direction_str]: { "start_duty": start_duty_with_excess, "start_time": delta_time } };
    }

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
            this.direction_sign = -1.0;
        }
        else {
            throw `Unrecognised direction ${direction_str} should be 'cw' or 'ccw'`;
        }
    }
}
