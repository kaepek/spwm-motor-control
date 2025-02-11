
import { Task } from "../../../../external/kaepek-io/lib/host/controller/utils/task.js";
import { SendWord } from "../../../../external/kaepek-io/lib/host/controller/utils/send-word.js";
import { console2 } from "../../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";
import { ESCParsedLineData, RotationDetector } from "../../rotation-detector.js";
import { delay } from "../utils/delay.js";

export class GetIdleDuty extends Task<RotationDetector<ESCParsedLineData>> {
    max_duty: number;
    initial_duty = 0;
    wait_time = 6;
    current_duty: number | null = null;
    word_sender: SendWord;
    incoming_data: any;
    wait_timeout: any;
    start_duty: number | null = null;
    idle_duty: number | null = null;
    timeout_run = false;
    escape_duty = false;
    all_finished = false;
    rotations_start = 0;
    direction = 0;
    direction_str = "cw";

    async send_next_word() {
        (this.current_duty as number)--;
        let finished = false;
        if (this.current_duty as number > this.max_duty) {
            this.current_duty = this.max_duty;
            finished = true;
        }
        const mapped_duty = parseInt(((65534 / this.max_duty) * (this.current_duty as number)).toString());
        console2.info("Sending word thrustui16", mapped_duty);
        this.word_sender.send_word("thrustui16", mapped_duty);
        this.create_wait_logic();
        this.rotations_start = this.incoming_data.rotations;
        return finished;
    }

    create_wait_logic() {
        this.escape_duty = false;
        this.timeout_run = false;
        clearTimeout(this.wait_timeout);
        this.wait_timeout = setTimeout(() => {
            // incoming data should have motion
            if (!this.incoming_data) {
                return this.return_promise_rejector(`First message from peripheral device not received in ${this.wait_time} milliseconds, check that a director is running or increase wait_time.`);
            }
            else {
                if (this.incoming_data.motion == true) {
                    // escape to next duty
                    this.escape_duty = true;
                }
                else {
                    this.all_finished = true;
                    return this.return_promise_resolver();
                }
                this.timeout_run = true;
            }
            this.timeout_run = true;
        }, this.wait_time);
    }

    async run(state: any) {
        this.start_duty = state[this.direction_str].start_duty;
        const startup_time = state[this.direction_str].start_time;
        this.current_duty = Math.round(this.start_duty as number * (this.max_duty / 65534));
        console2.info(`GetMinDuty program running`);
        console2.info(`Resetting controller please wait...`);
        await delay(300);
        await this.word_sender.send_word("thrustui16", 0);
        await delay(300);
        await this.word_sender.send_word("directionui8", this.direction);
        await delay(100);
        await this.word_sender.send_word("reset");
        await delay(300);
        await this.word_sender.send_word("start");
        console2.info(`Starting motor with startup duty: ${this.current_duty}`);
        console2.info("Sending word thrustui16", this.start_duty);
        await this.word_sender.send_word("thrustui16", this.start_duty as number);
        console2.warn(`Waiting for motor startup: ${startup_time} [ms]`);
        await delay(startup_time);
        this.create_wait_logic();
        return super.run(); // tick will now run every time the device outputs a line. 
    }

    async tick(incoming_data: RotationDetector<ESCParsedLineData>) {
        if (this.all_finished) return;
        this.incoming_data = incoming_data;
        if (this.timeout_run === false) {
            // wait for timout atleast.
            return;
        }
        else if (this.escape_duty === true) {
            // timeout detect motion move on
            if (this.incoming_data.motion == false) {
                this.all_finished = true;
                return this.return_promise_resolver();
            }
            if ((incoming_data["rotations"] as number - this.rotations_start) > 1.1) {
                // we have n complete rotations
                this.send_next_word();
            }
        }
    }

    async done() {
        // this will be called after return_promise_resolver is called.
        // turn off motor
        await this.word_sender.send_word("thrustui16", 0);
        const final_duty_before_stall = parseInt(((65534 / this.max_duty) * ((this.current_duty as number) + 10)).toString());
        console2.info(`GetMinDuty program finished`);
        console2.success(`Found idle duty ${final_duty_before_stall}`);
        // return found start duty.
        return { [this.direction_str]: { "idle_duty": final_duty_before_stall }};
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
        }
        else {
            throw `Unrecognised direction ${direction_str} should be 'cw' or 'ccw'`;
        }
    }
}
