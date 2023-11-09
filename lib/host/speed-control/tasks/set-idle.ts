
import { Task } from "../../../../external/kaepek-io/lib/host/controller/utils/task.js";
import { SendWord } from "../../../../external/kaepek-io/lib/host/controller/utils/send-word.js";
import { console2 } from "../../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";
import { ESCParsedLineData, RotationDetector } from "../../rotation-detector.js";
import { delay } from "../utils/delay.js";

export class SetIdleDuty extends Task<RotationDetector<ESCParsedLineData>> {
    start_duty: number | null = null;
    idle_duty: number | null = null;
    word_sender: SendWord;
    wait_time: number;
    wait_timeout: any;
    timeout_run = false;
    min_speed: number = Number.POSITIVE_INFINITY;
    direction = 0;
    direction_str = "cw";
    max_duty: number = 2047;
    idle_duty_override: number | undefined;

    async run(state: any) {
        const start_duty = state[this.direction_str].start_duty;
        const start_time = state[this.direction_str].start_time;
        let idle_duty = state[this.direction_str].idle_duty;

        if (this.idle_duty_override !== undefined) {
            idle_duty = this.idle_duty_override;
        }
        this.idle_duty = idle_duty;
        
        this.start_duty = start_duty;
        if (this.idle_duty_override !== undefined && this.idle_duty_override === 0) {
            this.start_duty = this.idle_duty_override;
        }

        console2.info(`SetIdleDuty program running, targeting an idle thrustui16 value of ${this.idle_duty}`);
        
        if (this.start_duty !== 0) {
            console2.info("Sending word thrustui16", this.start_duty);
            await this.word_sender.send_word("thrustui16", this.start_duty as number);
            console2.warn(`Waiting for startup ${start_time} [s].`);
            await delay(start_time);
            console2.success("Startup wait done.");
        }

        console2.info("Attempting idle or set speed.");
        console2.info("Sending word thrustui16.", this.idle_duty);
        await this.word_sender.send_word("thrustui16", this.idle_duty as number);
        console2.warn("Checking for motor idle.");
        this.create_timeout();
        return super.run(); // tick will now run every time the device outputs a line. 
    }

    async create_timeout() {
        if (this.wait_timeout) clearTimeout(this.wait_timeout);
        this.wait_timeout = setTimeout(async () => {
            console2.success("Detected motor idle.");
            this.return_promise_resolver();
        }, this.wait_time);
    }
    
    async tick(incoming_data: RotationDetector<ESCParsedLineData>) {
        const current_speed = Math.abs(incoming_data.line_data.parsed_data.kalman_velocity);
        if (current_speed < this.min_speed) { // should have some sort of check to see if stalled. todo
            this.min_speed = current_speed;
            this.create_timeout();
        }
    }

    async done() {
        console2.info(`SetIdleDuty program finished, motor should now be idleing`);
        return {}
    }

    constructor(input$: Observable<any>, word_sender: SendWord, direction_str = "cw", idle_duty_override: undefined | number = undefined, max_duty = 2047, wait_time = 3000) {
        super(input$);
        this.max_duty = max_duty;
        this.word_sender = word_sender;
        this.direction_str = direction_str;
        this.wait_time = wait_time;
        this.idle_duty_override = idle_duty_override;
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
