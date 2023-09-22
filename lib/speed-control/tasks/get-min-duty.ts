
import { Task } from "../../../external/kaepek-io/lib/host/ts-adaptors/task.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";
import { ESCParsedLineData, RotationDetector } from "../../rotation-detector.js";

async function delay(ms: number) {
    return new Promise<void>((resolve, reject) => {
        setTimeout(resolve, ms);
    })
}

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
                return this.return_promise_rejector(`First message from peripheral device not received in ${this.wait_time} milliseconds, sugest checking that a director is running or increase wait_time.`);
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

    timeout_run = false;
    escape_duty = false;
    async run(state: any) {
        this.start_duty = state.start_duty;
        this.current_duty = this.start_duty as number * (this.max_duty / 65534);
        console2.info(`GetMinDuty program running`);
        console2.info("Sending word thrustui16", this.start_duty);
        await this.word_sender.send_word("thrustui16", 0);
        await delay(100);
        await this.word_sender.send_word("thrustui16", this.start_duty as number);
        await delay(100);
        await this.word_sender.send_word("reset");
        await this.word_sender.send_word("start");
        await delay(1000);
        this.create_wait_logic();
        return super.run(); // tick will now run every time the device outputs a line. 
    }

    all_finished = false;
    rotations_start = 0;
    
    async tick(incoming_data: RotationDetector<ESCParsedLineData>) {
        if (this.all_finished) return;
        // console.log("called tick");
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
                // console.log("Rotations achieved", incoming_data["rotations"] - this.rotations_start);
                this.send_next_word();
            }
        }
    }

    async done() {
        // this will be called after return_promise_resolver is called.
        // turn off motor
        await this.word_sender.send_word("thrustui16", 0);
        await this.word_sender.send_word("stop");
        const final_duty_before_stall = parseInt(((65534 / this.max_duty) * ((this.current_duty as number) + 1)).toString());
        console2.info(`GetMinDuty program finished`);
        console2.success(`Found idle duty ${final_duty_before_stall}`);
        // return found start duty.
        return { "idle_duty": final_duty_before_stall }
    }

    constructor(input$: Observable<any>, word_sender: SendWord, max_duty = 2047) {
        super(input$);
        this.max_duty = max_duty;
        this.word_sender = word_sender;
    }
}
