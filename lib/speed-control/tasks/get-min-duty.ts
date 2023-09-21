
import { Task } from "../../../external/kaepek-io/lib/host/ts-adaptors/task.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";

export class GetMinDuty extends Task {
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
        // parseInt(((65534 / this.max_duty) * (this.current_duty as number)).toString())
        // 20425 / 65534
        console2.info(`GetMinDuty program running`);
        await this.word_sender.send_word("thrustui16", this.current_duty as number);
        await this.word_sender.send_word("reset");
        await this.word_sender.send_word("start");
        this.create_wait_logic();
        return super.run();
    }

    all_finished = false;

    async tick(incoming_data: any) {
        if (this.all_finished) return;
        // console.log("called tick");
        this.incoming_data = incoming_data;

        if (this.timeout_run === false) {
            // wait for timout atleast.
            return;
        }
        else if (this.escape_duty === true) {
            // timeout detect motion move on
            this.send_next_word();
        }
    }

    async done() {
        // this will be called after return_promise_resolver is called.
        // turn off motor
        this.idle_duty = this.current_duty;
        await this.word_sender.send_word("thrustui16", 0);
        await this.word_sender.send_word("stop");
        console2.info(`GetMinDuty program finished`);
        console2.success(`Found idle duty ${this.idle_duty}`);
        // return found start duty.
        return { "idle_duty": this.idle_duty }
    }

    constructor(input$: Observable<any>, word_sender: SendWord, max_duty = 2047) {
        super(input$);
        this.max_duty = max_duty;
        this.word_sender = word_sender;
    }
}
