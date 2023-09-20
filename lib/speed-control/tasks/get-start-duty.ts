
import { Task } from "../../../external/kaepek-io/lib/host/ts-adaptors/tasks.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";

export class GetStartDuty extends Task {
    max_duty: number;
    initial_duty = 0;
    wait_time = 6;
    current_duty: number | null = null;
    word_sender: SendWord;
    incoming_data: any;
    wait_timeout: any;
    start_duty: number | null = null;
    resolver = Promise.resolve();

    async send_next_word() {
        this.resolver = this.resolver.then(() => {

            return new Promise((resolve, reject) => {
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
                this.create_wait_logic(resolve);
                return finished;
            });

        });
    }

    create_wait_logic(resolver?: any) {
        this.escape_duty = false;
        this.timeout_run = false;
        clearTimeout(this.wait_timeout);
        this.wait_timeout = setTimeout(() => {
            // incoming data should have motion
            if (this.incoming_data.motion == false) {
                // escape to next duty
                this.escape_duty = true;
            }
            if (resolver) {
                resolver();
            }
            this.timeout_run = true;
        }, this.wait_time);
    }

    timeout_run = false;
    escape_duty = false;
    async run(state: any) {
        this.current_duty = 0;
        await this.word_sender.send_word("thrustui16", this.current_duty as number);
        await this.word_sender.send_word("reset");
        await this.word_sender.send_word("start");
        console2.info(`GetStartDuty program running`);
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
        }
        else if (this.escape_duty === true) {
            if (this.current_duty === this.max_duty) {
                this.all_finished = true;
                // await new Promise((resolve) => setTimeout(resolve, 1000));
                await this.word_sender.send_word("thrustui16", 0);
                await this.word_sender.send_word("stop");
                return this.return_promise_rejector("Reached max duty an rotation was not detected");
            }
            // timeout detect lack of motion move on
            this.send_next_word();
            // this.create_wait_logic();
        }
        else if (this.escape_duty === false) {
            if (incoming_data["motion"] === false) {
                // stop motion
                this.send_next_word();
                // this.create_wait_logic();
            }
            else {
                // we still have motion
                if (incoming_data["rotations"] > 5) {
                    // we have 5 complete rotations
                    return this.return_promise_resolver();
                }
            }
        }
    }

    async done() {
        // this will be called after return_promise_resolver is called.
        // turn off motor
        await this.word_sender.send_word("thrustui16", 0);
        await this.word_sender.send_word("stop");
        console2.info(`GetStartDuty program finished`);
        console2.success(`Found start duty ${this.start_duty}`);
        // return found start duty.
        return { "start_duty": this.start_duty }
    }

    constructor(input$: Observable<any>, word_sender: SendWord, max_duty = 2047) {
        super(input$);
        this.max_duty = max_duty;
        this.word_sender = word_sender;
    }
}
