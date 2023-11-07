
import { Task } from "../../../external/kaepek-io/lib/host/controller/utils/task.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/controller/utils/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";
import { ESCParsedLineData, RotationDetector } from "../../rotation-detector.js";

async function delay(ms: number) {
    return new Promise<void>((resolve, reject) => {
        setTimeout(resolve, ms);
    })
}

export class GetIdleDuty2 extends Task<RotationDetector<ESCParsedLineData>> {
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
    all_finished = false;
    rotations_start = 0;
    rotation_time_recorded = false;
    rotation_time:number | null = null;
    start_time : Date | null = null;
    first_run = true;
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
        this.start_time = new Date();
        this.rotations_start = this.incoming_data.rotations;
        this.rotation_time_recorded = false;
        this.create_wait_logic();
        return finished;
    }

    create_wait_logic() {
        this.timeout_run = false;
        clearTimeout(this.wait_timeout);
        const wait = Math.max(this.wait_time, 1);
        this.wait_timeout = setTimeout(() => {
            console2.log("WAITED", wait);
            // incoming data should have motion
            if (!this.incoming_data) {
                return this.return_promise_rejector(`First message from peripheral device not received in ${this.wait_time} milliseconds, sugest checking that a director is running or increase wait_time.`);
            }
            this.timeout_run = true;
        }, wait);
    }

    async run(state: any) {
        this.start_duty = state[this.direction_str].start_duty;
        this.current_duty = this.start_duty as number * (this.max_duty / 65534);
        console2.info(`GetMinDuty program running`);
        console2.info("Sending word thrustui16", this.start_duty);
        await this.word_sender.send_word("thrustui16", 0);
        await this.word_sender.send_word("directionui8", this.direction);
        await delay(100);
        await this.word_sender.send_word("thrustui16", this.start_duty as number);
        await delay(100);
        await this.word_sender.send_word("reset");
        await this.word_sender.send_word("start");
        await delay(1000);
        return super.run(); // tick will now run every time the device outputs a line. 
    }

    async tick(incoming_data: RotationDetector<ESCParsedLineData>) {
        if (this.all_finished) return;

        this.incoming_data = incoming_data;

        if (this.first_run && this.rotations_start === 0) {
            this.start_time = new Date();
            this.rotations_start = this.incoming_data.rotations;
            this.create_wait_logic();
        }

        const rotations = (incoming_data["rotations"] as number - this.rotations_start);

        if (rotations > 1 && this.rotation_time_recorded === false) {
            const end_time = new Date();
            const diff = (end_time.getTime() - (this.start_time as Date).getTime());
            this.rotation_time = diff;
            this.rotation_time_recorded = true;
            this.wait_time = this.rotation_time * 1.8;
            console2.log("UPDATED WAIT TIME", this.wait_time, this.rotation_time);
        }

        if (this.timeout_run === false) {
            // wait for timout atleast.
            return;
        }
        else {
            console2.log("rotations", rotations, incoming_data["rotations"], this.rotations_start);
            console2.log(`ROTATIONS TIMEOUT DONE`, rotations, this.rotation_time, this.first_run);
            // timeout elapsed ... make sure we have made atleast a full rotation.
            if (rotations > 1.1) {
                this.first_run = false;
                this.send_next_word();
            } else {
                if (this.first_run !== true) { // have to permit the first rotation as we dont know the rotation time here.
                    this.all_finished = true;
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
        const final_duty_before_stall = parseInt(((65534 / this.max_duty) * ((this.current_duty as number) + 1)).toString());
        console2.info(`GetMinDuty program finished`);
        console2.success(`Found idle duty ${final_duty_before_stall}`);
        // return found start duty.
        return { [this.direction_str]: { "idle_duty": final_duty_before_stall + 75 }}; // add a bit more (currently using battery, it will discharge and may lose rotation)
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
    }
}
