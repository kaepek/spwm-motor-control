
import { Task } from "../../../external/kaepek-io/lib/host/ts-adaptors/task.js";
import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";
import { Observable } from "rxjs";
import { RotationDetector } from "../../rotation-detector.js";

async function delay(ms: number) {
    return new Promise<void>((resolve, reject) => {
        setTimeout(resolve, ms);
    })
}

export class CollectAccelerationData extends Task<RotationDetector> {
    max_duty: number;
    wait_time = 50;
    word_sender: SendWord;
    incoming_data: any;
    wait_timeout: any;
    start_duty: number | null = null;
    idle_duty: number | null = null;

    async run(state: any) {
        this.start_duty = state.start_duty;
        this.idle_duty = state.idle_duty;
        // parseInt(((65534 / this.max_duty) * (this.current_duty as number)).toString())
        // 20425 / 65534
        console2.info(`CollectAccelerationData program running`);
        console2.info("Sending word thrustui16", this.start_duty);
        await this.word_sender.send_word("thrustui16", 0);
        await delay(100);
        await this.word_sender.send_word("thrustui16", this.start_duty as number);
        await delay(100);
        await this.word_sender.send_word("reset");
        await this.word_sender.send_word("start");
        await delay(1000);
        console2.info("Sending word thrustui16", this.idle_duty);
        await this.word_sender.send_word("thrustui16", this.idle_duty as number);

        setTimeout(()=>{
            return this.return_promise_resolver();
        }, 30000);
        return super.run(); // tick will now run every time the device outputs a line.
    }

    all_finished = false;

    async tick(incoming_data: RotationDetector) {

    }

    async done() {
        // this will be called after return_promise_resolver is called.
        // turn off motor
        await this.word_sender.send_word("thrustui16", 0);
        await this.word_sender.send_word("stop");
        console2.info(`CollectAccelerationData program finished`);
        return { }
    }

    constructor(input$: Observable<any>, word_sender: SendWord, max_duty = 2047) {
        super(input$);
        this.max_duty = max_duty;
        this.word_sender = word_sender;
    }
}
