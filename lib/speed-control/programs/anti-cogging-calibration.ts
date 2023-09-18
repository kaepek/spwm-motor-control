import NetworkAdaptor from "../../../external/kaepek-io/lib/host/ts-adaptors/network.js";
import { parseArgs } from "node:util";
import fs from "fs";
import { Observable, Subscription } from "rxjs";

import { rotation_detector } from "../../rotation-detector.js";
import { Task } from "../../../external/kaepek-io/lib/host/ts-adaptors/tasks.js";

import { SendWord } from "../../../external/kaepek-io/lib/host/ts-adaptors/send-word.js";
import { console2 } from "../../../external/kaepek-io/lib/host/controller/utils/log.js";


const parse_options: any = {
    options: {
        input_config_file: {
            type: "string",
            short: "c"
        },
        output_data_file: {
            type: "string",
            short: "o"
        },
        incoming_address: {
            type: "string",
            short: "a"
        },
        incoming_port: {
            type: "string",
            short: "p"
        },
        incoming_protocol: {
            type: "string",
            short: "n"
        },
        outgoing_address: {
            type: "string",
            short: "o",
        },
        outgoing_port: {
            type: "string",
            short: "x"
        },
        outgoing_protocol: {
            type: "string",
            short: "v"
        },
        command_address: {
            type: "string",
            short: "y",
        },
        command_port: {
            type: "string",
            short: "u"
        },
        command_protocol: {
            type: "string",
            short: "m"
        },
    }
};

let parsed_options: any = { values: {}, positionals: [] };

try {
    parsed_options = parseArgs(parse_options as any);
}
catch (e: any) {
    console.error(`AntiCoggingCalibration argument parser error: ${e.message}`);
    process.exit(1);
}

const missing_options: Array<string> = [];
Object.keys(parse_options.options).forEach((option_name) => {
    if (!parsed_options.values[option_name] && ((option_name !== "outgoing_address") && (option_name !== "outgoing_port") && (option_name !== "outgoing_protocol"))) {
        missing_options.push(option_name);
    }
});

if (missing_options.length !== 0) {
    console.error(`AntiCoggingCalibration: Missing the following arguments ${missing_options.map(option_str => {
        const option = (parse_options.options)[option_str];
        return `--${option_str} or -${option.short}`
    }).join(", ")}`);
    process.exit(1);
}

const values = parsed_options.values;

// check if files exists
const cwd = process.cwd();

const full_output_data_path = `${cwd}/${values["output_data_file"]}`;
const output_data_path_exists = fs.existsSync(full_output_data_path);
if (output_data_path_exists) {
    console.error(`AntiCoggingCalibration: output_data_file ${full_output_data_path} exists already.`);
    process.exit(1);
}


const full_input_config_path = `${cwd}/${values["input_config_file"]}`;
const input_config_exists = fs.existsSync(full_input_config_path);
if (!input_config_exists) {
    console.error(`AntiCoggingCalibration: input_config_file ${full_input_config_path} does not exist`);
    process.exit(1);
}

const outgoing_address = values["outgoing_address"] || null;
let outgoing_port = values["outgoing_port"] || null;
if (outgoing_port !== null) {
    outgoing_port = parseFloat(outgoing_port);
}
const outgoing_protocol = values["outgoing_protocol"];

const command_address = values["command_address"];
const command_port = parseFloat(values["command_port"]);
const command_protocol = values["command_protocol"];

if (command_protocol !== "udp") {
    console.error(`AntiCoggingCalibration: unknown command_protocol ${command_protocol} expects 'upd'`);
    process.exit(1);
}

console.log("VALUEs", values);


const word_sender = new SendWord(command_address, command_port, command_protocol);

console.log("about to init network adaptor", values);
const adaptor = new NetworkAdaptor(values["incoming_address"], parseFloat(values["incoming_port"]), values["incoming_protocol"], values["input_config_file"], ",", outgoing_address, outgoing_port, outgoing_protocol);

// what tasks do we need for this program

class GetStartDuty extends Task {
    max_duty: number;
    initial_duty = 0;
    wait_time = 10;
    current_duty: number | null = null;
    word_sender: SendWord;

    incoming_data: any;

    wait_timeout: any;

    start_duty: number | null = null;

// ((double)com_torque_value / (double)65535) * 0.5;

    async send_next_word() {
        (this.current_duty as number)++;
        let finished = false;
        if (this.current_duty as number > this.max_duty) {
            this.current_duty = this.max_duty;
            finished = true;
        }

        const mapped_word = parseInt(((65534 / this.max_duty) * (this.current_duty as number)).toString());
        console2.info("Sending word thrustui16", mapped_word);
        this.word_sender.send_word("thrustui16", mapped_word);
        return finished;
    }

    create_wait_logic() {
        this.escape_duty = false;
        this.timeout_run = false;
        this.wait_timeout = setTimeout(() => {
            // incoming data should have motion
            if (this.incoming_data.motion == false) {
                // escape to next duty
                this.escape_duty = true;
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

    tick(incoming_data: any) {
        // console.log("called tick");
        this.incoming_data = incoming_data;

        if (this.timeout_run === false) {
            // wait for timout atleast.
        }
        else if (this.escape_duty === true) {
            if (this.current_duty === this.max_duty) {
                this.word_sender.send_word("thrustui16", 0);
                this.word_sender.send_word("stop");
                return this.return_promise_rejector("Reached max duty an rotation was not detected");
            }
            // timeout detect lack of motion move on
            this.send_next_word();
            this.create_wait_logic();
        }
        else if (this.escape_duty === false) {
            if (incoming_data["motion"] === false) {
                // stop motion
                this.send_next_word();
                this.create_wait_logic();
            }
            else {
                // we still have motion
                if (incoming_data["rotations"] > 5) {
                    // we have 5 complete rotations
                    this.start_duty = this.current_duty;
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



const rotation$ = rotation_detector(adaptor.incoming_data$, true);

const get_start_duty_task = new GetStartDuty(rotation$, word_sender);

const tasks = [get_start_duty_task];

// need to parse state to each task when we are running it

async function main() {
    let state = {};

    // should call run then call wait
    let resolver: Promise<any> = Promise.resolve();

    await adaptor.ready();

    tasks.forEach((task) => {
        resolver = resolver.then(async () => {
            const return_prom = task.wait().then((state_data_additions) => {
                state = { ...state_data_additions, ...state };
                return state_data_additions;
            });
            await task.run(state);
            return return_prom;
        });
    });

    return resolver;
}

main().then(output => {
    console2.success("All finished, result:", JSON.stringify(output));
    process.exit(0);
}).catch((err) => {
    console2.error(err);
    process.exit(1);
});
