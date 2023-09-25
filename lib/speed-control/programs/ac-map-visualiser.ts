import { ArgumentHandlers, CliArg, CliArgType, parse_args } from "../../../external/kaepek-io/lib/host/ts-adaptors/cli-args.js";
import { ASCIIParser } from "../../../external/kaepek-io/lib/host/ts-adaptors/ascii-parser.js"
import { ACMap } from "../tasks/collect-acceleration-data";
import fs from "fs";

const cli_args: Array<CliArg> = [
    {
        name: "input_file",
        type: CliArgType.InputJSONFile, // InputJSONFilePathArgumentHandler
        short: "i",
        required: true
    },
    {
        name: "input_config_file",
        type: CliArgType.InputJSONFile,
        short: "c",
        required: true
    },
    {
        name: "output_file",
        type: CliArgType.OutputFilePath,
        short: "o",
        required: true
    },
];

const parsed_args = parse_args("AntiCoggingCalibrationGrapher", cli_args, ArgumentHandlers) as any;
const graph_config = parsed_args.input_config_file;
const ac_map = parsed_args.input_file as ACMap;

const parser = new ASCIIParser(graph_config.inputs, ",");

const lines: Array<string> = [];

Object.keys(ac_map.cw.transformed_angular_acc_bins).forEach((angle_str, idx) => {
    const angle_number = parseFloat(angle_str);
    lines.push(parser.serialise(
        {
            compressed_angle: angle_number,
            cw_duty_modifier: ac_map.cw.transformed_angular_acc_bins[angle_str],
            ccw_duty_modifier: ac_map.ccw.transformed_angular_acc_bins[angle_str],
            cw_mean_vel: ac_map.cw.mean_vel[angle_str],
            ccw_mean_vel: ac_map.ccw.mean_vel[angle_str],
            cw_mean_acc: ac_map.cw.mean[angle_str],
            ccw_mean_acc: ac_map.ccw.mean[angle_str],
        }
    ));
}); 

const output = lines.join("\n");

fs.writeFileSync(`${parsed_args.output_file}`, output);

