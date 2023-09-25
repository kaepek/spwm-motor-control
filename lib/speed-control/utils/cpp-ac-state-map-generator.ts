import { ACMap } from "../tasks/collect-acceleration-data.js";

const header = ``;
const get_direction_ac_map_definition = (name: string, data: Array<number>) => {
    const lines = [];
    lines.push(`// ${name} anti-cogging map over ${data.length} compressed angular steps`);
    lines.push(`#define ${name} {\\`);
    const data_last_idx = data.length - 1;
    data.forEach((item: number, idx: number) => {
        if (data_last_idx === idx) {
            // we are in the last element
            lines.push(`${item} }`);
        }
        else {
            lines.push(`${item},\\`);
        }
    });
    return lines.join("\n");
};


export function generate_ac_map_cpp(ac_map: ACMap) {
    const number_of_bins = Object.keys(ac_map.cw.transformed_angular_bins).length;
    const ac_map_cpp = `const float AC_MAP[2][${number_of_bins}] = {CW_AC_MAP, CCW_AC_MAP};`;
    const cw_data = Object.keys(ac_map.cw.transformed_angular_bins).map((angle_bin) => ac_map.cw.transformed_angular_bins[angle_bin]);
    const cw_map = get_direction_ac_map_definition("CW_AC_MAP", cw_data);
    const ccw_data = Object.keys(ac_map.ccw.transformed_angular_bins).map((angle_bin) => ac_map.ccw.transformed_angular_bins[angle_bin]);
    const ccw_map = get_direction_ac_map_definition("CCW_AC_MAP", ccw_data);
    return [
        header,
        cw_map,
        ccw_map,
        ac_map_cpp
    ].join("\n\n");
}
