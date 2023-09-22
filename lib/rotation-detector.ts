import { Observable, pipe } from "rxjs";
import { filter, map } from "rxjs/operators";

export type RotationDetector = {
    motion: boolean;
    rotations: number;
    line_data: any;
} | {
    motion: boolean;
    line_data: any;
    rotations?: undefined;
};

export function rotation_detector(data_line$: Observable<any>, direction = true, displacement_max = 1) { // false cw?
    let tracking = false;
    let initial_displacment: number = null as any;
    const $ = data_line$.pipe(map((line_data: any) => {
        const velocity = parseFloat(line_data.parsed_data["kalman_velocity"]);
        const displacement = parseFloat(line_data.parsed_data["kalman_displacement"]);
        if (tracking === false && ((direction == true && velocity > 0.0) || (direction == false && velocity < 0.0))) {
            initial_displacment = displacement;
            tracking = true;
            return { motion: true, rotations: Math.abs(displacement - initial_displacment) / displacement_max, line_data };
        }
        else if (tracking === true && (((direction == true && velocity > 0.0) || (direction == false && velocity < 0.0)))) {
            return { motion: true, rotations: Math.abs(displacement - initial_displacment) / displacement_max, line_data };
        }
        else {
            tracking = false;
            return { motion: false, line_data }
        };
    }));
    return $;
}