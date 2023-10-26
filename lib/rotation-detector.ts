import { Observable, pipe } from "rxjs";
import { filter, map } from "rxjs/operators";

/*
{
            motion: true,
            rotations: 49.01000000000022,
            line_data: {
                parsed_data: {
                    kalman_hz: 33678.19,
                    sample_hz: 263922.52,
                    time: 2670.0623,
                    com_thrust_percentage: 0.1158,
                    com_direction: 0,
                    eular_displacement: 87372444,
                    eular_velocity: 0,
                    eular_acceleration: 0,
                    eular_jerk: 0,
                    kalman_displacement: 5332.79,
                    kalman_velocity: 1.8,
                    kalman_acceleration: 1.09,
                    kalman_jerk: 185.15,
                    voltage_phase_a: 7,
                    voltage_phase_b: 140,
                    voltage_phase_c: 209,
                    encoder_raw_displacement: 12956
                },
                info: { address: '127.0.0.1', family: 'IPv4', port: 55470, size: 105 }
            }
        }

*/

export type ESCParsedLineData = {
    parsed_data: {
        kalman_hz: number,
        sample_hz: number,
        time: number,
        com_thrust_percentage: number,
        com_direction: number,
        eular_displacement: number,
        eular_velocity: number,
        eular_acceleration: number,
        eular_jerk: number,
        kalman_displacement: number,
        kalman_velocity: number,
        kalman_acceleration: number,
        kalman_jerk: number,
        voltage_phase_a: number,
        voltage_phase_b: number,
        voltage_phase_c: number,
        encoder_raw_displacement: number
    },
    info: { address: string, family: string, port: number, size: number }
};

export type RotationDetector<LineData> = {
    motion: boolean;
    rotations: number;
    line_data: LineData;
} | {
    motion: boolean;
    line_data: LineData;
    rotations?: undefined;
};

export function rotation_detector(data_line$: Observable<any>, direction = true, displacement_max = 1) { // true cw, false ccw
    if (direction != true && direction != false) {
        throw "Unrecognised direction boolean, true is cw (clockwise), false is ccw (counter-clockwise)"
    }
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
            return { motion: false, rotations: 0, line_data } // Math.abs(displacement - initial_displacment) / displacement_max
        };
    }));
    return $;
}