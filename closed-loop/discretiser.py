"""
Discretiser

Takes input:

cw_zero_displacement = -1.86 # [deg]
ccw_zero_displacement = 23.85 # [deg]
cw_phase_displacement = 240.1 # [deg]
ccw_phase_displacement = 240.1 # [deg]
number_of_poles = 14
encoder_divisions = 16384 # 2^14
encoder_compression_factor = 1    
"""

"""
Then uses this to compute via a sin model the relevant phase voltages.
"""


import numpy as np
import matplotlib.pyplot as plt


def deg_to_rad(deg):
    return deg * np.pi/180

def rad_to_deg(rad):
    return rad * 180/np.pi

def get_sin_model(poles):
    sin_period_coeff = (poles / 2)
    def sin_model(angular_position, angular_displacement, phase_current_displacement):
        coefficient=1.0
        phase_a_current = np.zeros((1,angular_position.shape[0]))# 0
        phase_b_current = np.zeros((1,angular_position.shape[0]))# 0
        phase_c_current = np.zeros((1,angular_position.shape[0])) # 0
        phase_a_current += coefficient * np.sin(sin_period_coeff * (angular_position + angular_displacement))
        phase_b_current += coefficient * np.sin(sin_period_coeff * (angular_position + angular_displacement + phase_current_displacement))
        phase_c_current += coefficient * np.sin(sin_period_coeff * (angular_position + angular_displacement + (2 * phase_current_displacement)))
        return np.asarray([phase_a_current, phase_b_current, phase_c_current]).ravel()
    return sin_model

def mmap(lamb, values):
    return list(map(lamb, values))

def create_voltage_scatter(ax,independant_axis_data,dependant_axis_data):
    mmap(lambda x: ax.scatter(independant_axis_data,dependant_axis_data[x[0]],zorder=1, color=x[1], s=1, label=x[2]),[[0, "red", "a-vn"],[1, "orange", "b-vn"],[2, "black","c-vn"]])
    ax.legend(loc="center right")
    ax.set_xlim(left=0, right=2*np.pi)
    ax.hlines(y=[0], xmin=[0], xmax=[2*np.pi], colors='purple', linestyles='--', lw=1, label='Multiple Lines')

def get_voltage_bins(encoder_divisions, number_of_poles, encoder_compression_factor, cw_zero_displacement, cw_phase_displacement, ccw_zero_displacement, ccw_phase_displacement):
    # get a range for angular_position in radians
    angular_resolution = int(encoder_divisions / encoder_compression_factor) # must be int
    angular_position = [((2*float(i)*np.pi)/float(angular_resolution)) for i in range(angular_resolution)]
    assert (angular_position[len(angular_position) - 1] + ((2*np.pi)/angular_resolution)) == (2*np.pi)
    print(angular_position)
    print(len(angular_position))
    angular_position = np.asarray(angular_position)
    # get sin model
    model = get_sin_model(number_of_poles)
    cw_data = model(angular_position, deg_to_rad(cw_zero_displacement), deg_to_rad(cw_phase_displacement))
    ccw_data = model(angular_position, deg_to_rad(ccw_zero_displacement), deg_to_rad(ccw_phase_displacement))

    print("cw_data", len(cw_data), cw_data)
    print("ccw_data", len(ccw_data), ccw_data)

    plot_title = 'Fit parameters:\n cw angular_disp=%.2f phase_current_disp=%.2f\n' % (cw_zero_displacement, cw_phase_displacement)
    plot_title += 'ccw angular_disp=%.2f phase_current_disp=%.2f' % (ccw_zero_displacement, ccw_phase_displacement)

    fig, ax = plt.subplots(nrows=2, ncols=1, figsize=(240, 20))
    fig.suptitle(plot_title,fontsize=20)

    create_voltage_scatter(ax[0],angular_position,cw_data.reshape(3, angular_position.shape[0])) 
    create_voltage_scatter(ax[1],angular_position,ccw_data.reshape(3, angular_position.shape[0])) 

    fig.savefig("discretiser-test.png", pad_inches=0, bbox_inches='tight')


cw_zero_displacement = -1.86 # [deg]
ccw_zero_displacement = 23.85 # [deg]
cw_phase_displacement = 240.1 # [deg]
ccw_phase_displacement = 240.1 # [deg]
number_of_poles = 14
encoder_divisions = 16384 # 2^14
encoder_compression_factor = 4

get_voltage_bins(encoder_divisions, number_of_poles, encoder_compression_factor, cw_zero_displacement, cw_phase_displacement, ccw_zero_displacement, ccw_phase_displacement)


"""
How would we lookup values....

use models to create sin tables over compressed range e.g. 0->4095
then take new encoder value (e.g. 16383) 16383 / (compressions 4) = 4095.75 => round nearest => 4096 => take mod => 0
lookup zeroth position index voltage values and apply them to pwm 
"""