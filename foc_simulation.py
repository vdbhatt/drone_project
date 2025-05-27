from bldc_motor_model import BLDCMotor
from bldc_controller import FOCController
import matplotlib.pyplot as plt
import math
import numpy as np

# Motor and Controller Parameters
# Motor Parameters
R_phase = 0.2  # Ohms
L_phase = 0.0002 # Henrys
Ke = 0.05      # Back-EMF constant (Volt-seconds/radian)
J_rotor = 0.0001 # Rotor inertia (kg*m^2)
friction = 0.0001 # Friction coefficient

# Controller Parameters
V_dc_bus = 12.0  # Volts

# FOC Gains (these will need tuning)
k_p_d = 0.1
k_i_d = 0.01
k_p_q = 0.15
k_i_q = 0.02

# Voltage Limits for FOC
voltage_limit_d = V_dc_bus / 2.0 # Example
voltage_limit_q = V_dc_bus / 2.0 # Example

# Target Currents
target_id = 0.0
target_iq_setpoint = 1.0 # Amperes - this will be our command for torque

# Instantiate Motor and Controller
motor = BLDCMotor(
    phase_resistance=R_phase,
    phase_inductance=L_phase,
    back_emf_constant=Ke,
    rotor_inertia=J_rotor,
    friction_coefficient=friction
)

foc_controller = FOCController(
    k_p_d=k_p_d, k_i_d=k_i_d,
    k_p_q=k_p_q, k_i_q=k_i_q,
    voltage_limit_d=voltage_limit_d,
    voltage_limit_q=voltage_limit_q,
    target_id=target_id
)

# Simulation Setup
time_step = 0.00001  # 10 us
simulation_duration = 1.0 # seconds

# Initialize lists for logging data
time_log = []
speed_log = []
angle_log = []
Ia_log, Ib_log, Ic_log = [], [], []
Va_log, Vb_log, Vc_log = [], [], []
Id_log, Iq_log = [], []
Vd_log, Vq_log = [], []
target_iq_log = [] # For plotting the setpoint

# Simulation Loop
num_steps = int(simulation_duration / time_step)
for step in range(num_steps):
    current_time = step * time_step

    # Get current motor state
    rotor_angle = motor.rotor_angle
    Ia, Ib, Ic = motor.phase_currents[0], motor.phase_currents[1], motor.phase_currents[2]

    # FOC control step
    # For simplicity, target_iq can be constant, or varied (e.g., step change)
    if current_time > 0.1: # Example: Apply torque command after 0.1s
        current_target_iq = target_iq_setpoint
    else:
        current_target_iq = 0.0
    
    target_iq_log.append(current_target_iq) # Log the target_iq

    Va, Vb, Vc = foc_controller.control_step(Ia, Ib, Ic, rotor_angle, current_target_iq, time_step)

    # Update motor state
    motor.update_state([Va, Vb, Vc], time_step)

    # Log data
    time_log.append(current_time)
    speed_log.append(motor.rotor_speed * (60 / (2 * math.pi))) # RPM
    angle_log.append(rotor_angle)
    Ia_log.append(Ia)
    Ib_log.append(Ib)
    Ic_log.append(Ic)
    Va_log.append(Va)
    Vb_log.append(Vb)
    Vc_log.append(Vc)

    # Log d-q currents and voltages
    # Re-calculate Clarke transform for logging Id_measured, Iq_measured
    Ialpha_measured, Ibeta_measured = foc_controller.clarke_transform(Ia, Ib, Ic)
    Id_measured, Iq_measured = foc_controller.park_transform(Ialpha_measured, Ibeta_measured, rotor_angle)
    Id_log.append(Id_measured)
    Iq_log.append(Iq_measured)
    
    # Access stored saturated Vd and Vq from the controller
    Vd_log.append(foc_controller.Vd_saturated)
    Vq_log.append(foc_controller.Vq_saturated)

# Plotting (after the loop)
plt.figure(figsize=(15, 12))

# Rotor speed (RPM) vs. time
plt.subplot(5, 1, 1)
plt.plot(time_log, speed_log, label='Rotor Speed (RPM)')
plt.title('Rotor Speed vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Speed (RPM)')
plt.grid(True)
plt.legend()

# Phase currents (Ia, Ib, Ic) vs. time
plt.subplot(5, 1, 2)
plt.plot(time_log, Ia_log, label='Ia')
plt.plot(time_log, Ib_log, label='Ib')
plt.plot(time_log, Ic_log, label='Ic')
plt.title('Phase Currents vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()
plt.grid(True)

# d-q currents (Id, Iq_measured, target_iq) vs. time
plt.subplot(5, 1, 3)
plt.plot(time_log, Id_log, label='Id_measured')
plt.plot(time_log, Iq_log, label='Iq_measured')
plt.plot(time_log, target_iq_log, label='target_Iq', linestyle='--')
plt.title('d-q Currents vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()
plt.grid(True)

# Applied phase voltages (Va, Vb, Vc from FOC) vs. time
plt.subplot(5, 1, 4)
plt.plot(time_log, Va_log, label='Va')
plt.plot(time_log, Vb_log, label='Vb')
plt.plot(time_log, Vc_log, label='Vc')
plt.title('Applied Phase Voltages vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend()
plt.grid(True)

# d-q voltages (Vd, Vq from FOC controller output) vs. time
plt.subplot(5, 1, 5)
plt.plot(time_log, Vd_log, label='Vd_output')
plt.plot(time_log, Vq_log, label='Vq_output')
plt.title('d-q Voltages (Controller Output) vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
