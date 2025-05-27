import matplotlib.pyplot as plt
from bldc_motor_model import BLDCMotor
from bldc_controller import SixStepCommutator

# Motor parameters
PHASE_RESISTANCE = 0.2  # Ohms
PHASE_INDUCTANCE = 0.0002  # Henry
BACK_EMF_CONSTANT = 0.05  # Vs/rad (also Nm/A)
ROTOR_INERTIA = 0.0001  # kg.m^2
FRICTION_COEFFICIENT = 0.0001 # Nm/(rad/s)

# Controller parameters
DC_BUS_VOLTAGE = 12.0  # Volts

# Simulation parameters
TIME_STEP = 0.00001  # 10 us
SIMULATION_DURATION = 1.0  # 1 second

# Instantiate motor and controller
motor = BLDCMotor(
    phase_resistance=PHASE_RESISTANCE,
    phase_inductance=PHASE_INDUCTANCE,
    back_emf_constant=BACK_EMF_CONSTANT,
    rotor_inertia=ROTOR_INERTIA,
    friction_coefficient=FRICTION_COEFFICIENT
)
controller = SixStepCommutator(dc_bus_voltage=DC_BUS_VOLTAGE)

# Initialize lists to store results
time_log = []
rotor_speed_log = []
rotor_angle_log = []
phase_currents_a_log = []
phase_currents_b_log = []
phase_currents_c_log = []
applied_voltages_a_log = []
applied_voltages_b_log = []
applied_voltages_c_log = []

# Simulation loop
current_time = 0.0
while current_time < SIMULATION_DURATION:
    # Log current state
    time_log.append(current_time)
    rotor_speed_log.append(motor.rotor_speed)
    rotor_angle_log.append(motor.rotor_angle)
    phase_currents_a_log.append(motor.phase_currents[0])
    phase_currents_b_log.append(motor.phase_currents[1])
    phase_currents_c_log.append(motor.phase_currents[2])

    # Get applied voltages from controller
    applied_voltages = controller.get_phase_voltages(motor.rotor_angle)
    applied_voltages_a_log.append(applied_voltages[0])
    applied_voltages_b_log.append(applied_voltages[1])
    applied_voltages_c_log.append(applied_voltages[2])

    # Update motor state
    motor.update_state(applied_voltages, TIME_STEP)

    # Increment time
    current_time += TIME_STEP

# Plot results
plt.figure(figsize=(12, 10))

plt.subplot(4, 1, 1)
plt.plot(time_log, rotor_speed_log)
plt.title('Rotor Speed vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Speed (rad/s)')
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(time_log, rotor_angle_log)
plt.title('Rotor Angle vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(time_log, phase_currents_a_log, label='Ia')
plt.plot(time_log, phase_currents_b_log, label='Ib')
plt.plot(time_log, phase_currents_c_log, label='Ic')
plt.title('Phase Currents vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(time_log, applied_voltages_a_log, label='Va')
plt.plot(time_log, applied_voltages_b_log, label='Vb')
plt.plot(time_log, applied_voltages_c_log, label='Vc')
plt.title('Applied Phase Voltages vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
