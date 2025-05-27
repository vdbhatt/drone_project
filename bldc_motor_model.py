import math

class BLDCMotor:
    def __init__(self, phase_resistance, phase_inductance, back_emf_constant, rotor_inertia, friction_coefficient=0):
        # Motor parameters
        self.phase_resistance = phase_resistance
        self.phase_inductance = phase_inductance
        self.back_emf_constant = back_emf_constant
        self.rotor_inertia = rotor_inertia
        self.friction_coefficient = friction_coefficient

        # State variables
        self.rotor_angle = 0  # radians
        self.rotor_speed = 0  # radians/second
        self.phase_currents = [0, 0, 0]  # Amperes (A, B, C)
        self.phase_voltages = [0, 0, 0]  # Volts (A, B, C)

    def _calculate_back_emf(self):
        """Calculates the back-EMF for each phase."""
        emf_a = self.back_emf_constant * self.rotor_speed * math.sin(self.rotor_angle)
        emf_b = self.back_emf_constant * self.rotor_speed * math.sin(self.rotor_angle - 2 * math.pi / 3)
        emf_c = self.back_emf_constant * self.rotor_speed * math.sin(self.rotor_angle + 2 * math.pi / 3)
        return [emf_a, emf_b, emf_c]

    def _calculate_electromagnetic_torque(self):
        """Calculates the electromagnetic torque."""
        back_emf = self._calculate_back_emf()
        if self.rotor_speed == 0:
            return 0
        
        torque = (back_emf[0] * self.phase_currents[0] +
                  back_emf[1] * self.phase_currents[1] +
                  back_emf[2] * self.phase_currents[2]) / self.rotor_speed
        return torque

    def update_state(self, applied_voltages, dt):
        # Electrical Model
        back_emf = self._calculate_back_emf()

        for i in range(3):
            voltage_drop_due_to_resistance = self.phase_currents[i] * self.phase_resistance
            voltage_across_inductor = applied_voltages[i] - back_emf[i] - voltage_drop_due_to_resistance
            rate_of_change_of_current = voltage_across_inductor / self.phase_inductance
            self.phase_currents[i] += rate_of_change_of_current * dt

        # Mechanical Model
        electromagnetic_torque = self._calculate_electromagnetic_torque()
        load_torque = self.friction_coefficient * self.rotor_speed
        net_torque = electromagnetic_torque - load_torque
        angular_acceleration = net_torque / self.rotor_inertia
        
        self.rotor_speed += angular_acceleration * dt
        self.rotor_angle += self.rotor_speed * dt
        self.rotor_angle = math.fmod(self.rotor_angle, 2 * math.pi)

        # Update phase voltages
        self.phase_voltages = applied_voltages
