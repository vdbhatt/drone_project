import math

class SixStepCommutator:
    def __init__(self, dc_bus_voltage):
        self.dc_bus_voltage = dc_bus_voltage

    def get_phase_voltages(self, rotor_angle_rad):
        """
        Determines the phase voltages based on the rotor angle (six-step commutation).
        Assumes a star-connected motor.
        Voltages are applied with respect to a common ground/neutral.
        One phase is high (dc_bus_voltage), one is low (0V), one is open (0V).
        """
        v_high = self.dc_bus_voltage
        v_low = 0  # Representing connection to GND or negative DC bus rail
        
        # Normalize angle to be within 0 to 2*pi
        angle = math.fmod(rotor_angle_rad, 2 * math.pi)
        if angle < 0:
            angle += 2 * math.pi

        s1_max = math.pi / 3  # 60 degrees
        s2_max = 2 * math.pi / 3 # 120 degrees
        s3_max = math.pi  # 180 degrees
        s4_max = 4 * math.pi / 3 # 240 degrees
        s5_max = 5 * math.pi / 3 # 300 degrees
        # s6_max is 2 * math.pi

        va, vb, vc = 0, 0, 0 # Default to all off

        if 0 <= angle < s1_max:
            # Sector 1: A+, B- (C open) -> Va=Vdc, Vb=0, Vc=0 (Incorrect mapping previously)
            # Correct: Va = High, Vb = Low, Vc = Open
            va = v_high
            vb = v_low
            vc = 0 # Open phase
        elif s1_max <= angle < s2_max:
            # Sector 2: A+, C- (B open) -> Va=Vdc, Vc=0, Vb=0
            # Correct: Va = High, Vc = Low, Vb = Open
            va = v_high
            vc = v_low
            vb = 0 # Open phase
        elif s2_max <= angle < s3_max:
            # Sector 3: B+, C- (A open) -> Vb=Vdc, Vc=0, Va=0
            # Correct: Vb = High, Vc = Low, Va = Open
            vb = v_high
            vc = v_low
            va = 0 # Open phase
        elif s3_max <= angle < s4_max:
            # Sector 4: B+, A- (C open) -> Vb=Vdc, Va=0, Vc=0
            # Correct: Vb = High, Va = Low, Vc = Open
            vb = v_high
            va = v_low
            vc = 0 # Open phase
        elif s4_max <= angle < s5_max:
            # Sector 5: C+, A- (B open) -> Vc=Vdc, Va=0, Vb=0
            # Correct: Vc = High, Va = Low, Vb = Open
            vc = v_high
            va = v_low
            vb = 0 # Open phase
        elif s5_max <= angle < (2 * math.pi):
            # Sector 6: C+, B- (A open) -> Vc=Vdc, Vb=0, Va=0
            # Correct: Vc = High, Vb = Low, Va = Open
            vc = v_high
            vb = v_low
            va = 0 # Open phase
        
        return [va, vb, vc]


class FOCController:
    def __init__(self, k_p_d, k_i_d, k_p_q, k_i_q, voltage_limit_d, voltage_limit_q, target_id=0):
        # PI controller gains for d-axis
        self.k_p_d = k_p_d
        self.k_i_d = k_i_d
        # PI controller gains for q-axis
        self.k_p_q = k_p_q
        self.k_i_q = k_i_q
        # Voltage limits for d and q axes
        self.voltage_limit_d = voltage_limit_d
        self.voltage_limit_q = voltage_limit_q
        # Target d-axis current
        self.target_id = target_id
        # Error accumulators for integral terms
        self.id_error_integral = 0.0
        self.iq_error_integral = 0.0

    def clarke_transform(self, Ia, Ib, Ic):
        """Converts three-phase currents (Ia, Ib, Ic) to two-phase stationary (Ialpha, Ibeta)."""
        Ialpha = (2/3) * (Ia - 0.5 * Ib - 0.5 * Ic)
        Ibeta = (2/3) * ((math.sqrt(3)/2) * Ib - (math.sqrt(3)/2) * Ic)
        return (Ialpha, Ibeta)

    def park_transform(self, Ialpha, Ibeta, angle_rad):
        """Converts two-phase stationary currents (Ialpha, Ibeta) to two-phase rotating (Id, Iq)."""
        Id = Ialpha * math.cos(angle_rad) + Ibeta * math.sin(angle_rad)
        Iq = -Ialpha * math.sin(angle_rad) + Ibeta * math.cos(angle_rad)
        return (Id, Iq)

    def inverse_park_transform(self, Vd, Vq, angle_rad):
        """Converts two-phase rotating voltages (Vd, Vq) to two-phase stationary (Valpha, Vbeta)."""
        Valpha = Vd * math.cos(angle_rad) - Vq * math.sin(angle_rad)
        Vbeta = Vd * math.sin(angle_rad) + Vq * math.cos(angle_rad)
        return (Valpha, Vbeta)

    def inverse_clarke_transform(self, Valpha, Vbeta):
        """Converts two-phase stationary voltages (Valpha, Vbeta) to three-phase (Va, Vb, Vc)."""
        Va = Valpha
        Vb = -0.5 * Valpha + (math.sqrt(3)/2) * Vbeta
        Vc = -0.5 * Valpha - (math.sqrt(3)/2) * Vbeta
        return (Va, Vb, Vc)

    def control_step(self, current_Ia, current_Ib, current_Ic, rotor_angle_rad, target_iq, dt):
        """
        Performs one step of the FOC control logic.
        Takes current phase currents, rotor angle, target q-axis current, and dt.
        Returns target phase voltages (Va, Vb, Vc).
        """
        # a. Transformations
        Ialpha, Ibeta = self.clarke_transform(current_Ia, current_Ib, current_Ic)
        Id_measured, Iq_measured = self.park_transform(Ialpha, Ibeta, rotor_angle_rad)

        # b. Id Controller
        id_error = self.target_id - Id_measured
        
        self.id_error_integral += id_error * dt
        # Clamp integral to prevent excessive windup if k_i_d is not zero
        if self.k_i_d != 0:
            max_integral_val_d = self.voltage_limit_d / abs(self.k_i_d) if self.k_i_d != 0 else float('inf')
            self.id_error_integral = max(min(self.id_error_integral, max_integral_val_d), -max_integral_val_d)

        id_p_term = self.k_p_d * id_error
        id_i_term = self.k_i_d * self.id_error_integral
        Vd_output = id_p_term + id_i_term
        Vd_output = max(min(Vd_output, self.voltage_limit_d), -self.voltage_limit_d)

        # c. Iq Controller
        iq_error = target_iq - Iq_measured
        
        self.iq_error_integral += iq_error * dt
        # Clamp integral to prevent excessive windup if k_i_q is not zero
        if self.k_i_q != 0:
            max_integral_val_q = self.voltage_limit_q / abs(self.k_i_q) if self.k_i_q != 0 else float('inf')
            self.iq_error_integral = max(min(self.iq_error_integral, max_integral_val_q), -max_integral_val_q)
            
        iq_p_term = self.k_p_q * iq_error
        iq_i_term = self.k_i_q * self.iq_error_integral
        Vq_output = iq_p_term + iq_i_term
        Vq_output = max(min(Vq_output, self.voltage_limit_q), -self.voltage_limit_q)

        # Store saturated voltages for logging
        self.Vd_saturated = Vd_output
        self.Vq_saturated = Vq_output

        # d. Inverse Transformations
        Valpha, Vbeta = self.inverse_park_transform(Vd_output, Vq_output, rotor_angle_rad)
        Va, Vb, Vc = self.inverse_clarke_transform(Valpha, Vbeta)
        
        return (Va, Vb, Vc)
