"""
Aerodynamic model for sounding rocket
Calculates forces and moments based on angle of attack and control deflections
"""

import numpy as np

class Aerodynamics:
    """Aerodynamic force and moment calculator"""
    
    def __init__(self, rocket_config):
        """Initialize aerodynamics model"""
        self.config = rocket_config
        self.S = rocket_config.reference_area
        self.d = rocket_config.diameter
        self.L = rocket_config.length
        
    def angle_of_attack(self, velocity_body):
        """Calculate angle of attack and sideslip angle"""
        Vx, Vy, Vz = velocity_body
        V_total = np.linalg.norm(velocity_body)
        
        if V_total > 1.0:
            alpha = np.arctan2(Vz, Vx)  # Pitch plane
            beta = np.arcsin(np.clip(Vy / V_total, -1, 1))  # Yaw plane
        else:
            alpha = 0.0
            beta = 0.0
        
        return alpha, beta, V_total
    
    def drag_coefficient(self, alpha, mach):
        """Calculate drag coefficient"""
        Cd0 = self.config.Cd0
        
        if mach > 0.8:
            # Transonic drag rise
            Cd0 *= (1 + 0.2 * (mach - 0.8)**2)
        
        # Induced drag due to angle of attack
        Cd_induced = self.config.Cd_alpha2 * alpha**2
        Cd = Cd0 + Cd_induced
        return Cd
    
    def lift_coefficient(self, alpha, beta, mach):
        """Calculate lift coefficients"""
        Cl_alpha = self.config.Cl_alpha
        
        # Compressibility correction (Prandtl-Glauert)
        if 0 < mach < 0.8:
            beta_comp = np.sqrt(1 - mach**2)
            Cl_alpha /= beta_comp
        elif mach >= 0.8:
            Cl_alpha *= 0.8
        
        Cl_pitch = Cl_alpha * alpha
        Cl_yaw = Cl_alpha * beta
        return Cl_pitch, Cl_yaw
    
    def moment_coefficient(self, alpha, beta, mach):
        """Calculate pitching and yawing moment coefficients"""
        Cm_alpha = self.config.Cm_alpha
        Cm_pitch = Cm_alpha * alpha
        Cm_yaw = Cm_alpha * beta
        return Cm_pitch, Cm_yaw
    
    def control_forces_moments(self, delta_pitch, delta_yaw, dynamic_pressure):
        """Calculate forces and moments due to control deflections"""
        Cl_delta = self.config.Cl_delta
        Cm_delta = self.config.Cm_delta
        
        # Forces due to control (in body frame)
        Fz_control = -dynamic_pressure * self.S * Cl_delta * delta_pitch
        Fy_control = -dynamic_pressure * self.S * Cl_delta * delta_yaw
        
        # Moments due to control
        L_ref = self.config.nozzle_length
        My_control = dynamic_pressure * self.S * self.d * Cm_delta * delta_pitch
        Mz_control = dynamic_pressure * self.S * self.d * Cm_delta * delta_yaw
        
        return np.array([0, Fy_control, Fz_control]), \
               np.array([0, My_control, Mz_control])
    
    def calculate_forces_moments(self, velocity_body, angular_velocity_body, 
                                 altitude, mach, delta_pitch, delta_yaw, 
                                 dynamic_pressure):
        """Calculate total aerodynamic forces and moments"""
        alpha, beta, V_total = self.angle_of_attack(velocity_body)
        
        if V_total < 1.0:
            return np.zeros(3), np.zeros(3)
        
        # Get aerodynamic coefficients
        Cd = self.drag_coefficient(alpha, mach)
        Cl_pitch, Cl_yaw = self.lift_coefficient(alpha, beta, mach)
        Cm_pitch, Cm_yaw = self.moment_coefficient(alpha, beta, mach)
        
        # Calculate forces in body frame
        D = dynamic_pressure * self.S * Cd
        L_pitch = dynamic_pressure * self.S * Cl_pitch
        L_yaw = dynamic_pressure * self.S * Cl_yaw
        
        Fx = -D
        Fy = L_yaw
        Fz = L_pitch
        F_aero_body = np.array([Fx, Fy, Fz])
        
        # Calculate moments
        My = dynamic_pressure * self.S * self.d * Cm_pitch
        Mz = dynamic_pressure * self.S * self.d * Cm_yaw
        
        # Damping moments
        p, q, r = angular_velocity_body
        Clp = -0.5
        Cmq = -10.0
        Cnr = -10.0
        
        Mx_damp = dynamic_pressure * self.S * self.d * Clp * (p * self.d / (2 * V_total))
        My_damp = dynamic_pressure * self.S * self.d * Cmq * (q * self.d / (2 * V_total))
        Mz_damp = dynamic_pressure * self.S * self.d * Cnr * (r * self.d / (2 * V_total))
        
        M_aero_body = np.array([Mx_damp, My + My_damp, Mz + Mz_damp])
        
        # Add control forces and moments
        F_control, M_control = self.control_forces_moments(delta_pitch, delta_yaw, 
                                                           dynamic_pressure)
        
        F_aero_body += F_control
        M_aero_body += M_control
        
        return F_aero_body, M_aero_body
