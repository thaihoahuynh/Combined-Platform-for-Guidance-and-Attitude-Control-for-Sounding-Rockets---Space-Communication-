"""
Control system implementation
Includes PID controllers for attitude and guidance
"""

import numpy as np

class PIDController:
    """PID controller with anti-windup and filtering"""
    
    def __init__(self, Kp, Ki, Kd, max_integral=5.0, filter_N=10.0):
        """Initialize PID controller"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_integral = max_integral
        self.filter_N = filter_N
        
        # State variables
        self.integral = 0.0
        self.previous_error = 0.0
        self.filtered_derivative = 0.0
        
        # For debugging/logging
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.filtered_derivative = 0.0
    
    def compute(self, error, dt, enable_integral=True):
        """Compute PID control output"""
        # Proportional term
        p_term = self.Kp * error
        
        # Integral term with anti-windup
        if enable_integral:
            self.integral += error * dt
            self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        
        i_term = self.Ki * self.integral
        
        # Derivative term with filtering
        derivative = (error - self.previous_error) / dt
        alpha = dt * self.filter_N / (1 + dt * self.filter_N)
        self.filtered_derivative = (1 - alpha) * self.filtered_derivative + alpha * derivative
        d_term = self.Kd * self.filtered_derivative
        
        self.previous_error = error
        
        # Store for debugging
        self.last_p_term = p_term
        self.last_i_term = i_term
        self.last_d_term = d_term
        
        output = p_term + i_term + d_term
        return output
    
    def update_gains(self, Kp, Ki, Kd):
        """Update PID gains (for gain scheduling)"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd


class AttitudeController:
    """Three-axis attitude controller using PID"""
    
    def __init__(self, control_config):
        """Initialize attitude controller"""
        self.roll_pid = PIDController(
            control_config.roll_Kp, 
            control_config.roll_Ki, 
            control_config.roll_Kd,
            control_config.max_integral_error
        )
        
        self.pitch_pid = PIDController(
            control_config.pitch_Kp, 
            control_config.pitch_Ki, 
            control_config.pitch_Kd,
            control_config.max_integral_error
        )
        
        self.yaw_pid = PIDController(
            control_config.yaw_Kp, 
            control_config.yaw_Ki, 
            control_config.yaw_Kd,
            control_config.max_integral_error
        )
        
        self.config = control_config
    
    def reset(self):
        """Reset all controllers"""
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
    
    def compute(self, attitude_current, attitude_desired, dt, phase='coast'):
        """Compute attitude control commands"""
        # Calculate errors (wrap angles to [-pi, pi])
        error_roll = self._wrap_angle(attitude_desired[0] - attitude_current[0])
        error_pitch = self._wrap_angle(attitude_desired[1] - attitude_current[1])
        error_yaw = self._wrap_angle(attitude_desired[2] - attitude_current[2])
        
        # Apply gain scheduling if enabled
        if self.config.use_gain_scheduling:
            if phase == 'boost':
                gain_mult = self.config.gain_boost_phase
            else:
                gain_mult = self.config.gain_coast_phase
            
            self.roll_pid.Kp *= gain_mult
            self.pitch_pid.Kp *= gain_mult
            self.yaw_pid.Kp *= gain_mult
        else:
            gain_mult = 1.0
        
        # Compute control for each axis
        delta_roll = self.roll_pid.compute(error_roll, dt)
        delta_pitch = self.pitch_pid.compute(error_pitch, dt)
        delta_yaw = self.yaw_pid.compute(error_yaw, dt)
        
        # Restore original gains
        if self.config.use_gain_scheduling and gain_mult != 1.0:
            self.roll_pid.Kp /= gain_mult
            self.pitch_pid.Kp /= gain_mult
            self.yaw_pid.Kp /= gain_mult
        
        return np.array([delta_roll, delta_pitch, delta_yaw])
    
    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-pi, pi]"""
        return np.arctan2(np.sin(angle), np.cos(angle))


class GuidanceController:
    """Guidance controller for trajectory tracking"""
    
    def __init__(self, control_config):
        """Initialize guidance controller"""
        self.position_pid_north = PIDController(
            control_config.guidance_Kp,
            control_config.guidance_Ki,
            control_config.guidance_Kd
        )
        
        self.position_pid_east = PIDController(
            control_config.guidance_Kp,
            control_config.guidance_Ki,
            control_config.guidance_Kd
        )
        
        self.config = control_config
    
    def reset(self):
        """Reset guidance controller"""
        self.position_pid_north.reset()
        self.position_pid_east.reset()
    
    def compute(self, position_current, position_target, velocity_current, dt):
        """Compute desired attitude from position error"""
        # Calculate position errors
        error_north = position_target[0] - position_current[0]
        error_east = position_target[1] - position_current[1]
        
        # Compute desired accelerations using PID
        accel_north = self.position_pid_north.compute(error_north, dt)
        accel_east = self.position_pid_east.compute(error_east, dt)
        
        # Limit acceleration commands
        max_accel = 20.0  # m/s²
        accel_north = np.clip(accel_north, -max_accel, max_accel)
        accel_east = np.clip(accel_east, -max_accel, max_accel)
        
        # Convert to attitude commands
        velocity_magnitude = np.linalg.norm(velocity_current[:2])
        
        if velocity_magnitude > 10.0:
            g = 9.81
            pitch_command = -accel_north / g
            yaw_command = accel_east / g
            
            max_angle = np.deg2rad(15)
            pitch_command = np.clip(pitch_command, -max_angle, max_angle)
            yaw_command = np.clip(yaw_command, -max_angle, max_angle)
        else:
            pitch_command = 0.0
            yaw_command = 0.0
        
        roll_command = 0.0
        return np.array([roll_command, pitch_command, yaw_command])


class LowPassFilter:
    """Second-order low-pass filter for control signal smoothing"""
    
    def __init__(self, cutoff_freq, damping=0.707):
        """Initialize low-pass filter"""
        self.omega_n = 2 * np.pi * cutoff_freq
        self.zeta = damping
        self.state = np.zeros(2)
    
    def filter(self, input_signal, dt):
        """Apply filter to input signal"""
        A = np.array([
            [0, 1],
            [-self.omega_n**2, -2*self.zeta*self.omega_n]
        ])
        B = np.array([0, self.omega_n**2])
        
        state_dot = A @ self.state + B * input_signal
        self.state += state_dot * dt
        
        output = self.state[0]
        return output
    
    def reset(self):
        """Reset filter state"""
        self.state = np.zeros(2)