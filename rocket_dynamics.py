"""
6-DOF Rocket Dynamics Model
Implements equations of motion for rocket with thrust vector control
"""

import numpy as np
from atmosphere import Atmosphere, Wind
from aerodynamics import Aerodynamics
from config import rocket, disturbance

class RocketDynamics:
    """
    6 Degree of Freedom rocket dynamics
    State vector: [x, y, z, Vx, Vy, Vz, phi, theta, psi, p, q, r]
    - Position: (x,y,z) in NED frame (m)
    - Velocity: (Vx,Vy,Vz) in NED frame (m/s)
    - Attitude: (phi,theta,psi) - roll, pitch, yaw Euler angles (rad)
    - Angular velocity: (p,q,r) in body frame (rad/s)
    """
    
    def __init__(self, rocket_config, disturbance_config):
        """
        Initialize rocket dynamics
        
        Args:
            rocket_config: RocketConfig instance
            disturbance_config: DisturbanceConfig instance
        """
        self.config = rocket_config
        self.dist_config = disturbance_config
        
        # Initialize subsystems
        self.atmosphere = Atmosphere()
        self.aerodynamics = Aerodynamics(rocket_config)
        self.wind = Wind(
            wind_north=disturbance_config.wind_north,
            wind_east=disturbance_config.wind_east,
            wind_up=disturbance_config.wind_vertical,
            turbulence_intensity=disturbance_config.wind_turbulence_intensity
        )
        
        # State variables (will be initialized in reset())
        self.state = None
        self.time = 0.0
        
        # Motor state
        self.propellant_remaining = rocket_config.propellant_mass
        self.motor_burning = False
        
        # Apply uncertainties to parameters
        self._apply_uncertainties()
        
        # Initialize state
        self.reset()
    
    def _apply_uncertainties(self):
        """Apply parametric uncertainties from config"""
        # Mass uncertainty
        self.mass_factor = 1.0 + np.random.uniform(
            -self.dist_config.mass_uncertainty,
            self.dist_config.mass_uncertainty
        )
        
        # Thrust uncertainty
        self.thrust_factor = 1.0 + np.random.uniform(
            -self.dist_config.thrust_uncertainty,
            self.dist_config.thrust_uncertainty
        )
        
        # Aero uncertainty
        self.aero_factor = 1.0 + np.random.uniform(
            -self.dist_config.aero_uncertainty,
            self.dist_config.aero_uncertainty
        )
    
    def reset(self, initial_position=None, initial_attitude=None):
        """
        Reset rocket to initial conditions
        
        Args:
            initial_position: Initial position [N, E, D] (m), default [0,0,0]
            initial_attitude: Initial attitude [roll, pitch, yaw] (rad)
        """
        if initial_position is None:
            initial_position = np.zeros(3)
        
        if initial_attitude is None:
            # Default: vertical attitude (pitch from vertical)
            from config import sim
            elevation_rad = np.deg2rad(sim.initial_elevation)
            azimuth_rad = np.deg2rad(sim.initial_azimuth)
            initial_attitude = np.array([
                np.deg2rad(sim.initial_roll + self.dist_config.initial_roll_rate * 0.01),
                np.pi/2 - elevation_rad + np.deg2rad(self.dist_config.initial_pitch_error),
                azimuth_rad + np.deg2rad(self.dist_config.initial_yaw_error)
            ])
        
        # State: [N, E, D, Vn, Ve, Vd, phi, theta, psi, p, q, r]
        self.state = np.zeros(12)
        self.state[0:3] = initial_position  # Position
        self.state[6:9] = initial_attitude  # Attitude
        self.state[9] = np.deg2rad(self.dist_config.initial_roll_rate)  # Initial roll rate
        
        self.time = 0.0
        self.propellant_remaining = self.config.propellant_mass
        self.motor_burning = False
        
        # Reset wind turbulence
        self.wind.prev_turbulence = np.zeros(3)
    
    def ignite_motor(self):
        """Ignite the rocket motor"""
        self.motor_burning = True
    
    def get_mass(self):
        """Get current rocket mass"""
        current_mass = (self.config.dry_mass + self.propellant_remaining) * self.mass_factor
        return current_mass
    
    def get_inertia_matrix(self):
        """Get current moment of inertia matrix (diagonal)"""
        # Simplified: assume inertia scales linearly with propellant mass
        mass_fraction = self.propellant_remaining / self.config.propellant_mass
        
        # Inertia reduces as propellant burns
        I_xx = self.config.I_roll * (1 + 0.2 * mass_fraction)
        I_yy = self.config.I_pitch * (1 + 0.5 * mass_fraction)
        I_zz = self.config.I_yaw * (1 + 0.5 * mass_fraction)
        
        return np.diag([I_xx, I_yy, I_zz])
    
    def get_thrust(self):
        """
        Get current thrust magnitude
        
        Returns:
            Thrust force (N)
        """
        if self.motor_burning and self.propellant_remaining > 0:
            thrust = self.config.thrust_nominal * self.thrust_factor
        else:
            thrust = 0.0
            self.motor_burning = False
        
        return thrust
    
    def euler_to_dcm(self, phi, theta, psi):
        """
        Convert Euler angles to Direction Cosine Matrix (DCM)
        Rotation sequence: Z-Y-X (yaw-pitch-roll)
        DCM transforms from body frame to NED frame
        
        Args:
            phi: Roll angle (rad)
            theta: Pitch angle (rad)
            psi: Yaw angle (rad)
            
        Returns:
            3x3 DCM matrix
        """
        # Rotation matrices
        cos_phi, sin_phi = np.cos(phi), np.sin(phi)
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)
        cos_psi, sin_psi = np.cos(psi), np.sin(psi)
        
        # Combined DCM (body to NED)
        dcm = np.array([
            [cos_theta*cos_psi, 
             sin_phi*sin_theta*cos_psi - cos_phi*sin_psi,
             cos_phi*sin_theta*cos_psi + sin_phi*sin_psi],
            [cos_theta*sin_psi,
             sin_phi*sin_theta*sin_psi + cos_phi*cos_psi,
             cos_phi*sin_theta*sin_psi - sin_phi*cos_psi],
            [-sin_theta,
             sin_phi*cos_theta,
             cos_phi*cos_theta]
        ])
        
        return dcm
    
    def euler_rates_matrix(self, phi, theta):
        """
        Matrix to convert body angular rates to Euler angle rates
        [phi_dot, theta_dot, psi_dot] = E * [p, q, r]
        
        Args:
            phi: Roll angle (rad)
            theta: Pitch angle (rad)
            
        Returns:
            3x3 transformation matrix
        """
        cos_phi, sin_phi = np.cos(phi), np.sin(phi)
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)
        
        # Avoid singularity at theta = ±90 degrees
        if np.abs(cos_theta) < 0.01:
            cos_theta = 0.01 * np.sign(cos_theta)
        
        E = np.array([
            [1, sin_phi*np.tan(theta), cos_phi*np.tan(theta)],
            [0, cos_phi, -sin_phi],
            [0, sin_phi/cos_theta, cos_phi/cos_theta]
        ])
        
        return E
    
    def derivatives(self, state, control_deflections):
        """
        Calculate state derivatives (equations of motion)
        
        Args:
            state: Current state vector [12x1]
            control_deflections: [delta_pitch, delta_yaw] (radians)
            
        Returns:
            state_dot: Time derivative of state [12x1]
        """
        # Unpack state
        position = state[0:3]  # [N, E, D]
        velocity_ned = state[3:6]  # [Vn, Ve, Vd]
        attitude = state[6:9]  # [phi, theta, psi]
        omega_body = state[9:12]  # [p, q, r]
        
        phi, theta, psi = attitude
        
        # Get DCM (body to NED)
        dcm_bn = self.euler_to_dcm(phi, theta, psi)
        dcm_nb = dcm_bn.T  # NED to body
        
        # Convert NED velocity to body frame
        velocity_body = dcm_nb @ velocity_ned
        
        # Get atmospheric properties
        altitude = -position[2]  # Altitude (D is negative altitude in NED)
        V_total = np.linalg.norm(velocity_ned)
        rho = self.atmosphere.density(altitude)
        mach = self.atmosphere.mach_number(altitude, V_total)
        q_dyn = self.atmosphere.dynamic_pressure(altitude, V_total)
        
        # Get wind velocity and compute relative velocity
        wind_ned = self.wind.get_wind_velocity(altitude, self.time, 0.01)
        velocity_relative_ned = velocity_ned - wind_ned
        velocity_relative_body = dcm_nb @ velocity_relative_ned
        V_relative = np.linalg.norm(velocity_relative_ned)
        
        # Aerodynamic forces and moments (in body frame)
        delta_pitch, delta_yaw = control_deflections
        F_aero_body, M_aero_body = self.aerodynamics.calculate_forces_moments(
            velocity_relative_body, omega_body, altitude, mach,
            delta_pitch, delta_yaw, q_dyn
        )
        
        # Apply aerodynamic uncertainty
        F_aero_body *= self.aero_factor
        M_aero_body *= self.aero_factor
        
        # Thrust force (along body X-axis)
        thrust = self.get_thrust()
        
        # Add nozzle misalignment
        nozzle_misalign = np.deg2rad(self.dist_config.nozzle_misalignment)
        F_thrust_body = thrust * np.array([
            np.cos(nozzle_misalign),
            0,
            np.sin(nozzle_misalign)
        ])
        
        # Thrust moment due to TVC deflection
        # Moment arm is distance from CoM to nozzle pivot
        moment_arm = self.config.nozzle_length
        M_thrust_body = np.array([
            0,
            -thrust * moment_arm * delta_pitch,  # Pitch moment
            thrust * moment_arm * delta_yaw      # Yaw moment
        ])
        
        # Total forces and moments in body frame
        F_body = F_thrust_body + F_aero_body
        M_body = M_thrust_body + M_aero_body
        
        # Transform forces to NED frame
        F_ned = dcm_bn @ F_body
        
        # Add gravity (NED frame)
        mass = self.get_mass()
        g_ned = np.array([0, 0, 9.81 * mass])  # Weight down
        F_ned += g_ned
        
        # Linear acceleration (NED frame)
        accel_ned = F_ned / mass
        
        # Angular acceleration (body frame)
        I = self.get_inertia_matrix()
        I_inv = np.linalg.inv(I)
        
        # Euler's equation: I*omega_dot + omega × (I*omega) = M
        omega_dot_body = I_inv @ (M_body - np.cross(omega_body, I @ omega_body))
        
        # Euler angle rates
        E = self.euler_rates_matrix(phi, theta)
        attitude_dot = E @ omega_body
        
        # Position rate
        position_dot = velocity_ned
        
        # Assemble state derivative
        state_dot = np.zeros(12)
        state_dot[0:3] = position_dot
        state_dot[3:6] = accel_ned
        state_dot[6:9] = attitude_dot
        state_dot[9:12] = omega_dot_body
        
        return state_dot
    
    def update(self, dt, control_deflections):
        """
        Update rocket state using RK4 integration
        
        Args:
            dt: Time step (seconds)
            control_deflections: [delta_pitch, delta_yaw] (radians)
        """
        # RK4 integration
        k1 = self.derivatives(self.state, control_deflections)
        k2 = self.derivatives(self.state + 0.5*dt*k1, control_deflections)
        k3 = self.derivatives(self.state + 0.5*dt*k2, control_deflections)
        k4 = self.derivatives(self.state + dt*k3, control_deflections)
        
        self.state += (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
        
        # Update propellant mass
        if self.motor_burning and self.propellant_remaining > 0:
            mdot = self.config.propellant_mass / self.config.burn_time
            self.propellant_remaining -= mdot * dt
            
            if self.propellant_remaining <= 0:
                self.propellant_remaining = 0
                self.motor_burning = False
        
        # Update time
        self.time += dt
        
        # Stop if crashed
        if self.state[2] > 0:  # Below ground level
            self.state[2] = 0
            self.state[3:6] = 0  # Zero velocity
    
    def get_state_dict(self):
        """Get current state as dictionary"""
        altitude = -self.state[2]
        velocity_ned = self.state[3:6]
        V_total = np.linalg.norm(velocity_ned)
        
        return {
            'time': self.time,
            'position': self.state[0:3].copy(),
            'velocity': velocity_ned.copy(),
            'attitude': self.state[6:9].copy(),
            'angular_velocity': self.state[9:12].copy(),
            'altitude': altitude,
            'velocity_magnitude': V_total,
            'mass': self.get_mass(),
            'thrust': self.get_thrust(),
            'motor_burning': self.motor_burning
        }


# Test function
if __name__ == "__main__":
    from config import sim
    
    print("Rocket Dynamics Test:")
    print("-" * 50)
    
    rocket_sim = RocketDynamics(rocket, disturbance)
    rocket_sim.ignite_motor()
    
    dt = 0.01
    
    # Simulate for 10 seconds
    for i in range(int(10/dt)):
        # No control deflections
        control = np.array([0.0, 0.0])
        rocket_sim.update(dt, control)
        
        if i % 100 == 0:  # Print every second
            state = rocket_sim.get_state_dict()
            print(f"\nTime: {state['time']:.1f}s")
            print(f"  Altitude: {state['altitude']:.1f} m")
            print(f"  Velocity: {state['velocity_magnitude']:.1f} m/s")
            print(f"  Attitude (deg): {np.rad2deg(state['attitude'])}")
            print(f"  Thrust: {state['thrust']:.0f} N")