"""
Reference trajectory generation for guidance control
"""

import numpy as np

class ReferenceTrajectory:
    """
    Generates reference trajectory for the rocket to follow
    """
    
    def __init__(self, launch_elevation, launch_azimuth, target_apogee, 
                 target_range, burn_time):
        """
        Initialize trajectory generator
        
        Args:
            launch_elevation: Launch elevation angle (degrees)
            launch_azimuth: Launch azimuth angle (degrees from North)
            target_apogee: Target apogee altitude (m)
            target_range: Target downrange distance (m)
            burn_time: Motor burn time (seconds)
        """
        self.launch_elevation_deg = launch_elevation
        self.launch_azimuth_deg = launch_azimuth
        self.launch_elevation = np.deg2rad(launch_elevation)
        self.launch_azimuth = np.deg2rad(launch_azimuth)
        self.target_apogee = target_apogee
        self.target_range = target_range
        self.burn_time = burn_time
        
        # Calculate trajectory parameters
        self._calculate_trajectory_params()
    
    def _calculate_trajectory_params(self):
        """
        Calculate trajectory parameters based on targets
        This is a simplified ballistic trajectory estimation
        """
        # Simplified trajectory model
        # Assumes parabolic path after burnout
        
        # Average velocity during boost (rough estimate)
        g = 9.81
        
        # Time to apogee (rough estimate)
        # Using kinematic equations: h = v0*t - 0.5*g*t²
        # At apogee: v = 0, so v0 = g*t_apogee
        self.estimated_apogee_time = 20.0  # seconds (typical for 5km apogee)
        
        # Required velocity at burnout to reach target apogee
        # Using energy: 0.5*m*v² = m*g*h
        self.burnout_velocity = np.sqrt(2 * g * self.target_apogee)
        
    def get_reference_position(self, time):
        """
        Get reference position at given time
        
        Args:
            time: Current time (seconds)
            
        Returns:
            position: Reference position [north, east, altitude] (m)
        """
        if time < self.burn_time:
            # During boost phase - follow programmed attitude
            # Simplified: linear trajectory in launch direction
            
            # Average acceleration during boost
            avg_accel = self.burnout_velocity / self.burn_time
            
            # Distance traveled
            distance = 0.5 * avg_accel * time**2
            
            # Decompose into components
            altitude = distance * np.sin(self.launch_elevation)
            horizontal_distance = distance * np.cos(self.launch_elevation)
            
            north = horizontal_distance * np.cos(self.launch_azimuth)
            east = horizontal_distance * np.sin(self.launch_azimuth)
            
        else:
            # After burnout - ballistic trajectory
            time_since_burnout = time - self.burn_time
            
            # Velocity at burnout
            v_burnout = self.burnout_velocity
            
            # Vertical motion (with gravity)
            altitude_burnout = 0.5 * (v_burnout * np.sin(self.launch_elevation)) * self.burn_time
            v_vertical = v_burnout * np.sin(self.launch_elevation)
            
            altitude = altitude_burnout + v_vertical * time_since_burnout - \
                      0.5 * 9.81 * time_since_burnout**2
            
            # Horizontal motion (constant velocity, ignoring drag)
            v_horizontal = v_burnout * np.cos(self.launch_elevation)
            horizontal_distance = v_horizontal * time
            
            north = horizontal_distance * np.cos(self.launch_azimuth)
            east = horizontal_distance * np.sin(self.launch_azimuth)
            
            # Don't go below ground
            altitude = max(altitude, 0)
        
        return np.array([north, east, altitude])
    
    def get_reference_attitude(self, time):
        """
        Get reference attitude at given time (open-loop trajectory)
        
        Args:
            time: Current time (seconds)
            
        Returns:
            attitude: Reference attitude [roll, pitch, yaw] (radians)
        """
        # Simplified attitude profile
        
        if time < 1.0:
            # Initial vertical climb
            pitch = np.deg2rad(90 - self.launch_elevation_deg)
            yaw = self.launch_azimuth
            
        elif time < self.burn_time:
            # Pitch over during boost
            # Linear pitch program from initial to final pitch
            progress = (time - 1.0) / (self.burn_time - 1.0)
            
            initial_pitch = np.deg2rad(90 - self.launch_elevation_deg)
            final_pitch = np.deg2rad(45)  # Pitch to 45 degrees by end of boost
            
            pitch = initial_pitch + progress * (final_pitch - initial_pitch)
            yaw = self.launch_azimuth
            
        else:
            # Maintain attitude after burnout
            pitch = np.deg2rad(45)
            yaw = self.launch_azimuth
        
        # No roll command
        roll = 0.0
        
        return np.array([roll, pitch, yaw])
    
    def get_reference_velocity(self, time):
        """
        Get reference velocity at given time
        
        Args:
            time: Current time (seconds)
            
        Returns:
            velocity: Reference velocity [Vn, Ve, Vd] (m/s)
        """
        if time < self.burn_time:
            # During boost
            avg_accel = self.burnout_velocity / self.burn_time
            velocity_magnitude = avg_accel * time
        else:
            # After burnout
            time_since_burnout = time - self.burn_time
            v_vertical_burnout = self.burnout_velocity * np.sin(self.launch_elevation)
            v_vertical = v_vertical_burnout - 9.81 * time_since_burnout
            v_horizontal = self.burnout_velocity * np.cos(self.launch_elevation)
            
            Vn = v_horizontal * np.cos(self.launch_azimuth)
            Ve = v_horizontal * np.sin(self.launch_azimuth)
            Vd = -v_vertical  # Down is positive in NED frame
            
            return np.array([Vn, Ve, Vd])
        
        # During boost
        Vn = velocity_magnitude * np.cos(self.launch_elevation) * np.cos(self.launch_azimuth)
        Ve = velocity_magnitude * np.cos(self.launch_elevation) * np.sin(self.launch_azimuth)
        Vd = -velocity_magnitude * np.sin(self.launch_elevation)
        
        return np.array([Vn, Ve, Vd])


class GravityTurnTrajectory:
    """
    Gravity turn trajectory (more realistic for rockets)
    Rocket maintains attitude aligned with velocity vector
    """
    
    def __init__(self, initial_pitch_angle, pitch_rate):
        """
        Initialize gravity turn trajectory
        
        Args:
            initial_pitch_angle: Initial pitch angle from vertical (degrees)
            pitch_rate: Rate of pitch change (deg/s)
        """
        self.initial_pitch = np.deg2rad(initial_pitch_angle)
        self.pitch_rate = np.deg2rad(pitch_rate)
    
    def get_pitch_angle(self, time):
        """
        Get pitch angle for gravity turn
        
        Args:
            time: Current time (seconds)
            
        Returns:
            Pitch angle from vertical (radians)
        """
        if time < 2.0:
            # Vertical ascent for tower clearance
            return 0.0
        else:
            # Gradual pitch over
            time_since_start = time - 2.0
            pitch = self.initial_pitch + self.pitch_rate * time_since_start
            
            # Limit to 90 degrees
            pitch = min(pitch, np.pi/2)
            
            return pitch


# Test function
if __name__ == "__main__":
    print("Trajectory Generator Test:")
    print("-" * 50)
    
    traj = ReferenceTrajectory(
        launch_elevation=85,
        launch_azimuth=90,
        target_apogee=5000,
        target_range=2000,
        burn_time=5.0
    )
    
    # Test at different times
    times = [0, 2.5, 5, 10, 20]
    
    for t in times:
        pos = traj.get_reference_position(t)
        att = traj.get_reference_attitude(t)
        vel = traj.get_reference_velocity(t)
        
        print(f"\nTime: {t}s")
        print(f"  Position (N,E,Alt): [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}] m")
        print(f"  Attitude (R,P,Y): [{np.rad2deg(att[0]):.1f}, {np.rad2deg(att[1]):.1f}, {np.rad2deg(att[2]):.1f}] deg")
        print(f"  Velocity (N,E,D): [{vel[0]:.1f}, {vel[1]:.1f}, {vel[2]:.1f}] m/s")