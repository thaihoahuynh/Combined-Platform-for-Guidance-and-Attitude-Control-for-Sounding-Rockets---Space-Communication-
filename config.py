"""
Configuration file for sounding rocket simulation
Contains all physical parameters and simulation settings
"""

import numpy as np

class RocketConfig:
    """Physical parameters of the sounding rocket"""
    
    # Mass properties
    dry_mass = 50.0  # kg (without propellant)
    propellant_mass = 30.0  # kg
    total_mass = dry_mass + propellant_mass  # kg
    
    # Geometry
    length = 3.0  # m
    diameter = 0.15  # m
    reference_area = np.pi * (diameter/2)**2  # m^2
    
    # Moments of inertia (kg·m²)
    I_roll = 2.0  # Roll axis (longitudinal)
    I_pitch = 20.0  # Pitch axis (lateral)
    I_yaw = 20.0  # Yaw axis (lateral)
    
    # Center of mass (from nose)
    center_of_mass = 1.8  # m
    
    # Center of pressure (from nose) - changes with speed
    center_of_pressure_subsonic = 2.2  # m
    center_of_pressure_supersonic = 2.0  # m
    
    # Propulsion
    thrust_nominal = 2000.0  # N
    burn_time = 5.0  # seconds
    specific_impulse = 200.0  # seconds
    nozzle_length = 0.4  # m (distance from CoM to nozzle pivot)
    
    # Control system limits
    max_deflection = 10.0  # degrees (nozzle or canard)
    max_deflection_rate = 20.0  # deg/s
    actuator_bandwidth = 12.0  # Hz
    actuator_damping = 0.7
    
    # Aerodynamic coefficients (simplified)
    Cd0 = 0.5  # Zero-lift drag coefficient
    Cl_alpha = 4.5  # Lift curve slope (per radian)
    Cm_alpha = -2.0  # Pitching moment curve slope (per radian)
    Cd_alpha2 = 0.3  # Induced drag factor
    
    # Control effectiveness
    Cl_delta = 0.8  # Lift due to control deflection (per radian)
    Cm_delta = -1.5  # Moment due to control deflection (per radian)


class SimulationConfig:
    """Simulation parameters"""
    
    dt = 0.01  # Time step (seconds) - 100 Hz
    duration = 60.0  # Total simulation time (seconds)
    
    # Launch conditions
    launch_altitude = 0.0  # m (sea level)
    launch_latitude = -2.31  # degrees (Alcântara, Brazil)
    launch_longitude = -44.37  # degrees
    
    initial_elevation = 85.0  # degrees (launch angle from horizontal)
    initial_azimuth = 90.0  # degrees (East)
    initial_roll = 0.0  # degrees
    
    # Control activation times
    control_start_time = 0.5  # seconds (after launch rail departure)
    guidance_start_time = 5.0  # seconds (after motor burnout)
    guidance_end_time = 15.0  # seconds
    control_end_time = 30.0  # seconds (before apogee)
    
    # Target trajectory
    target_apogee = 5000.0  # m
    target_range = 2000.0  # m (downrange distance)


class ControlConfig:
    """Controller parameters"""
    
    # Attitude PID gains (Roll)
    roll_Kp = 1.5
    roll_Ki = 0.5
    roll_Kd = 0.3
    
    # Attitude PID gains (Pitch)
    pitch_Kp = 1.8
    pitch_Ki = 0.6
    pitch_Kd = 0.4
    
    # Attitude PID gains (Yaw)
    yaw_Kp = 1.8
    yaw_Ki = 0.6
    yaw_Kd = 0.4
    
    # Guidance PID gains
    guidance_Kp = 0.3
    guidance_Ki = 0.1
    guidance_Kd = 0.5
    
    # Low-pass filter for control
    filter_cutoff_freq = 8.0  # Hz
    filter_damping = 0.5
    
    # Anti-windup limits
    max_integral_error = 5.0  # degrees
    
    # Gain scheduling parameters
    use_gain_scheduling = True
    
    # Phase-based gain multipliers
    gain_boost_phase = 1.2  # During motor burn
    gain_coast_phase = 0.8  # After burnout


class DisturbanceConfig:
    """Environmental disturbances and uncertainties"""
    
    # Wind model
    enable_wind = True
    wind_north = 5.0  # m/s
    wind_east = 3.0  # m/s
    wind_vertical = 0.0  # m/s
    wind_turbulence_intensity = 0.1  # fraction of mean wind
    
    # Thrust variations
    thrust_uncertainty = 0.0  # ±10% (0.1)
    
    # Mass uncertainty
    mass_uncertainty = 0.0  # ±5% (0.05)
    
    # Aerodynamic uncertainty
    aero_uncertainty = 0.0  # ±15% (0.15)
    
    # Initial attitude errors
    initial_pitch_error = 0.0  # degrees
    initial_yaw_error = 0.0  # degrees
    initial_roll_rate = 0.0  # deg/s
    
    # Mechanical asymmetries
    nozzle_misalignment = 0.0  # degrees
    fin_misalignment = 0.0  # degrees
    center_of_mass_offset = 0.0  # m


class LoggingConfig:
    """Data logging and plotting configuration"""
    
    log_interval = 1  # Log every N simulation steps
    
    # Plot settings
    plot_3d_trajectory = True
    plot_altitude = True
    plot_attitudes = True
    plot_control_deflections = True
    plot_velocities = True
    plot_angular_rates = True
    plot_position_errors = True
    
    # Save options
    save_data_to_csv = True
    save_plots = True
    output_directory = "simulation_results"
    
    # Animation
    create_animation = False
    animation_fps = 30


# Create global config instances
rocket = RocketConfig()
sim = SimulationConfig()
control = ControlConfig()
disturbance = DisturbanceConfig()
logging = LoggingConfig()