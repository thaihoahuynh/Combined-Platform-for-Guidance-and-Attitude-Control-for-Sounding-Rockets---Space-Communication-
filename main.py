"""
Main simulation script for sounding rocket guidance and attitude control
Combines all modules to run complete simulation
"""

import numpy as np
import pandas as pd
from tqdm import tqdm

# Import project modules
from config import rocket, sim, control, disturbance, logging
from rocket_dynamics import RocketDynamics
from controllers import AttitudeController, GuidanceController, LowPassFilter
from trajectory import ReferenceTrajectory
from plotting import SimulationPlotter

class RocketSimulation:
    """
    Main simulation class that coordinates all components
    """
    
    def __init__(self, enable_attitude_control=True, enable_guidance_control=False):
        """
        Initialize simulation
        
        Args:
            enable_attitude_control: Enable attitude control
            enable_guidance_control: Enable guidance control
        """
        print("=" * 60)
        print("SOUNDING ROCKET GUIDANCE AND ATTITUDE CONTROL SIMULATION")
        print("=" * 60)
        
        self.enable_attitude = enable_attitude_control
        self.enable_guidance = enable_guidance_control
        
        # Initialize rocket dynamics
        print("\n[1/5] Initializing rocket dynamics...")
        self.rocket = RocketDynamics(rocket, disturbance)
        
        # Initialize controllers
        print("[2/5] Initializing controllers...")
        self.attitude_controller = AttitudeController(control)
        self.guidance_controller = GuidanceController(control)
        
        # Control signal filters
        self.pitch_filter = LowPassFilter(control.filter_cutoff_freq, control.filter_damping)
        self.yaw_filter = LowPassFilter(control.filter_cutoff_freq, control.filter_damping)
        
        # Initialize reference trajectory
        print("[3/5] Generating reference trajectory...")
        self.reference = ReferenceTrajectory(
            launch_elevation=sim.initial_elevation,
            launch_azimuth=sim.initial_azimuth,
            target_apogee=sim.target_apogee,
            target_range=sim.target_range,
            burn_time=rocket.burn_time
        )
        
        # Data storage
        print("[4/5] Setting up data logging...")
        self.data = {
            'time': [],
            'position': [],
            'velocity': [],
            'attitude': [],
            'angular_velocity': [],
            'control': [],
            'reference_position': [],
            'reference_attitude': []
        }
        
        # Plotter
        print("[5/5] Initializing plotter...")
        self.plotter = SimulationPlotter(save_dir=logging.output_directory)
        
        print("\n✓ Initialization complete!\n")
    
    def run(self):
        """
        Run the simulation
        """
        print("=" * 60)
        print("STARTING SIMULATION")
        print("=" * 60)
        print(f"Duration: {sim.duration}s")
        print(f"Time step: {sim.dt}s")
        print(f"Attitude control: {'ENABLED' if self.enable_attitude else 'DISABLED'}")
        print(f"Guidance control: {'ENABLED' if self.enable_guidance else 'DISABLED'}")
        print()
        
        # Reset everything
        self.rocket.reset()
        self.attitude_controller.reset()
        self.guidance_controller.reset()
        self.pitch_filter.reset()
        self.yaw_filter.reset()
        
        # Ignite motor
        self.rocket.ignite_motor()
        
        # Time array
        num_steps = int(sim.duration / sim.dt)
        
        # Simulation loop with progress bar
        print("Running simulation...")
        for step in tqdm(range(num_steps), desc="Progress"):
            time = step * sim.dt
            
            # Get current state
            state = self.rocket.get_state_dict()
            position = state['position']
            velocity = state['velocity']
            attitude = state['attitude']
            altitude = state['altitude']
            
            # Determine flight phase
            if self.rocket.motor_burning:
                phase = 'boost'
            else:
                phase = 'coast'
            
            # Get reference trajectory
            ref_position = self.reference.get_reference_position(time)
            ref_attitude = self.reference.get_reference_attitude(time)
            
            # Guidance control (outer loop)
            if self.enable_guidance and \
               sim.guidance_start_time <= time <= sim.guidance_end_time:
                # Guidance controller outputs desired attitude
                desired_attitude = self.guidance_controller.compute(
                    position, ref_position, velocity, sim.dt
                )
                # Add reference attitude offset
                desired_attitude += ref_attitude
            else:
                # Follow reference attitude directly
                desired_attitude = ref_attitude
            
            # Attitude control (inner loop)
            if self.enable_attitude and time >= sim.control_start_time and \
               time <= sim.control_end_time and altitude > 10:
                
                # Compute control commands
                control_commands = self.attitude_controller.compute(
                    attitude, desired_attitude, sim.dt, phase
                )
                
                # Extract pitch and yaw commands (ignore roll for TVC)
                pitch_cmd = control_commands[1]
                yaw_cmd = control_commands[2]
                
                # Apply low-pass filtering
                pitch_filtered = self.pitch_filter.filter(pitch_cmd, sim.dt)
                yaw_filtered = self.yaw_filter.filter(yaw_cmd, sim.dt)
                
                # Apply control limits
                max_deflection = np.deg2rad(rocket.max_deflection)
                pitch_limited = np.clip(pitch_filtered, -max_deflection, max_deflection)
                yaw_limited = np.clip(yaw_filtered, -max_deflection, max_deflection)
                
                control_deflections = np.array([pitch_limited, yaw_limited])
            else:
                # No control
                control_deflections = np.zeros(2)
            
            # Update rocket dynamics
            self.rocket.update(sim.dt, control_deflections)
            
            # Log data (at specified intervals)
            if step % logging.log_interval == 0:
                self.data['time'].append(time)
                self.data['position'].append(position.copy())
                self.data['velocity'].append(velocity.copy())
                self.data['attitude'].append(attitude.copy())
                self.data['angular_velocity'].append(state['angular_velocity'].copy())
                self.data['control'].append(control_deflections.copy())
                self.data['reference_position'].append(ref_position.copy())
                self.data['reference_attitude'].append(desired_attitude.copy())
            
            # Stop if rocket has landed
            if altitude <= 0 and time > 10:
                print(f"\n✓ Rocket has landed at t = {time:.2f}s")
                break
        
        # Convert lists to arrays
        print("\n✓ Simulation complete!")
        print("\nPost-processing data...")
        self._post_process_data()
        
        print("✓ Data processed!")
    
    def _post_process_data(self):
        """Convert logged data to numpy arrays"""
        self.data['time'] = np.array(self.data['time'])
        self.data['position'] = np.array(self.data['position'])
        self.data['velocity'] = np.array(self.data['velocity'])
        self.data['attitude'] = np.array(self.data['attitude'])
        self.data['angular_velocity'] = np.array(self.data['angular_velocity'])
        self.data['control'] = np.array(self.data['control'])
        self.data['reference_position'] = np.array(self.data['reference_position'])
        self.data['reference_attitude'] = np.array(self.data['reference_attitude'])
    
    def print_summary(self):
        """Print simulation summary statistics"""
        print("\n" + "=" * 60)
        print("SIMULATION SUMMARY")
        print("=" * 60)
        
        altitude = -self.data['position'][:, 2]
        velocity = self.data['velocity']
        V_total = np.linalg.norm(velocity, axis=1)
        
        # Find apogee
        apogee_idx = np.argmax(altitude)
        apogee_altitude = altitude[apogee_idx]
        apogee_time = self.data['time'][apogee_idx]
        
        # Max velocity
        max_vel_idx = np.argmax(V_total)
        max_velocity = V_total[max_vel_idx]
        max_vel_time = self.data['time'][max_vel_idx]
        
        # Final position
        final_north = self.data['position'][-1, 0]
        final_east = self.data['position'][-1, 1]
        final_range = np.sqrt(final_north**2 + final_east**2)
        
        # Maximum control deflections
        max_pitch = np.max(np.abs(self.data['control'][:, 0]))
        max_yaw = np.max(np.abs(self.data['control'][:, 1]))
        
        print(f"\nTrajectory:")
        print(f"  Apogee altitude:     {apogee_altitude:.1f} m (at t={apogee_time:.1f}s)")
        print(f"  Max velocity:        {max_velocity:.1f} m/s (at t={max_vel_time:.1f}s)")
        print(f"  Final range:         {final_range:.1f} m")
        print(f"  Final position:      N={final_north:.1f}m, E={final_east:.1f}m")
        print(f"  Flight duration:     {self.data['time'][-1]:.1f} s")
        
        print(f"\nControl Performance:")
        print(f"  Max pitch deflection: {np.rad2deg(max_pitch):.2f}°")
        print(f"  Max yaw deflection:   {np.rad2deg(max_yaw):.2f}°")
        
        # Attitude errors (during controlled flight)
        if self.enable_attitude:
            control_mask = (self.data['time'] >= sim.control_start_time) & \
                          (self.data['time'] <= sim.control_end_time)
            
            att_error = self.data['attitude'][control_mask] - \
                       self.data['reference_attitude'][control_mask]
            
            # Wrap angles
            att_error = np.arctan2(np.sin(att_error), np.cos(att_error))
            
            rms_pitch_error = np.sqrt(np.mean(att_error[:, 1]**2))
            rms_yaw_error = np.sqrt(np.mean(att_error[:, 2]**2))
            
            print(f"  RMS pitch error:      {np.rad2deg(rms_pitch_error):.3f}°")
            print(f"  RMS yaw error:        {np.rad2deg(rms_yaw_error):.3f}°")
        
        # Position errors (during guidance)
        if self.enable_guidance:
            guidance_mask = (self.data['time'] >= sim.guidance_start_time) & \
                           (self.data['time'] <= sim.guidance_end_time)
            
            pos_error = self.data['position'][guidance_mask] - \
                       self.data['reference_position'][guidance_mask]
            
            rms_lateral_error = np.sqrt(np.mean(pos_error[:, 0]**2 + pos_error[:, 1]**2))
            
            print(f"  RMS lateral position error: {rms_lateral_error:.1f} m")
        
        print("\n" + "=" * 60 + "\n")
    
    def plot_results(self):
        """Generate all plots"""
        print("Generating plots...")
        
        # Prepare reference data in same format
        ref_data = {
            'time': self.data['time'],
            'position': self.data['reference_position'],
            'altitude': -self.data['reference_position'][:, 2],
            'attitude': self.data['reference_attitude']
        }
        
        # Create plots
        if logging.plot_3d_trajectory:
            print("  - 3D trajectory")
            self.plotter.plot_3d_trajectory(self.data, save=logging.save_plots)
        
        if logging.plot_altitude:
            print("  - Altitude vs time")
            self.plotter.plot_altitude_vs_time(self.data, ref_data, save=logging.save_plots)
        
        if logging.plot_attitudes:
            print("  - Attitude angles")
            self.plotter.plot_attitudes(self.data, ref_data, save=logging.save_plots)
        
        if logging.plot_control_deflections and self.enable_attitude:
            print("  - Control deflections")
            self.plotter.plot_control_deflections(self.data, save=logging.save_plots)
        
        if logging.plot_velocities:
            print("  - Velocities")
            self.plotter.plot_velocities(self.data, save=logging.save_plots)
        
        if logging.plot_angular_rates:
            print("  - Angular rates")
            self.plotter.plot_angular_rates(self.data, save=logging.save_plots)
        
        if logging.plot_position_errors and self.enable_guidance:
            print("  - Position errors")
            self.plotter.plot_position_errors(self.data, ref_data, save=logging.save_plots)
        
        print("  - Footprint")
        self.plotter.plot_footprint(self.data, ref_data, save=logging.save_plots)
        
        print("  - Summary plot")
        self.plotter.create_summary_plot(self.data, ref_data, save=logging.save_plots)
        
        print("✓ All plots generated!")
    
    def save_data_to_csv(self):
        """Save simulation data to CSV file"""
        if logging.save_data_to_csv:
            print("\nSaving data to CSV...")
            
            # Create dataframe
            df = pd.DataFrame({
                'time': self.data['time'],
                'north': self.data['position'][:, 0],
                'east': self.data['position'][:, 1],
                'altitude': -self.data['position'][:, 2],
                'Vn': self.data['velocity'][:, 0],
                'Ve': self.data['velocity'][:, 1],
                'Vd': self.data['velocity'][:, 2],
                'roll': np.rad2deg(self.data['attitude'][:, 0]),
                'pitch': np.rad2deg(self.data['attitude'][:, 1]),
                'yaw': np.rad2deg(self.data['attitude'][:, 2]),
                'p': np.rad2deg(self.data['angular_velocity'][:, 0]),
                'q': np.rad2deg(self.data['angular_velocity'][:, 1]),
                'r': np.rad2deg(self.data['angular_velocity'][:, 2]),
                'delta_pitch': np.rad2deg(self.data['control'][:, 0]),
                'delta_yaw': np.rad2deg(self.data['control'][:, 1])
            })
            
            filename = f"{logging.output_directory}/simulation_data.csv"
            df.to_csv(filename, index=False)
            print(f"✓ Data saved to: {filename}")


def main():
    """
    Main function to run simulation scenarios
    """
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║" + " " * 10 + "ROCKET CONTROL SIMULATION SCENARIOS" + " " * 12 + "║")
    print("╚" + "═" * 58 + "╝")
    print()
    
    # Scenario selection
    print("Select simulation scenario:")
    print("  1. No control (baseline)")
    print("  2. Attitude control only")
    print("  3. Attitude + Guidance control")
    print()
    
    choice = input("Enter choice (1-3) [default: 2]: ").strip()
    if not choice:
        choice = "2"
    
    # Configure scenario
    if choice == "1":
        enable_attitude = False
        enable_guidance = False
        scenario_name = "No Control (Baseline)"
    elif choice == "2":
        enable_attitude = True
        enable_guidance = False
        scenario_name = "Attitude Control Only"
    elif choice == "3":
        enable_attitude = True
        enable_guidance = True
        scenario_name = "Attitude + Guidance Control"
    else:
        print("Invalid choice! Using default (Attitude control only)")
        enable_attitude = True
        enable_guidance = False
        scenario_name = "Attitude Control Only"
    
    print(f"\n→ Running scenario: {scenario_name}\n")
    
    # Create and run simulation
    sim_instance = RocketSimulation(
        enable_attitude_control=enable_attitude,
        enable_guidance_control=enable_guidance
    )
    
    sim_instance.run()
    sim_instance.print_summary()
    sim_instance.save_data_to_csv()
    sim_instance.plot_results()
    
    print("\n" + "=" * 60)
    print("SIMULATION COMPLETE!")
    print("=" * 60)
    print(f"Results saved to: {logging.output_directory}/")
    print()


if __name__ == "__main__":
    main()