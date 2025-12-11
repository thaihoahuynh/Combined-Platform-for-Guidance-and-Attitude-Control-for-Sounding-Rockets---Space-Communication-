"""
Plotting and visualization functions for simulation results
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

class SimulationPlotter:
    """
    Create plots and visualizations of simulation results
    """
    
    def __init__(self, save_dir="simulation_results"):
        """
        Initialize plotter
        
        Args:
            save_dir: Directory to save plots
        """
        self.save_dir = save_dir
        
        # Create directory if it doesn't exist
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        # Set plot style
        plt.style.use('seaborn-v0_8-darkgrid')
        self.colors = plt.cm.tab10(np.linspace(0, 1, 10))
    
    def plot_3d_trajectory(self, data, save=True):
        """
        Plot 3D trajectory
        
        Args:
            data: Dictionary with time-series data
            save: Whether to save the plot
        """
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        north = data['position'][:, 0]
        east = data['position'][:, 1]
        altitude = -data['position'][:, 2]  # Convert down to up
        
        # Plot trajectory
        scatter = ax.scatter(north, east, altitude, c=data['time'], 
                           cmap='viridis', s=10, alpha=0.6)
        
        # Mark important points
        ax.scatter([north[0]], [east[0]], [altitude[0]], 
                  color='green', s=200, marker='^', label='Launch')
        
        apogee_idx = np.argmax(altitude)
        ax.scatter([north[apogee_idx]], [east[apogee_idx]], [altitude[apogee_idx]], 
                  color='red', s=200, marker='*', label='Apogee')
        
        # Labels
        ax.set_xlabel('North (m)', fontsize=12)
        ax.set_ylabel('East (m)', fontsize=12)
        ax.set_zlabel('Altitude (m)', fontsize=12)
        ax.set_title('3D Trajectory', fontsize=14, fontweight='bold')
        
        # Colorbar
        cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
        cbar.set_label('Time (s)', fontsize=10)
        
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save:
            plt.savefig(f"{self.save_dir}/3d_trajectory.png", dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_altitude_vs_time(self, data, reference=None, save=True):
        """
        Plot altitude vs time
        
        Args:
            data: Dictionary with time-series data
            reference: Optional reference trajectory data
            save: Whether to save the plot
        """
        fig, ax = plt.subplots(figsize=(12, 6))
        
        altitude = -data['position'][:, 2]
        
        ax.plot(data['time'], altitude, linewidth=2, label='Actual', color=self.colors[0])
        
        if reference is not None:
            ax.plot(reference['time'], reference['altitude'], 
                   '--', linewidth=2, label='Reference', color=self.colors[1])
        
        # Mark apogee
        apogee_idx = np.argmax(altitude)
        ax.plot(data['time'][apogee_idx], altitude[apogee_idx], 
               'r*', markersize=20, label=f'Apogee: {altitude[apogee_idx]:.0f}m')
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Altitude (m)', fontsize=12)
        ax.set_title('Altitude vs Time', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save:
            plt.savefig(f"{self.save_dir}/altitude_vs_time.png", dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_attitudes(self, data, reference=None, save=True):
        """
        Plot attitude angles vs time
        
        Args:
            data: Dictionary with time-series data
            reference: Optional reference attitude data
            save: Whether to save the plot
        """
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        attitude_deg = np.rad2deg(data['attitude'])
        attitude_labels = ['Roll', 'Pitch', 'Yaw']
        
        for i, (ax, label) in enumerate(zip(axes, attitude_labels)):
            ax.plot(data['time'], attitude_deg[:, i], 
                   linewidth=2, label='Actual', color=self.colors[i])
            
            if reference is not None:
                ref_deg = np.rad2deg(reference['attitude'][:, i])
                ax.plot(data['time'], ref_deg, '--', 
                       linewidth=2, label='Reference', color=self.colors[i+3])
            
            ax.set_ylabel(f'{label} (deg)', fontsize=11)
            ax.legend(fontsize=9)
            ax.grid(True, alpha=0.3)
        
        axes[-1].set_xlabel('Time (s)', fontsize=12)
        fig.suptitle('Attitude Angles vs Time', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        
        if save:
            plt.savefig(f"{self.save_dir}/attitudes.png", dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_control_deflections(self, data, save=True):
        """
        Plot control deflections vs time
        
        Args:
            data: Dictionary with time-series data
            save: Whether to save the plot
        """
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        control_deg = np.rad2deg(data['control'])
        
        axes[0].plot(data['time'], control_deg[:, 0], 
                    linewidth=2, color=self.colors[0])
        axes[0].set_ylabel('Pitch Deflection (deg)', fontsize=11)
        axes[0].grid(True, alpha=0.3)
        axes[0].axhline(y=10, color='r', linestyle='--', alpha=0.5, label='Limit')
        axes[0].axhline(y=-10, color='r', linestyle='--', alpha=0.5)
        axes[0].legend(fontsize=9)
        
        axes[1].plot(data['time'], control_deg[:, 1], 
                    linewidth=2, color=self.colors[1])
        axes[1].set_ylabel('Yaw Deflection (deg)', fontsize=11)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        axes[1].axhline(y=10, color='r', linestyle='--', alpha=0.5, label='Limit')
        axes[1].axhline(y=-10, color='r', linestyle='--', alpha=0.5)
        axes[1].legend(fontsize=9)
        
        fig.suptitle('Control Deflections vs Time', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        
        if save:
            plt.savefig(f"{self.save_dir}/control_deflections.png", dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_velocities(self, data, save=True):
        """
        Plot velocity components vs time
        
        Args:
            data: Dictionary with time-series data
            save: Whether to save the plot
        """
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        # Velocity components
        axes[0].plot(data['time'], data['velocity'][:, 0], 
                    linewidth=2, label='North', color=self.colors[0])
        axes[0].plot(data['time'], data['velocity'][:, 1], 
                    linewidth=2, label='East', color=self.colors[1])
        axes[0].plot(data['time'], -data['velocity'][:, 2], 
                    linewidth=2, label='Up', color=self.colors[2])
        axes[0].set_ylabel('Velocity (m/s)', fontsize=11)
        axes[0].legend(fontsize=9)
        axes[0].grid(True, alpha=0.3)
        
        # Total velocity magnitude
        V_total = np.linalg.norm(data['velocity'], axis=1)
        axes[1].plot(data['time'], V_total, 
                    linewidth=2, color=self.colors[3])
        axes[1].set_ylabel('Total Velocity (m/s)', fontsize=11)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        
        fig.suptitle('Velocity vs Time', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        
        if save:
            plt.savefig(f"{self.save_dir}/velocities.png", dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_angular_rates(self, data, save=True):
        """
        Plot angular rates vs time
        
        Args:
            data: Dictionary with time-series data
            save: Whether to save the plot
        """
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        angular_rates_deg = np.rad2deg(data['angular_velocity'])
        rate_labels = ['Roll Rate (p)', 'Pitch Rate (q)', 'Yaw Rate (r)']
        
        for i, (ax, label) in enumerate(zip(axes, rate_labels)):
            ax.plot(data['time'], angular_rates_deg[:, i], 
                   linewidth=2, color=self.colors[i])
            ax.set_ylabel(f'{label} (deg/s)', fontsize=11)
            ax.grid(True, alpha=0.3)
        
        axes[-1].set_xlabel('Time (s)', fontsize=12)
        fig.suptitle('Angular Rates vs Time', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        
        if save:
            plt.savefig(f"{self.save_dir}/angular_rates.png", dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_position_errors(self, data, reference, save=True):
        """
        Plot position tracking errors
        
        Args:
            data: Dictionary with actual trajectory data
            reference: Dictionary with reference trajectory data
            save: Whether to save the plot
        """
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # Calculate errors
        error_north = reference['position'][:, 0] - data['position'][:, 0]
        error_east = reference['position'][:, 1] - data['position'][:, 1]
        error_altitude = reference['altitude'] - (-data['position'][:, 2])
        
        axes[0].plot(data['time'], error_north, linewidth=2, color=self.colors[0])
        axes[0].set_ylabel('North Error (m)', fontsize=11)
        axes[0].grid(True, alpha=0.3)
        axes[0].axhline(y=0, color='k', linestyle='-', alpha=0.3)
        
        axes[1].plot(data['time'], error_east, linewidth=2, color=self.colors[1])
        axes[1].set_ylabel('East Error (m)', fontsize=11)
        axes[1].grid(True, alpha=0.3)
        axes[1].axhline(y=0, color='k', linestyle='-', alpha=0.3)
        
        axes[2].plot(data['time'], error_altitude, linewidth=2, color=self.colors[2])
        axes[2].set_ylabel('Altitude Error (m)', fontsize=11)
        axes[2].set_xlabel('Time (s)', fontsize=12)
        axes[2].grid(True, alpha=0.3)
        axes[2].axhline(y=0, color='k', linestyle='-', alpha=0.3)
        
        fig.suptitle('Position Tracking Errors', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        
        if save:
            plt.savefig(f"{self.save_dir}/position_errors.png", dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_footprint(self, data, reference=None, save=True):
        """
        Plot ground track (footprint)
        
        Args:
            data: Dictionary with time-series data
            reference: Optional reference trajectory
            save: Whether to save the plot
        """
        fig, ax = plt.subplots(figsize=(10, 10))
        
        north = data['position'][:, 0]
        east = data['position'][:, 1]
        
        # Plot actual trajectory
        scatter = ax.scatter(east, north, c=data['time'], 
                           cmap='viridis', s=20, alpha=0.7)
        
        # Plot reference if available
        if reference is not None:
            ax.plot(reference['position'][:, 1], reference['position'][:, 0], 
                   'r--', linewidth=2, alpha=0.5, label='Reference')
        
        # Mark launch and impact
        ax.plot(east[0], north[0], 'g^', markersize=15, label='Launch')
        ax.plot(east[-1], north[-1], 'rs', markersize=15, label='Impact')
        
        ax.set_xlabel('East (m)', fontsize=12)
        ax.set_ylabel('North (m)', fontsize=12)
        ax.set_title('Ground Track (Footprint)', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Time (s)', fontsize=10)
        
        plt.tight_layout()
        
        if save:
            plt.savefig(f"{self.save_dir}/footprint.png", dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def create_summary_plot(self, data, reference=None, save=True):
        """
        Create a comprehensive summary plot
        
        Args:
            data: Dictionary with time-series data
            reference: Optional reference trajectory
            save: Whether to save the plot
        """
        fig = plt.figure(figsize=(16, 12))
        
        # 3D trajectory
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        north = data['position'][:, 0]
        east = data['position'][:, 1]
        altitude = -data['position'][:, 2]
        ax1.plot(north, east, altitude, linewidth=2)
        ax1.set_xlabel('North (m)')
        ax1.set_ylabel('East (m)')
        ax1.set_zlabel('Altitude (m)')
        ax1.set_title('3D Trajectory')
        
        # Altitude
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(data['time'], altitude, linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Altitude (m)')
        ax2.set_title('Altitude')
        ax2.grid(True, alpha=0.3)
        
        # Velocity
        ax3 = fig.add_subplot(2, 3, 3)
        V_total = np.linalg.norm(data['velocity'], axis=1)
        ax3.plot(data['time'], V_total, linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Velocity (m/s)')
        ax3.set_title('Total Velocity')
        ax3.grid(True, alpha=0.3)
        
        # Attitudes
        ax4 = fig.add_subplot(2, 3, 4)
        attitude_deg = np.rad2deg(data['attitude'])
        ax4.plot(data['time'], attitude_deg[:, 0], label='Roll')
        ax4.plot(data['time'], attitude_deg[:, 1], label='Pitch')
        ax4.plot(data['time'], attitude_deg[:, 2], label='Yaw')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Angle (deg)')
        ax4.set_title('Attitude Angles')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # Control deflections
        ax5 = fig.add_subplot(2, 3, 5)
        control_deg = np.rad2deg(data['control'])
        ax5.plot(data['time'], control_deg[:, 0], label='Pitch')
        ax5.plot(data['time'], control_deg[:, 1], label='Yaw')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Deflection (deg)')
        ax5.set_title('Control Deflections')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # Footprint
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.plot(east, north, linewidth=2)
        ax6.plot(east[0], north[0], 'g^', markersize=10, label='Launch')
        ax6.plot(east[-1], north[-1], 'rs', markersize=10, label='Impact')
        ax6.set_xlabel('East (m)')
        ax6.set_ylabel('North (m)')
        ax6.set_title('Ground Track')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        plt.tight_layout()
        
        if save:
            plt.savefig(f"{self.save_dir}/summary_plot.png", dpi=300, bbox_inches='tight')
        
        plt.show()


# Example usage
if __name__ == "__main__":
    print("Plotting module loaded successfully!")
    print("Use SimulationPlotter class to visualize simulation results.")