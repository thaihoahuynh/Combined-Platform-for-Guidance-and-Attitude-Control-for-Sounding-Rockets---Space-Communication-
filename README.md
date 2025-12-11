# Sounding Rocket Guidance and Attitude Control Simulation

A comprehensive Python simulation of a sounding rocket with thrust vector control (TVC), featuring 6-DOF dynamics, PID-based attitude control, and trajectory guidance.

## 📋 Project Overview

This project implements a complete simulation framework for a small sounding rocket with active control. It includes:

- **6-DOF Rigid Body Dynamics**: Full equations of motion in 3D space
- **Atmospheric Model**: Standard atmosphere with wind disturbances
- **Aerodynamic Model**: Coefficient-based forces and moments
- **Thrust Vector Control**: Nozzle deflection for attitude control
- **PID Controllers**: Separate controllers for roll, pitch, and yaw
- **Guidance System**: Trajectory tracking with outer-loop control
- **Comprehensive Visualization**: Multiple plots for analysis

## 🚀 Features

### Flight Dynamics
- 6 degree of freedom equations of motion
- Time-varying mass and inertia
- Euler angle kinematics with DCM transformations
- RK4 numerical integration

### Aerodynamics
- Angle of attack and sideslip angle calculations
- Mach-dependent coefficients
- Control surface effectiveness
- Damping moments

### Control System
- **Attitude Control**: Three-axis PID controllers
- **Guidance Control**: Position-based trajectory tracking
- **Gain Scheduling**: Phase-dependent controller gains
- **Low-pass Filtering**: Noise reduction and stabilization
- **Anti-windup**: Integral term saturation prevention

### Environmental Models
- International Standard Atmosphere (ISA)
- Constant wind with turbulence
- Parametric uncertainties

## 📁 Project Structure

```
rocket_control_project/
│
├── main.py                 # Main simulation script
├── config.py              # Configuration parameters
├── rocket_dynamics.py     # 6-DOF dynamics model
├── controllers.py         # PID and guidance controllers
├── aerodynamics.py        # Aerodynamic forces/moments
├── atmosphere.py          # Atmospheric properties
├── trajectory.py          # Reference trajectory generation
├── plotting.py            # Visualization functions
├── requirements.txt       # Python dependencies
└── README.md             # This file
```

## 🛠️ Installation

### Prerequisites
- Python 3.8 or higher
- pip package manager

### Setup

1. Create project directory:
```bash
mkdir rocket_control_project
cd rocket_control_project
```

2. Copy all Python files to the directory

3. Install dependencies:
```bash
pip install -r requirements.txt
```

## 🎯 Usage

### Basic Simulation

Run the main simulation script:
```bash
python main.py
```

You'll be prompted to select a scenario:
1. **No control**: Baseline trajectory without control
2. **Attitude control only**: Stabilizes attitude angles
3. **Attitude + Guidance control**: Full trajectory tracking

### Configuration

Edit `config.py` to modify:
- **Rocket parameters**: Mass, geometry, inertia
- **Motor specifications**: Thrust, burn time
- **Control gains**: PID parameters
- **Disturbances**: Wind, uncertainties
- **Simulation settings**: Duration, time step

Example: Change target apogee
```python
# In config.py, modify SimulationConfig class
target_apogee = 10000.0  # Changed to 10 km
```

### Running Individual Modules

Test individual components:

```bash
# Test atmosphere model
python atmosphere.py

# Test aerodynamics
python aerodynamics.py

# Test controllers
python controllers.py

# Test rocket dynamics
python rocket_dynamics.py
```

## 📊 Output

### Generated Plots
The simulation creates several plots in `simulation_results/`:

1. **3D Trajectory**: Complete flight path in 3D space
2. **Altitude vs Time**: Vertical profile with apogee
3. **Attitude Angles**: Roll, pitch, yaw evolution
4. **Control Deflections**: Actuator commands over time
5. **Velocities**: Velocity components and magnitude
6. **Angular Rates**: Body-frame rotation rates
7. **Position Errors**: Tracking performance (guidance mode)
8. **Footprint**: Ground track projection
9. **Summary Plot**: Comprehensive overview

### Data Export
Simulation data is saved to CSV format:
- `simulation_results/simulation_data.csv`

Contains time-series data for all state variables and control inputs.

## 🔬 Technical Details

### Coordinate Systems

**NED Frame (North-East-Down)**
- Origin at launch point
- X-axis: North
- Y-axis: East
- Z-axis: Down (towards Earth center)

**Body Frame**
- Origin at rocket center of mass
- X-axis: Along rocket longitudinal axis (nose direction)
- Y-axis: Right wing (if present)
- Z-axis: Down (perpendicular to X in vertical plane)

### State Vector
```
[N, E, D, Vn, Ve, Vd, φ, θ, ψ, p, q, r]
```
- Position: (N, E, D) in meters
- Velocity: (Vn, Ve, Vd) in m/s
- Attitude: (φ, θ, ψ) roll, pitch, yaw in radians
- Angular velocity: (p, q, r) in rad/s

### Controller Architecture

```
Reference Trajectory
        ↓
    Guidance Controller (Outer Loop)
        ↓
    Desired Attitude
        ↓
    Attitude Controller (Inner Loop)
        ↓
    Control Deflections
        ↓
    Rocket Dynamics
```

### PID Controller Equations

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de/dt
```

With:
- Proportional gain (Kp): Immediate response
- Integral gain (Ki): Eliminates steady-state error
- Derivative gain (Kd): Damping, reduces overshoot

## 🎛️ Tuning Guidelines

### Attitude Controller Gains

**Starting values** (in `config.py`):
- Roll: Kp=1.5, Ki=0.5, Kd=0.3
- Pitch/Yaw: Kp=1.8, Ki=0.6, Kd=0.4

**Tuning process**:
1. Start with Kp only (Ki=Kd=0)
2. Increase Kp until oscillations begin
3. Add Kd to dampen oscillations
4. Add Ki to remove steady-state error
5. Fine-tune all three gains iteratively

**Effects**:
- ↑ Kp: Faster response, more oscillation
- ↑ Ki: Eliminates steady-state error, can cause overshoot
- ↑ Kd: Better damping, sensitive to noise

### Guidance Controller Gains

**Starting values**:
- Kp=0.3, Ki=0.1, Kd=0.5

**Considerations**:
- Lower gains than attitude (slower outer loop)
- Focus on steady tracking, not fast response
- Avoid aggressive commands

## 🧪 Example Scenarios

### Scenario 1: Wind Disturbance Test

In `config.py`, set:
```python
wind_north = 10.0  # 10 m/s north wind
wind_east = 5.0    # 5 m/s east wind
wind_turbulence_intensity = 0.2  # 20% turbulence
```

Run with attitude control to see stabilization performance.

### Scenario 2: Uncertainty Analysis

Add parametric uncertainties:
```python
thrust_uncertainty = 0.1   # ±10% thrust
mass_uncertainty = 0.05    # ±5% mass
aero_uncertainty = 0.15    # ±15% aerodynamics
```

Run multiple simulations (Monte Carlo) to assess robustness.

### Scenario 3: High Altitude Mission

Change target:
```python
target_apogee = 30000.0  # 30 km apogee
burn_time = 8.0          # Longer burn
thrust_nominal = 3000.0  # More thrust
```

## 📈 Performance Metrics

The simulation computes:
- **Apogee altitude**: Maximum height reached
- **Apogee time**: Time to reach apogee
- **Maximum velocity**: Peak speed during flight
- **Final range**: Horizontal distance from launch
- **RMS attitude error**: Control accuracy
- **RMS position error**: Guidance accuracy
- **Maximum control deflection**: Actuator usage

## 🐛 Troubleshooting

### Common Issues

**1. Simulation becomes unstable**
- Reduce time step (dt) in `config.py`
- Lower PID gains
- Check for numerical issues at singular attitudes

**2. Controller doesn't stabilize**
- Verify gains are positive
- Check control limits aren't too restrictive
- Ensure control is enabled during appropriate phase

**3. Import errors**
- Verify all files are in same directory
- Check Python version (≥3.8)
- Reinstall dependencies: `pip install -r requirements.txt`

**4. Plots don't appear**
- Check matplotlib backend
- Try: `import matplotlib; matplotlib.use('TkAgg')`
- Ensure display is available (for remote servers, save plots instead)

### Validation Checks

Test each module individually:
```bash
python atmosphere.py    # Should print atmosphere properties
python aerodynamics.py  # Should print force/moment values
python controllers.py   # Should show PID step response
```

## 📚 References

This implementation is based on:

1. **FOK Rocket Project** (Warsaw University of Technology)
   - IAC-20,D2,6,9,x60288: "Low cost rocket guidance and control development platform"
   - PID control design methodology
   - Wind tunnel testing approach

2. **VS-50 Launch Vehicle** (Brazilian-German Cooperation)
   - s42405-025-01027-0: "Robust Attitude and Guidance Control for a Launch Vehicle System"
   - LMI-based robust control concepts
   - Hardware-in-the-loop validation

3. **Classical References**
   - Stevens & Lewis: "Aircraft Control and Simulation"
   - Sutton & Biblarz: "Rocket Propulsion Elements"
   - Zipfel: "Modeling and Simulation of Aerospace Vehicle Dynamics"

## 🎓 Educational Value

This project is designed for undergraduate/graduate aerospace engineering students to:

- Understand 6-DOF dynamics formulation
- Learn control system design and tuning
- Explore guidance algorithms
- Practice numerical simulation techniques
- Analyze flight performance

## 🔮 Future Enhancements

Potential extensions:
- [ ] LQR/LQG optimal control
- [ ] Model predictive control (MPC)
- [ ] Kalman filter for state estimation
- [ ] Multiple stage rockets
- [ ] Flexible body dynamics (bending modes)
- [ ] 3D visualization/animation
- [ ] Real-time plotting during simulation
- [ ] Monte Carlo analysis automation
- [ ] Trajectory optimization

## 📝 License

This is an educational project. Feel free to use and modify for academic purposes.

## 👥 Contributing

Suggestions and improvements welcome! This is a learning project, so:
- Clear comments are valued
- Educational clarity over optimization
- Test before committing changes

## 📧 Contact

For questions about this project, please refer to the course instructor or teaching assistant.

---

**Happy simulating! 🚀**
