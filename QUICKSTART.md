# Quick Start Guide

Get your rocket simulation running in 5 minutes!

## 🚀 Installation (2 minutes)

### Step 1: Install Python
Download Python 3.8+ from [python.org](https://www.python.org/downloads/)

### Step 2: Create Project Folder
```bash
mkdir rocket_simulation
cd rocket_simulation
```

### Step 3: Save All Files
Copy these 9 files into your folder:
- `main.py`
- `config.py`
- `rocket_dynamics.py`
- `controllers.py`
- `aerodynamics.py`
- `atmosphere.py`
- `trajectory.py`
- `plotting.py`
- `requirements.txt`

### Step 4: Install Dependencies
```bash
pip install -r requirements.txt
```

## ▶️ Run Your First Simulation (1 minute)

```bash
python main.py
```

**Select option 2** (Attitude control only) when prompted.

Wait ~30 seconds for simulation to complete.

## 📊 View Results

Results are saved in `simulation_results/` folder:
- **3d_trajectory.png**: See the full flight path
- **altitude_vs_time.png**: Height profile
- **attitudes.png**: Rocket orientation
- **summary_plot.png**: Everything at once
- **simulation_data.csv**: All the numbers

## 🎯 What You Should See

**Console Output:**
```
Apogee altitude:     ~5000 m
Max velocity:        ~220 m/s
Flight duration:     ~30 s
```

**Plots:**
- Smooth trajectory climbing to ~5 km
- Attitude angles stabilizing near reference
- Small control deflections (<5 degrees)

## ✅ Quick Tests

### Test 1: No Control (Baseline)
Run `main.py` and select option 1
- Rocket will tumble without control
- Compare with controlled flight

### Test 2: Add Wind
Edit `config.py`:
```python
wind_north = 10.0  # Add 10 m/s wind
```
Run simulation again - controller should compensate!

### Test 3: Change Target Apogee
Edit `config.py`:
```python
target_apogee = 10000.0  # Aim for 10 km
```
Note: Current motor may not reach this high!

## 🎛️ Basic Modifications

### Make Rocket Go Higher
In `config.py`:
```python
thrust_nominal = 3000.0  # Increase thrust (was 2000)
burn_time = 7.0          # Burn longer (was 5)
```

### Make Control More Aggressive
In `config.py`:
```python
pitch_Kp = 2.5  # Increase from 1.8
pitch_Kd = 0.6  # Increase from 0.4
```

### Change Launch Angle
In `config.py`:
```python
initial_elevation = 75.0  # More horizontal (was 85)
```

## 🐛 Troubleshooting

### Problem: "Module not found"
**Solution:** Make sure all files are in same folder
```bash
ls  # Should show all 9 .py files
```

### Problem: Plots don't show
**Solution:** They're saved as PNG files in `simulation_results/`
Check that folder!

### Problem: Simulation crashes
**Solution:** Restore default `config.py` settings
- Delete your modified config.py
- Copy the original again

### Problem: Results look weird
**Solution:** Check these common mistakes:
- Did you enter valid numbers in config?
- Are PID gains positive?
- Is thrust > 0?

## 📚 Next Steps

### 1. Run All Three Scenarios
- No control
- Attitude control
- Guidance control

Compare the results!

### 2. Experiment with Parameters
Try changing one thing at a time:
- Wind speed
- Control gains
- Rocket mass
- Target altitude

### 3. Understand the Code
Read through files in this order:
1. `config.py` - See what you can change
2. `main.py` - Understand the simulation flow
3. `controllers.py` - Learn how PID works
4. `rocket_dynamics.py` - See the physics

### 4. For Your Report
Include these plots:
- 3D trajectory (shows full path)
- Altitude vs time (proves you hit target)
- Attitudes (shows control works)
- Control deflections (shows actuator usage)
- Comparison: with vs without control

### 5. Analysis Ideas
Calculate and discuss:
- Control effectiveness (RMS error)
- Robustness (try with disturbances)
- Sensitivity (vary one parameter ±20%)
- Comparison with literature (FOK rocket, VS-50)

## 💡 Pro Tips

1. **Always start simple**: Get basic simulation working before adding complexity

2. **One change at a time**: When tuning, change one gain and observe effect

3. **Save your configs**: Make a copy of config.py before major changes
   ```bash
   cp config.py config_backup.py
   ```

4. **Check the console**: Simulation prints useful info - read it!

5. **Compare scenarios**: Run baseline first, then controlled flight - the difference is impressive!

6. **Document everything**: Take screenshots of good results for your report

## 🎓 Understanding the Simulation

### What's Actually Happening?

1. **Rocket launches** vertically at 85 degrees
2. **Motor burns** for 5 seconds, providing thrust
3. **Controller activates** after 0.5 seconds
4. **Attitude control** keeps rocket pointed correctly
5. **Guidance control** (if enabled) adjusts path toward target
6. **Coasting phase** after burnout (~5-30s)
7. **Apogee** reached around 15-20 seconds
8. **Descent** back to ground

### Key Physics

- **Gravity** pulls rocket down (9.81 m/s²)
- **Thrust** pushes rocket forward (2000 N for 5s)
- **Aerodynamics** creates drag and moments
- **Control** deflects thrust to change attitude
- **Wind** (if enabled) pushes rocket off course

### Controller Logic

```
Error = Desired - Actual
Command = Kp×Error + Ki×∫Error + Kd×(dError/dt)
```

The controller continuously corrects to minimize error!

## 🎯 Validation Checklist

Before submitting, verify:
- [ ] Simulation runs without errors
- [ ] Apogee is reasonable (3-6 km with default settings)
- [ ] Max velocity is realistic (~200-250 m/s)
- [ ] Control deflections are within limits (±10°)
- [ ] Plots are generated and saved
- [ ] CSV data file is created
- [ ] You can explain what each plot shows

## 📞 Getting Help

If stuck:
1. Check error message carefully
2. Read the relevant section in README.md
3. Try running individual modules (e.g., `python atmosphere.py`)
4. Restore default settings and try again
5. Ask instructor/TA with specific error message

## 🎉 Success Indicators

You're on the right track if:
- ✅ Simulation completes without crashes
- ✅ Plots show smooth trajectories
- ✅ Apogee is within expected range
- ✅ Attitude errors decrease over time (with control)
- ✅ Control deflections are reasonable (<10°)
- ✅ You can run different scenarios

## 🚀 Ready for More?

Once basics work, try:
- Monte Carlo analysis (run 100 times with random uncertainties)
- Parametric studies (vary one parameter, plot results)
- Optimization (find best PID gains automatically)
- Comparison with papers (FOK rocket results)

---

**Now go launch some rockets! 🚀**

*Remember: Real rockets are harder than simulations, but simulations help you understand the principles!*