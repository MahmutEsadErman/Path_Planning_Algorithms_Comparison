# Thrust Calibration Guide

## Understanding Thrust Values

### Thrust Range: 0.0 to 1.0 (0% to 100%)

| Thrust Value | Purpose | Expected Behavior |
|--------------|---------|-------------------|
| 0.0 - 0.3    | Too Low | Drone will fall/crash |
| 0.4 - 0.5    | Hover (typical) | Maintains altitude for most drones |
| 0.5 - 0.6    | Light Climb | Slow upward movement |
| 0.6 - 0.7    | Takeoff | Steady climb |
| 0.7 - 0.9    | Fast Climb | Rapid ascent |
| 0.9 - 1.0    | Max Power | Emergency/aggressive maneuvers |

## Quick Testing Procedure

### Step 1: Find Hover Thrust
```bash
# Run the calibration script
cd /ros2_tutorials/python_scripts
python3 thrust_calibration.py hover

# Watch the logs:
# - "CLIMBING ↑" → reduce thrust
# - "DESCENDING ↓" → increase thrust  
# - "HOVERING ✓" → perfect!
```

### Step 2: Calculate Takeoff Thrust
```
Takeoff Thrust = Hover Thrust + 0.15 to 0.20

Example: 
- Hover = 0.50
- Takeoff = 0.65 to 0.70
```

### Step 3: Calculate Forward Flight Thrust
```
Forward Flight Thrust = Hover Thrust × (1/cos(pitch_angle))

For 15° pitch:
- Hover = 0.50
- Forward = 0.50 / cos(15°) = 0.518 ≈ 0.52

Rule of thumb: Hover + 0.05 to 0.10
```

## Physics Explanation

### Why Forward Flight Needs More Thrust

When pitched forward by angle θ:
- Vertical thrust component = Total Thrust × cos(θ)
- Horizontal thrust component = Total Thrust × sin(θ)

To maintain altitude:
```
Total Thrust × cos(θ) = Weight
Total Thrust = Weight / cos(θ)
```

Examples:
- 0° pitch: multiplier = 1.00 (no extra thrust)
- 15° pitch: multiplier = 1.035 (+3.5% thrust)
- 30° pitch: multiplier = 1.155 (+15.5% thrust)
- 45° pitch: multiplier = 1.414 (+41.4% thrust)

## Practical Testing Values

### Conservative Starting Values (Safe for Testing)
```python
THRUST_HOVER = 0.50      # Adjust based on your drone
THRUST_TAKEOFF = 0.65    # For gentle climb
THRUST_LANDING = 0.40    # Controlled descent
THRUST_FORWARD_15DEG = 0.52  # 15° pitch forward
THRUST_FORWARD_30DEG = 0.58  # 30° pitch forward
```

### Factors That Affect Thrust Requirements

1. **Drone Weight**
   - Heavier = more thrust needed
   - Adding payload? Increase thrust!

2. **Battery Level**
   - Low battery = reduced motor performance
   - May need higher thrust values as battery drains

3. **Altitude/Air Density**
   - Higher altitude = thinner air = more thrust needed
   - Outdoor wind = may need thrust adjustments

4. **Motor/Propeller Configuration**
   - Different setups have different efficiency curves

## Using the Calibration Script

### Test 1: Find Hover Thrust
```bash
# Start the script
python3 thrust_calibration.py hover

# In another terminal, arm and takeoff manually
# Then switch to OFFBOARD mode
# Observe the logs and adjust thrust_value in the code
```

### Test 2: Test Forward Flight
```bash
# Use your found hover thrust
python3 thrust_calibration.py forward 0.50 15

# This tests 15° pitch with thrust calculated from hover thrust
```

## Safety Tips

1. **Always start with lower thrust** and increase gradually
2. **Test in a safe, open area** with no obstacles
3. **Have RC transmitter ready** for manual takeover
4. **Monitor battery levels** - behavior changes as battery drains
5. **Start with small pitch angles** (5-10°) for forward flight
6. **Keep logs** of what works for your specific drone

## Monitoring Commands

### Check altitude and velocity in real-time:
```bash
# Local position (altitude)
ros2 topic echo /mavros/local_position/pose

# Velocity
ros2 topic echo /mavros/local_position/velocity_local

# Battery state
ros2 topic echo /mavros/battery
```

## Quick Diagnostic

**Drone is climbing:** Thrust too high → reduce by 0.05  
**Drone is falling:** Thrust too low → increase by 0.05  
**Drone oscillates up/down:** Thrust is close! → fine-tune by 0.01  
**Drone drifts forward while "hovering":** Pitch offset or center of gravity issue

## Example Flight Sequence

```python
# 1. Takeoff to 3 meters
thrust = 0.65  # Higher than hover
pitch = 0°
duration = 4 seconds

# 2. Hover and stabilize  
thrust = 0.50  # Hover thrust
pitch = 0°
duration = 3 seconds

# 3. Move forward
thrust = 0.52  # Slightly higher
pitch = 15°
duration = 5 seconds

# 4. Stop and hover
thrust = 0.50
pitch = 0°
duration = 3 seconds

# 5. Land
thrust = 0.40  # Lower than hover
pitch = 0°
duration = until ground contact
```

## Notes

- These values are **starting points** - every drone is different!
- Document your drone's optimal values for future reference
- Consider creating a config file with your drone's calibrated values
- Test incrementally - don't jump from 0.5 to 0.9!
