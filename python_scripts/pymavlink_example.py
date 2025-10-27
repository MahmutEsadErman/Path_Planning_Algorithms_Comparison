#!/usr/bin/env python3
"""
Example of sending manual control commands to ArduPilot SITL using pymavlink.
This script connects to SITL and sends joystick/RC-style control inputs.
"""

from pymavlink import mavutil
import time

# Connection string for SITL (default UDP port)
connection_string = 'udp:127.0.0.1:14550'

# Create the connection
print(f"Connecting to vehicle on: {connection_string}")
master = mavutil.mavlink_connection(connection_string)

# Wait for the heartbeat to confirm connection
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

def arm_vehicle():
    """Arm the vehicle."""
    print("Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # param1: 1 to arm
        0, 0, 0, 0, 0, 0  # unused parameters
    )
    
    # Wait for ack
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Vehicle armed successfully")
        return True
    else:
        print("Failed to arm vehicle")
        return False

def set_mode(mode):
    """Set flight mode."""
    # Get mode ID
    if mode not in master.mode_mapping():
        print(f"Unknown mode: {mode}")
        return False
    
    mode_id = master.mode_mapping()[mode]
    print(f"Setting mode to {mode}...")
    
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    
    # Wait for mode change confirmation
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack:
        print(f"Mode set to {mode}")
        return True
    return False

def send_manual_control(x, y, z, r, buttons=0):
    """
    Send manual control command.
    
    Args:
        x: pitch (-1000 to 1000, forward/backward)
        y: roll (-1000 to 1000, left/right)
        z: throttle (0 to 1000)
        r: yaw (-1000 to 1000, rotation)
        buttons: button states (bitmask)
    """
    print(master.target_system)
    master.mav.manual_control_send(
        master.target_system,
        x,  # pitch
        y,  # roll
        z,  # throttle
        r,  # yaw
        buttons  # buttons
    )

def main():
    """Main control loop."""
    
    # Set to STABILIZE mode (or GUIDED for copters)
    set_mode('STABILIZE')
    time.sleep(1)
    
    # Arm the vehicle
    if not arm_vehicle():
        print("Cannot proceed without arming")
        return
    
    time.sleep(2)
    
    print("\nStarting manual control...")
    print("Sending throttle up command for 5 seconds")
    
    # Send throttle up commands for 5 seconds
    start_time = time.time()
    while time.time() - start_time < 5:
        # x=0 (no pitch), y=0 (no roll), z=600 (60% throttle), r=0 (no yaw)
        send_manual_control(0, 0, 600, 0)
        time.sleep(0.1)  # 10Hz update rate
    
    print("Sending neutral commands with reduced throttle")
    
    # Send neutral commands for 3 seconds
    start_time = time.time()
    while time.time() - start_time < 3:
        # Neutral position with 40% throttle
        send_manual_control(0, 0, 400, 0)
        time.sleep(0.1)
    
    print("Sending forward pitch command")
    
    # Send forward pitch for 3 seconds
    start_time = time.time()
    while time.time() - start_time < 3:
        # x=300 (forward pitch), z=500 (50% throttle)
        send_manual_control(300, 0, 500, 0)
        time.sleep(0.1)
    
    print("Returning to neutral")
    
    # Return to neutral
    for _ in range(20):
        send_manual_control(0, 0, 400, 0)
        time.sleep(0.1)
    
    print("\nManual control demonstration complete")
    print("Note: Vehicle will remain armed. Disarm manually or implement disarm command.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nScript interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Closing connection...")