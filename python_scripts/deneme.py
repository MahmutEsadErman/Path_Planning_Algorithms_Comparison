from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()

# Gimbal'ı Earth Frame modunda -90° pitch'e ayarla
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
    0,
    -90,    # pitch (derece) - yere bakıyor
    0,      # yaw (derece)
    0,      # pitch rate (deg/s) - 0 = angle control
    0,      # yaw rate
    0x02,   # flags: 0x02 = GIMBAL_MANAGER_FLAGS_ROLL_LOCK (stabilize roll)
    0,
    mavutil.mavlink.GIMBAL_DEVICE_FLAGS_ROLL_LOCK | 
    mavutil.mavlink.GIMBAL_DEVICE_FLAGS_PITCH_LOCK  # Earth frame stabilizasyon
)