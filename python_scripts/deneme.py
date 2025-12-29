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

# Prepare message parameters
# C++: packet.latitude = 41.0258025 * 1e7;
lat = 410258025
lon = 288884930
alt = 600000
target_system = 0

# Create the SET_GPS_GLOBAL_ORIGIN message
# The encode function creates the message object

master.mav.set_gps_global_origin_send(
    target_system,
    lat,
    lon,
    alt
)
print("Sent SET_GPS_GLOBAL_ORIGIN")

