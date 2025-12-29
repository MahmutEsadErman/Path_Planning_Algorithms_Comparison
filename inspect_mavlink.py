import inspect
from pymavlink import mavutil

mav = mavutil.mavlink.MAVLink(None)
print(inspect.signature(mav.set_gps_global_origin_encode))
