import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion
import math
import time


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (in radians) to a quaternion.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q



class AttitudePublisher(Node):
    """
    This node publishes attitude targets to a MAVROS-enabled vehicle.
    It cycles through a predefined sequence of attitudes.
    """
    def __init__(self):
        super().__init__('attitude_publisher_node')
        self.publisher_ = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        
        # Create a timer to publish messages at 20 Hz
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # --- Parameters ---
        self.thrust_value = 0.5  # IMPORTANT: This must be tuned for your vehicle to hover!
        self.target_roll_deg = 0.0 # Target roll in degrees
        self.target_pitch_deg = 0.0 # Target pitch in degrees
        self.target_yaw_deg = 0.0   # Target yaw in degrees

        # self.move_up()
        # print("Taking off...")
        # time.sleep(3)
        # self.hover()
        # self.move_forward()
        # time.sleep(3)
        # self.hover()
        # self.turn_left()
        # time.sleep(1)
        # self.move_forward()
        # time.sleep(2)
        # self.hover()
        # self.turn_left()
        # time.sleep(1)
        # self.move_forward()
        # time.sleep(2)
        # self.move_down()
        # time.sleep(2)
        self.start = time.time()

    def timer_callback(self):
        if self.start + 5 > time.time():
            self.move_up()
            print("Taking off...")
        elif self.start + 10 > time.time() > self.start + 5:
            self.hover()
            self.move_forward()
            print("Moving forward...")
        elif self.start + 12 > time.time():
            self.turn_left()
            print("Turning left...")
        self.send_attitude(self.target_roll_deg, self.target_pitch_deg, self.target_yaw_deg, self.thrust_value)

    def send_attitude(self, roll_deg, pitch_deg, yaw_deg, thrust):
        """
        Helper function to send a single attitude command.
        """
        
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)
        
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.orientation = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0
        msg.thrust = thrust
        msg.type_mask = 7 # Ignore body rates

        self.publisher_.publish(msg)

    def turn_left(self):
        self.target_yaw_deg = 45.0

    def turn_right(self):
        self.target_yaw_deg = -45.0

    def move_forward(self):
        self.target_pitch_deg = -10.0  # Pitch down to move forward
    
    def move_backward(self):
        self.target_pitch_deg = 10.0   # Pitch up to move backward

    def move_up(self):
        self.thrust_value = 0.8  # Increase thrust to move up

    def move_down(self):
        self.thrust_value = 0.2  # Decrease thrust to move down
    
    def hover(self):
        self.target_pitch_deg = 0.0
        self.thrust_value = 0.5  # Set thrust to hover value

def main(args=None):
    rclpy.init(args=args)
    attitude_publisher = AttitudePublisher()

    try:
        rclpy.spin(attitude_publisher)
    except KeyboardInterrupt:
        print("Shutting down attitude publisher node.")
    finally:
        # Destroy the node explicitly
        attitude_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
