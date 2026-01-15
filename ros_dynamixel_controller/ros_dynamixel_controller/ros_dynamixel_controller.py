import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

from .dynamixel_controller import Dynamixel
from .dynamixel_address_book import DYNA_TO_AMP, DYNA_TO_REV_PER_MIN


class RosDynamixelController(Node):
    def __init__(self):
        super().__init__("ros_dynamixel_controller")

        self.motors_id = [11]
        self.motor_series = ["xm"]
        self.num_motors = len(self.motors_id)

        # Parameters
        self._declare_parameters()
        self._setup_communication()
        self._setup_publishers_subscribers()

        # Timer
        self.timer_period = 1.0 / 80.0  # 80Hz
        self.state_timer = self.create_timer(self.timer_period, self._state_callback)
        self.motor_timer = self.create_timer(self.timer_period, self._motor_callback)

    def _declare_parameters(self):
        """Declare and get all ROS parameters"""

        self.desired_current_topic = self.declare_parameter("desired_current_topic", "/desired_current").get_parameter_value().string_value
        self.measured_velocity_topic = (
            self.declare_parameter("measured_velocity_topic", "/measured_velocity").get_parameter_value().string_value
        )
        self.measured_current_topic = (
            self.declare_parameter("measured_current_topic", "/measured_current").get_parameter_value().string_value
        )

    def _setup_communication(self):
        """Initialize communication with Dynamixel servos"""
        self.get_logger().info("Setting up communication with Dynamixel servos...")
        self.servo = Dynamixel(
            ID=self.motors_id,
            descriptive_device_name="DYNAMIXEL",
            port_name="/dev/ttyUSB0",
            baudrate=1000000,
            series_name=self.motor_series,
        )
        self.servo.begin_communication()
        self.servo.disable_torque(False, ID="all")
        self.servo.set_current_limit(1000, ID="all")
        self.servo.set_operating_mode("current", ID="all")
        self.servo.enable_torque(False, ID="all")
        self.get_logger().info("Communication with Dynamixel servos established.")

    def _setup_publishers_subscribers(self):
        """Create all publishers and subscribers"""
        self.desired_current_publisher = self.create_publisher(Float32MultiArray, self.desired_current_topic, 10)
        self.measured_velocity_publisher = self.create_publisher(Float32MultiArray, self.measured_velocity_topic, 10)
        self.measured_current_publisher = self.create_publisher(Float32MultiArray, self.measured_current_topic, 10)

    def _send_desired_current_ma(self, currents: list[float]):
        """Send current commands to Dynamixel motors"""
        desired_current = [currents[i] / DYNA_TO_AMP for i in range(self.num_motors)]
        self.servo.write_current(desired_current, "all")

    def _publish_desired_current(self, currents: list[float]):
        """Publish desired current commands"""
        desired_current_msg = Float32MultiArray()
        desired_current_msg.data = currents
        self.desired_current_publisher.publish(desired_current_msg)

    def _motor_callback(self):
        """Motor commands"""
        desired_current = [0.0 for _ in range(self.num_motors)]
        self._send_desired_current_ma(desired_current)
        self._publish_desired_current(desired_current)

    def _state_callback(self):
        """Timer callback for reading and publishing motor states"""
        # Read and process wheel velocities
        measured_velocity_raw = self.servo.read_velocity("all")

        # Apply direction and scaling
        measured_velocity = [vel * DYNA_TO_REV_PER_MIN * 2 * np.pi * self.WHEEL_RADIUS for vel in measured_velocity_raw]

        # Publish measured velocity
        measured_velocity_msg = Float32MultiArray()
        measured_velocity_msg.data = measured_velocity
        self.measured_velocity_publisher.publish(measured_velocity_msg)

        # Read and publish current consumption
        wheel_current = self.servo.read_current("all")
        if wheel_current:
            wheel_current = [curr * DYNA_TO_AMP for curr in wheel_current]
            current_msg = Float32MultiArray()
            current_msg.data = wheel_current
            self.measured_current_publisher.publish(current_msg)

    def __del__(self):
        """Cleanup on destruction"""
        if hasattr(self, "servo"):
            self.servo.disable_torque(ID="all")


def main(args=None):
    rclpy.init(args=args)
    controller = RosDynamixelController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
