import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

from .dynamixel_controller import Dynamixel
from .dynamixel_address_book import DYNA_TO_AMP, DYNA_TO_DEGREE, DYNA_TO_REV_PER_MIN, DYNA_TO_TEMPERATURE, DYNA_TO_VOLTAGE


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
        self.timer_period: float = 1.0 / 80.0  # 80Hz
        self.state_timer = self.create_timer(self.timer_period, self._state_callback)
        self.motor_timer = self.create_timer(self.timer_period, self._motor_callback)

    def _declare_parameters(self):
        """Declare and get all ROS parameters"""

        self.desired_position_topic = self.declare_parameter("desired_position_topic", "/desired_position_rad").get_parameter_value().string_value
        self.measured_position_topic = self.declare_parameter("measured_position_topic", "/measured_position_rad").get_parameter_value().string_value
        self.measured_velocity_topic = self.declare_parameter("measured_velocity_topic", "/measured_velocity_rad_per_sec").get_parameter_value().string_value
        self.measured_current_topic = self.declare_parameter("measured_current_topic", "/measured_current_amp").get_parameter_value().string_value
        self.measured_voltage_topic = self.declare_parameter("measured_voltage_topic", "/measured_voltage_volt").get_parameter_value().string_value
        self.measured_temperature_topic = self.declare_parameter("measured_temperature_topic", "/measured_temperature_celsius").get_parameter_value().string_value

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
        self.servo.set_current_limit(100, ID="all") # Set current limit to 100mA, change for real tests
        self.servo.set_position_pid(800, 0, 200, ID="all") # Set position PID gains
        self.servo.set_profile_velocity(0, ID="all") # 0 implifes infinite velocity
        self.servo.set_profile_acceleration(0, ID="all") # 0 implies infinite acceleration
        self.servo.set_operating_mode("position", ID="all")
        self.servo.enable_torque(False, ID="all")
        self.get_logger().info("Communication with Dynamixel servos established.")

    def _setup_publishers_subscribers(self):
        """Create all publishers and subscribers"""
        self.desired_position_publisher = self.create_publisher(Float32MultiArray, self.desired_position_topic, 10)
        self.measured_position_publisher = self.create_publisher(Float32MultiArray, self.measured_position_topic, 10)
        self.measured_velocity_publisher = self.create_publisher(Float32MultiArray, self.measured_velocity_topic, 10)
        self.measured_current_publisher = self.create_publisher(Float32MultiArray, self.measured_current_topic, 10)
        self.measured_voltage_publisher = self.create_publisher(Float32MultiArray, self.measured_voltage_topic, 10)
        self.measured_temperature_publisher = self.create_publisher(Float32MultiArray, self.measured_temperature_topic, 10)

    def _send_position(self, positions: list[float]):
        """Send position commands to Dynamixel motors"""
        desired_position = [positions[i] / DYNA_TO_AMP for i in range(self.num_motors)]
        self.servo.write_position(desired_position, "all")

    def _publish_desired_position(self, positions: list[float]):
        """Publish desired position commands"""
        desired_position_msg = Float32MultiArray()
        desired_position_msg.data = positions
        self.desired_position_publisher.publish(desired_position_msg)

    def _motor_callback(self):
        """Motor commands"""
        desired_position = [0.0 for _ in range(self.num_motors)]
        self._send_position(desired_position)
        self._publish_desired_position(desired_position)

    def _state_callback(self):
        """Timer callback for reading and publishing motor states"""
        measured_position_raw = self.servo.read_position("all")
        measured_position = [pos * DYNA_TO_DEGREE / 360.0 * 2 * np.pi for pos in measured_position_raw]
        measured_position_msg = Float32MultiArray()
        measured_position_msg.data = measured_position
        self.measured_position_publisher.publish(measured_position_msg)

        measured_velocity_raw = self.servo.read_velocity("all")
        measured_velocity = [vel * DYNA_TO_REV_PER_MIN / 60 * 2 * np.pi for vel in measured_velocity_raw]
        measured_velocity_msg = Float32MultiArray()
        measured_velocity_msg.data = measured_velocity
        self.measured_velocity_publisher.publish(measured_velocity_msg)

        measured_current_raw = self.servo.read_current("all")
        measured_current = [curr * DYNA_TO_AMP for curr in measured_current_raw]
        measured_current_msg = Float32MultiArray()
        measured_current_msg.data = measured_current
        self.measured_current_publisher.publish(measured_current_msg)

        measured_voltage_raw = self.servo.read_voltage("all")
        measured_voltage = [volt * DYNA_TO_VOLTAGE for volt in measured_voltage_raw]
        measured_voltage_msg = Float32MultiArray()
        measured_voltage_msg.data = measured_voltage
        self.measured_voltage_publisher.publish(measured_voltage_msg)

        measured_temperature_raw = self.servo.read_temperature("all")
        measured_temperature = [temp * DYNA_TO_TEMPERATURE for temp in measured_temperature_raw]
        measured_temperature_msg = Float32MultiArray()
        measured_temperature_msg.data = measured_temperature
        self.measured_temperature_publisher.publish(measured_temperature_msg)

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
