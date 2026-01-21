import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
import numpy as np

from .dynamixel_address_book import (
    ADDR_PROFILE_VELOCITY,
    ADDR_PROFILE_ACCELERATION,
    DYNA_REVERSE_MODE,
    DYNA_TO_AMP,
    DYNA_TO_DEGREE,
    DYNA_TO_REV_PER_MIN,
    DYNA_TO_TEMPERATURE,
    DYNA_TO_VOLTAGE,
)

from dynamixel_easy_sdk import Connector, OperatingMode, Direction


class RosDynamixelController(Node):
    def __init__(self):
        super().__init__("ros_dynamixel_controller")

        self._desired_position_degrees = 0.0
        self._measured_position_rad = 0.0
        self._p_gain = 0.01

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
        self.joystick_topic = self.declare_parameter("joystick_topic", "/joy").get_parameter_value().string_value
        self.desired_position_topic = (
            self.declare_parameter("desired_position_topic", "/desired_position_rad").get_parameter_value().string_value
        )
        self.desired_current_topic = (
            self.declare_parameter("desired_current_topic", "/desired_current_amp").get_parameter_value().string_value
        )
        self.measured_position_topic = (
            self.declare_parameter("measured_position_topic", "/measured_position_rad").get_parameter_value().string_value
        )
        self.measured_velocity_topic = (
            self.declare_parameter("measured_velocity_topic", "/measured_velocity_rad_per_sec").get_parameter_value().string_value
        )
        self.measured_current_topic = (
            self.declare_parameter("measured_current_topic", "/measured_current_amp").get_parameter_value().string_value
        )

    def _setup_communication(self):
        """Initialize communication with Dynamixel servos"""
        self.get_logger().info("Setting up communication with Dynamixel servos...")

        id = 13

        self._connector = Connector("/dev/ttyUSB0", 1000000)
        self._motor = self._connector.createMotor(id)
        self._motor.disableTorque()
        # self._motor.setOperatingMode(OperatingMode.EXTENDED_POSITION)
        self._motor.setOperatingMode(OperatingMode.CURRENT)
        self._motor.setCurrentLimit(int(0.1 / DYNA_TO_AMP))  # Set current limit to 0.1A
        self._motor.setPositionPGain(50)  # Set position P gain
        self._motor.setPositionIGain(0)  # Set position I gain
        self._motor.setPositionDGain(0)  # Set position D gain
        self._motor.setVelocityPGain(0)  # Set velocity P gain
        self._motor.setVelocityIGain(0)  # Set velocity I gain
        # self._motor.setDirection(Direction.REVERSE)  # NORMAL or REVERSE
        self._motor.setVelocityLimit(1023)  # Set velocity limit to max
        self._motor._writeData(id, ADDR_PROFILE_VELOCITY, 4, 0)  # Set profile velocity to 0 (infinite)
        self._motor._writeData(id, ADDR_PROFILE_ACCELERATION, 4, 0)  # Set profile acceleration to 0 (infinite)
        self._motor._writeData(id, 88, 2, 0)  # Feedforward 1 gain to 0
        self._motor._writeData(id, 90, 2, 0)  # Feedforward 2 gain to 0
        self._motor.enableTorque()

        self._position_homing_offset = self._motor.getPresentPosition()

        self.get_logger().info("Communication with Dynamixel servos established.")

    def _setup_publishers_subscribers(self):
        """Create all publishers and subscribers"""
        self.joystick_subscription = self.create_subscription(Joy, self.joystick_topic, self._joystick_callback, 10)
        self.desired_position_publisher = self.create_publisher(Float32, self.desired_position_topic, 10)
        self.desired_current_publisher = self.create_publisher(Float32, self.desired_current_topic, 10)
        self.measured_position_publisher = self.create_publisher(Float32, self.measured_position_topic, 10)
        self.measured_velocity_publisher = self.create_publisher(Float32, self.measured_velocity_topic, 10)
        self.measured_current_publisher = self.create_publisher(Float32, self.measured_current_topic, 10)

    def _joystick_callback(self, msg: Joy):
        """Translate joystick input to desired position commands"""
        self._desired_position_degrees = msg.axes[0] * 360
        self._publish_desired_position(self._desired_position_degrees / 180.0 * np.pi)
        # self._desired_current_amps = -1.0 * msg.axes[0] * 0.05  # Max 0.05A
        # self._publish_desired_current(self._desired_current_amps)

    def _send_position(self, position: float):
        """Send position commands to Dynamixel motors"""
        desired_position = int(position / DYNA_TO_DEGREE)
        desired_position += self._position_homing_offset
        self._motor.setGoalPosition(desired_position)

    def _send_current(self, current: float):
        """Send current commands to Dynamixel motors"""
        desired_current = int(current / DYNA_TO_AMP)
        # if desired_current > 0.01:
        #     if not self._motor.isTorqueOn():
        #         self._motor.enableTorque()
        #     self._motor.setGoalCurrent(desired_current)
        # else:
        #     if self._motor.isTorqueOn():
        #         self._motor.disableTorque()
        self._motor.setGoalCurrent(desired_current)

    def _publish_desired_position(self, position: float):
        """Publish desired position commands"""
        desired_position_msg = Float32()
        desired_position_msg.data = position
        self.desired_position_publisher.publish(desired_position_msg)

    def _publish_desired_current(self, current: float):
        """Publish desired current commands"""
        desired_current_msg = Float32()
        desired_current_msg.data = current
        self.desired_current_publisher.publish(desired_current_msg)

    def _motor_callback(self):
        """Motor commands"""
        # self._send_position(self._desired_position_degrees)
        measured_position_raw = self._motor.getPresentPosition() - self._position_homing_offset
        self._measured_position_rad = measured_position_raw * DYNA_TO_DEGREE / 360.0 * 2 * np.pi
        desired_position_rads = self._desired_position_degrees / 180.0 * np.pi
        self._p_gain = 0.01
        self._desired_current_amps = self._p_gain * (desired_position_rads - self._measured_position_rad)
        self._publish_desired_current(self._desired_current_amps)
        self._send_current(self._desired_current_amps)

    def _state_callback(self):
        """Timer callback for reading and publishing motor states"""
        measured_position_msg = Float32()
        measured_position_msg.data = self._measured_position_rad
        self.measured_position_publisher.publish(measured_position_msg)

        measured_velocity_raw = self._motor.getPresentVelocity()
        measured_velocity = measured_velocity_raw * DYNA_TO_REV_PER_MIN / 60 * 2 * np.pi
        measured_velocity_msg = Float32()
        measured_velocity_msg.data = measured_velocity
        self.measured_velocity_publisher.publish(measured_velocity_msg)

        measured_current_raw = self._motor.getPresentCurrent()
        measured_current = measured_current_raw * DYNA_TO_AMP
        measured_current_msg = Float32()
        measured_current_msg.data = measured_current
        self.measured_current_publisher.publish(measured_current_msg)

    def __del__(self):
        """Cleanup on destruction"""
        self._connector.close()


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
