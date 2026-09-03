import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from .dynamixel_address_book import (
    ADDR_PRESENT_PWM,
    ADDR_PROFILE_VELOCITY,
    ADDR_PROFILE_ACCELERATION,
    DYNA_TO_AMP,
    DYNA_TO_DEGREE,
    DYNA_TO_PWM,
    DYNA_TO_REV_PER_MIN,
)

from dynamixel_easy_sdk import Connector, OperatingMode
from dynamixel_sdk.robotis_def import COMM_SUCCESS
from .scanner import scan_dynamixel_motors

# Bulk read Present PWM (124), Current (126), Velocity (128), Position (132) in one transaction.
_BULK_READ_START_ADDR = ADDR_PRESENT_PWM
_BULK_READ_LENGTH = 12  # 2 + 2 + 4 + 4 bytes


class RosDynamixelController(Node):
    def __init__(self):
        super().__init__("ros_dynamixel_controller")

        self._desired_position_rad = 0.0
        self._measured_position_rad = 0.0
        self._measured_velocity_rad_per_sec = 0.0
        self._position_homing_offset = 0

        # Parameters
        self._declare_parameters()
        self._setup_communication()
        self._setup_publishers_subscribers()

        # Timers
        self.state_timer = self.create_timer(1.0 / 200.0, self._state_callback)
        self.motor_timer = self.create_timer(1.0 / 200.0, self._motor_callback)
        self.reset_timer = self.create_timer(self.reset_delay_sec, self._on_reset_delay_elapsed)

    def _declare_parameters(self):
        """Declare and get all ROS parameters"""
        self.desired_position_topic = (
            self.declare_parameter("desired_position_topic", "/desired_position_rad").get_parameter_value().string_value
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
        self.measured_pwm_topic = (
            self.declare_parameter("measured_pwm_topic", "/measured_pwm_percentage").get_parameter_value().string_value
        )
        self.measured_position_error_topic = (
            self.declare_parameter("measured_position_error_topic", "/measured_position_error_rad").get_parameter_value().string_value
        )
        self.desired_pwm_topic = (
            self.declare_parameter("desired_pwm_topic", "/desired_pwm_percentage").get_parameter_value().string_value
        )
        self.bota_reset_service_name = (
            self.declare_parameter("bota_reset_service_name", "/bota_node/reset").get_parameter_value().string_value
        )
        self.hx711_reset_service_name = (
            self.declare_parameter("hx711_reset_service_name", "/hx711_node/reset").get_parameter_value().string_value
        )
        self.reset_delay_sec = (
            self.declare_parameter("reset_delay_sec", 2.0).get_parameter_value().double_value
        )

    def _setup_communication(self):
        """Initialize communication with Dynamixel servos"""
        self.get_logger().info("Setting up communication with Dynamixel servos...")

        id = 1  # 4 for small, 13 for big, 1 for mid

        self._connector = Connector("/dev/ttyUSB0", 1000000)
        self._motor = self._connector.createMotor(id)
        self._motor.disableTorque()
        self._motor.setOperatingMode(OperatingMode.PWM)
        self._motor.setPWMLimit(int(100 / DYNA_TO_PWM))  # Set PWM limit to ~100%
        self._motor._writeData(id, ADDR_PROFILE_VELOCITY, 4, 0)  # Set profile velocity to 0 (infinite)
        self._motor._writeData(id, ADDR_PROFILE_ACCELERATION, 4, 0)  # Set profile acceleration to 0 (infinite)
        self._motor._writeData(id, 88, 2, 0)  # Feedforward 1 gain to 0
        self._motor._writeData(id, 90, 2, 0)  # Feedforward 2 gain to 0
        self._motor._writeData(id, 9, 1, 0)  # set return delay to 0
        self._motor.enableTorque()

        self._position_homing_offset = self._motor.getPresentPosition()

        self.get_logger().info("Communication with Dynamixel servos established.")

    def _reset_sensors(self):
        """Trigger reset on sensor nodes via ROS services.

        This is non-blocking: if a service is not ready, it is skipped. The sensor
        nodes will shut themselves down and be restarted by Docker Compose, which
        re-runs their startup tare sequence.
        """
        self.get_logger().info("Triggering sensor reset services...")

        service_names = {
            "Bota": self.bota_reset_service_name,
            "HX711": self.hx711_reset_service_name,
        }

        for sensor_name, service_name in service_names.items():
            client = self.create_client(Trigger, service_name)
            if not client.service_is_ready():
                self.get_logger().warning(
                    f"{sensor_name} reset service ({service_name}) is not ready; skipping."
                )
                continue

            future = client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda fut, name=sensor_name: self._reset_service_done_callback(fut, name)
            )
            self.get_logger().info(f"{sensor_name} reset triggered via {service_name}.")

    def _reset_service_done_callback(self, future, sensor_name):
        """Log the result of an async reset service call."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{sensor_name} reset acknowledged: {response.message}")
            else:
                self.get_logger().warning(f"{sensor_name} reset reported failure: {response.message}")
        except Exception as e:
            self.get_logger().warning(f"{sensor_name} reset call failed: {e}")

    def _on_reset_delay_elapsed(self):
        """One-shot timer callback: trigger sensor resets after the initial delay."""
        self.reset_timer.cancel()
        self.get_logger().info(
            f"Reset delay of {self.reset_delay_sec} s elapsed; triggering sensor resets."
        )
        self._reset_sensors()

    def _setup_publishers_subscribers(self):
        """Create all publishers and subscribers"""
        self.desired_position_subscription = self.create_subscription(
            Float32, self.desired_position_topic, self._desired_position_callback, 10
        )

        self.measured_position_publisher = self.create_publisher(Float32, self.measured_position_topic, 10)
        self.measured_velocity_publisher = self.create_publisher(Float32, self.measured_velocity_topic, 10)
        self.measured_current_publisher = self.create_publisher(Float32, self.measured_current_topic, 10)
        self.measured_pwm_publisher = self.create_publisher(Float32, self.measured_pwm_topic, 10)
        self.measured_position_error_publisher = self.create_publisher(
            Float32, self.measured_position_error_topic, 10
        )
        self.desired_pwm_publisher = self.create_publisher(Float32, self.desired_pwm_topic, 10)

    def _desired_position_callback(self, msg: Float32):
        """Store latest desired position"""
        self._desired_position_rad = msg.data

    def _send_pwm(self, pwm_percentage: float):
        """Send PWM commands to Dynamixel motors"""
        desired_pwm = int(pwm_percentage / DYNA_TO_PWM)
        self._motor.setGoalPWM(desired_pwm)

    def _motor_callback(self):
        """Motor commands"""
        # Control law
        position_error = self._desired_position_rad - self._measured_position_rad

        measured_position_error_msg = Float32()
        measured_position_error_msg.data = position_error
        self.measured_position_error_publisher.publish(measured_position_error_msg)

        # strong motor settings
        p_gain = 0.1
        max_pwm = 90

        desired_pwm_percentage = 100.0 * p_gain * position_error
        desired_pwm_percentage = np.clip(desired_pwm_percentage, -max_pwm, max_pwm)
        desired_pwm_percentage += 2.0

        self._publish_desired_pwm(desired_pwm_percentage)
        self._send_pwm(desired_pwm_percentage)

    def _publish_desired_pwm(self, pwm_percentage: float):
        """Publish desired PWM commands"""
        desired_pwm_msg = Float32()
        desired_pwm_msg.data = pwm_percentage
        self.desired_pwm_publisher.publish(desired_pwm_msg)

    def _read_motor_state(self):
        """Read PWM, Current, Velocity and Position in a single Dynamixel transaction."""
        data, result, error = self._connector._packet_handler.readTxRx(
            self._connector._port_handler,
            self._motor.id,
            _BULK_READ_START_ADDR,
            _BULK_READ_LENGTH,
        )
        if result != COMM_SUCCESS or error != 0:
            self.get_logger().warning(
                f"Bulk motor-state read failed: comm_result={result}, dxl_error={error}"
            )
            return None
        return data

    @staticmethod
    def _parse_motor_state(data):
        """Parse the 12-byte bulk read response."""
        data = bytes(data)
        pwm_raw = int.from_bytes(data[0:2], byteorder="little", signed=True)
        current_raw = int.from_bytes(data[2:4], byteorder="little", signed=True)
        velocity_raw = int.from_bytes(data[4:8], byteorder="little", signed=True)
        position_raw = int.from_bytes(data[8:12], byteorder="little", signed=True)
        return pwm_raw, current_raw, velocity_raw, position_raw

    def _state_callback(self):
        """Timer callback for reading and publishing motor states"""
        data = self._read_motor_state()
        if data is None:
            return

        measured_pwm_raw, measured_current_raw, measured_velocity_raw, measured_position_raw = self._parse_motor_state(
            data
        )

        measured_position_raw -= self._position_homing_offset
        self._measured_position_rad = measured_position_raw * DYNA_TO_DEGREE / 360.0 * 2 * math.pi

        measured_position_msg = Float32()
        measured_position_msg.data = self._measured_position_rad
        self.measured_position_publisher.publish(measured_position_msg)

        self._measured_velocity_rad_per_sec = measured_velocity_raw * DYNA_TO_REV_PER_MIN / 60 * 2 * math.pi
        measured_velocity_msg = Float32()
        measured_velocity_msg.data = self._measured_velocity_rad_per_sec
        self.measured_velocity_publisher.publish(measured_velocity_msg)

        measured_current = measured_current_raw * DYNA_TO_AMP
        measured_current_msg = Float32()
        measured_current_msg.data = measured_current
        self.measured_current_publisher.publish(measured_current_msg)

        measured_pwm = measured_pwm_raw * DYNA_TO_PWM
        measured_pwm_msg = Float32()
        measured_pwm_msg.data = measured_pwm
        self.measured_pwm_publisher.publish(measured_pwm_msg)

    def __del__(self):
        """Cleanup on destruction"""
        self._motor.disableTorque()
        self._connector.closePort()


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
