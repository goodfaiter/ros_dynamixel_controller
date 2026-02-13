import enum
import math
import time
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
    DYNA_TO_PWM,
    DYNA_TO_REV_PER_MIN,
    DYNA_TO_TEMPERATURE,
    DYNA_TO_VOLTAGE,
)

from dynamixel_easy_sdk import Connector, OperatingMode, Direction
from .scanner import scan_dynamixel_motors
from .trajectory_generator import RampGenerator, RandomGenerator, StepGenerator, SineGenerator
from .rl_controller import RLController


class Mode(enum.Enum):
    MANUAL = 0
    RANDOM = 1
    STEP = 2
    RAMP = 3
    ZERO = 4
    NA = 5
    SINE = 6
    RL = 7


class RosDynamixelController(Node):
    def __init__(self):
        super().__init__("ros_dynamixel_controller")

        # scan_dynamixel_motors(self.get_logger())
        # return

        self.controller = RLController(device="cpu")
        self.controller.load("/colcon_ws/src/ros_dynamixel_controller/models/latest_player.pt")
        self.controller.init()
        self.controller.reset()

        self._desired_position_degrees = 0.0
        self._prev_desired_position_degrees: float = 0.0
        self._measured_position_rad = 0.0
        self._measured_velocity_rad_per_sec = 0.0
        self._p_gain = 0.01
        self._desired_ee_angle_rad = 0.0
        self._desired_position_delta_rad = 0.0
        self.radius = 0.0125  # radius of the tendon pulley

        # PD
        self._position_error = 0.0

        self._random_generator = RandomGenerator(0.0, 2.0 * np.pi)
        self._step_generator = StepGenerator(step_size=0.5)
        # self._ramp_generator = RampGenerator(min_step_size=0.005, max_step_size=0.03)
        self._ramp_generator = RampGenerator(min_step_size=0.01, max_step_size=0.05)
        # self._sine_generator = SineGenerator(amplitude=np.pi, min_freq = 0.05, max_freq=0.3, phase=1.5 * np.pi)
        self._sine_generator = SineGenerator(amplitude=np.pi, min_freq = 1.0, max_freq=2.0, phase=1.5 * np.pi)

        self._mode: Mode = Mode.MANUAL
        self._joy_ready = False
        self._latest_joy: Joy = None

        self._joy_call_time = time.time()
        self._switch: bool = False

        # Parameters
        self._declare_parameters()
        self._setup_communication()
        self._setup_publishers_subscribers()

        # Timer
        self.control_timer = self.create_timer(1.0 / 80.0, self._control_callback)
        self.state_timer = self.create_timer(1.0 / 80.0, self._state_callback)
        self.motor_timer = self.create_timer(1.0 / 20.0, self._motor_callback)
        self.rl_timer = self.create_timer(1.0 / 20.0, self._rl_callback)

    def _declare_parameters(self):
        """Declare and get all ROS parameters"""
        self.joystick_topic = self.declare_parameter("joystick_topic", "/joy").get_parameter_value().string_value
        self.desired_position_topic = (
            self.declare_parameter("desired_position_topic", "/desired_position_rad").get_parameter_value().string_value
        )
        self.desired_current_topic = (
            self.declare_parameter("desired_current_topic", "/desired_current_amp").get_parameter_value().string_value
        )
        self.desired_pwm_topic = self.declare_parameter("desired_pwm_topic", "/desired_pwm_percentage").get_parameter_value().string_value
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

    def _setup_communication(self):
        """Initialize communication with Dynamixel servos"""
        self.get_logger().info("Setting up communication with Dynamixel servos...")

        id = 1 # 4 for small, 13 for big, 1 for mid

        self._connector = Connector("/dev/ttyUSB0", 1000000)
        self._motor = self._connector.createMotor(id)
        self._motor.disableTorque()
        # self._motor.setOperatingMode(OperatingMode.EXTENDED_POSITION)
        self._motor.setOperatingMode(OperatingMode.PWM)
        self._motor.setPWMLimit(int(100 / DYNA_TO_PWM))  # Set PWM limit to ~100%
        # self._motor.setOperatingMode(OperatingMode.CURRENT)
        # self._motor.setCurrentLimit(int(1.0 / DYNA_TO_AMP))  # Set current limit to 1.0A
        # self._motor.setPositionPGain(50)  # Set position P gain
        # self._motor.setPositionIGain(0)  # Set position I gain
        # self._motor.setPositionDGain(0)  # Set position D gain
        # self._motor.setVelocityPGain(0)  # Set velocity P gain
        # self._motor.setVelocityIGain(0)  # Set velocity I gain
        # self._motor.setDirection(Direction.REVERSE)  # NORMAL or REVERSE
        # self._motor.setVelocityLimit(1023)  # Set velocity limit to max
        self._motor._writeData(id, ADDR_PROFILE_VELOCITY, 4, 0)  # Set profile velocity to 0 (infinite)
        self._motor._writeData(id, ADDR_PROFILE_ACCELERATION, 4, 0)  # Set profile acceleration to 0 (infinite)
        self._motor._writeData(id, 88, 2, 0)  # Feedforward 1 gain to 0
        self._motor._writeData(id, 90, 2, 0)  # Feedforward 2 gain to 0
        self._motor._writeData(id, 9, 1, 0) # set return delay to 9
        # self._motor._writeData(id, 8, 1, 3) # set baud rate to 1M

        # mode_value = self._motor._readData(id, 10, 1)
        # GOAL_BIT = 0b00000100
        # mode_value &= ~GOAL_BIT
        # self._motor._writeData(id, 10, 1, mode_value)  # Set torque on on goal
        self._motor.enableTorque()

        self._position_homing_offset = self._motor.getPresentPosition()

        self.get_logger().info("Communication with Dynamixel servos established.")

    def _setup_publishers_subscribers(self):
        """Create all publishers and subscribers"""
        self.joystick_subscription = self.create_subscription(Joy, self.joystick_topic, self._joystick_callback, 10)
        self.desired_ee_angle_publisher = self.create_publisher(Float32, "/desired_ee_angle_rad", 10)
        self.desired_position_publisher = self.create_publisher(Float32, self.desired_position_topic, 10)
        self.desired_current_publisher = self.create_publisher(Float32, self.desired_current_topic, 10)
        self.desired_pwm_publisher = self.create_publisher(Float32, self.desired_pwm_topic, 10)
        self.measured_position_publisher = self.create_publisher(Float32, self.measured_position_topic, 10)
        self.measured_velocity_publisher = self.create_publisher(Float32, self.measured_velocity_topic, 10)
        self.measured_current_publisher = self.create_publisher(Float32, self.measured_current_topic, 10)
        self.measured_pwm_publisher = self.create_publisher(Float32, self.measured_pwm_topic, 10)
        self.measured_position_error_publisher = self.create_publisher(Float32, self.measured_position_error_topic, 10)

    def _joystick_callback(self, msg: Joy):
        """Store latest joystick message"""
        # self.get_logger().info(f"Joystick axes: {msg.axes}, buttons: {msg.buttons}")
        # if self._joy_ready:
        #     return

        # if msg.axes[0] > 0.05 or msg.axes[0] < -0.05 or msg.buttons[0] == 1 or msg.buttons[2] == 1 or msg.buttons[3] == 1:
        self._latest_joy = msg

        # print(msg)

        # X was pressed
        if msg.buttons[0] == 1:
            if time.time() - self._joy_call_time > 0.5:
                self._random_generator.advance()
                self._mode = Mode.RANDOM
                self._joy_call_time = time.time()
        # Circl was pressed
        elif msg.buttons[1] == 1:
            self._sine_generator.advance()
            self._mode = Mode.SINE
        # Square was pressed
        elif msg.buttons[3] == 1:
            if time.time() - self._joy_call_time > 0.5:
                self._step_generator.advance()
                self._mode = Mode.STEP
                self._joy_call_time = time.time()
        # Triangle was pressed
        elif msg.buttons[2] == 1:
            self._ramp_generator.advance()
            self._mode = Mode.RAMP
        # Option was pressed
        elif msg.buttons[9] == 1:
            self._mode = Mode.ZERO
        # Right joystick moved
        elif msg.axes[3] > 0.01:
            self._mode = Mode.RL
        # Joystick moved
        elif msg.axes[0] > 0.01 or msg.axes[0] < -0.01:
            self._mode = Mode.MANUAL
        # else:
        #     self._mode = Mode.MANUAL
            # self._mode = Mode.ZERO

    def _send_position(self, position: float):
        """Send position commands to Dynamixel motors"""
        desired_position = int(position / DYNA_TO_DEGREE)
        desired_position += self._position_homing_offset
        self._motor.setGoalPosition(desired_position)

    def _send_current(self, current: float):
        """Send current commands to Dynamixel motors"""
        desired_current = int(current / DYNA_TO_AMP)
        self._motor.setGoalCurrent(desired_current)

    def _send_pwm(self, pwm_percentage: float):
        """Send PWM commands to Dynamixel motors"""
        desired_pwm = int(pwm_percentage / DYNA_TO_PWM)
        self._motor.setGoalPWM(desired_pwm)

    def _publish_desired_ee_angle(self, angle_rad: float):
        """Publish desired end-effector angle commands"""
        desired_ee_angle_msg = Float32()
        desired_ee_angle_msg.data = angle_rad
        self.desired_ee_angle_publisher.publish(desired_ee_angle_msg)

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

    def _publish_desired_pwm(self, pwm_percentage: float):
        """Publish desired PWM commands"""
        desired_pwm_msg = Float32()
        desired_pwm_msg.data = pwm_percentage
        self.desired_pwm_publisher.publish(desired_pwm_msg)

    def _publish_position_error(self, position_error: float):
        """Publish desired PWM commands"""
        measured_position_error_msg = Float32()
        measured_position_error_msg.data = position_error
        self.measured_position_error_publisher.publish(measured_position_error_msg)

    def _control_callback(self):
        """Timer callback for controlling the motor"""
        # if not self._joy_ready:
        #     return
        if self._latest_joy is None:
            return

        msg = self._latest_joy

        measured_position_raw = self._motor.getPresentPosition() - self._position_homing_offset
        self._measured_position_rad = measured_position_raw * DYNA_TO_DEGREE / 360.0 * 2 * np.pi

        if self._mode == Mode.RANDOM:
            self._desired_position_degrees = self._random_generator.generate() * 180.0 / np.pi
        elif self._mode == Mode.STEP:
            self._desired_position_degrees = self._step_generator.generate() * 180.0 / np.pi
        elif self._mode == Mode.RAMP:
            self._desired_position_degrees = self._ramp_generator.generate() * 180.0 / np.pi
        elif self._mode == Mode.SINE:
            self._desired_position_degrees = self._sine_generator.generate() * 180.0 / np.pi
        elif self._mode == Mode.ZERO:
            self._desired_position_degrees = 0.0
        elif self._mode == Mode.MANUAL:
            self._desired_position_degrees = msg.axes[0] * 360
        elif self._mode == Mode.RL:
            # x = msg.axes[4]
            # y = msg.axes[3]
            # vec = np.array([x, y])
            # unit_vec = vec / (np.linalg.norm(vec) + 1e-6)
            # sine = msg.axes[4]
            # cosine = msg.axes[3]
            # self._desired_ee_angle_rad = np.arctan2(unit_vec[0], unit_vec[1])
            # print(f"Desired EE angle (rad): {self._desired_ee_angle_rad:.2f}")
            # self.get_logger().info(f"Joystick axes: {self.controller.action_buffer.buffer}")
            self._desired_ee_angle_rad = np.clip(msg.axes[3] * 0.5 * np.pi, 0.0, None)
            desired_position_rads = np.clip(self._measured_position_rad + self._desired_position_delta_rad, 0.0, 2 * np.pi)
            self._desired_position_degrees = desired_position_rads * 180.0 / np.pi
        else:
            self._desired_position_degrees = self._measured_position_rad * 180.0 / np.pi

        self._publish_desired_ee_angle(self._desired_ee_angle_rad)
        self._publish_desired_position(self._desired_position_degrees / 180.0 * np.pi)

    def _motor_callback(self):
        """Motor commands"""

        # Control law
        desired_position_rads = self._desired_position_degrees / 180.0 * np.pi
        self._position_error = desired_position_rads - self._measured_position_rad
        self._publish_position_error(self._position_error)

        # strong motor settings
        self._p_gain = 0.1
        self._max_pwm = 90

        if self._position_error > 0.0:
            self._p_gain = 0.1
        else:
            self._p_gain = 0.4

        self._desired_pwm_percentage = 100.0 * self._p_gain * self._position_error
        self._desired_pwm_percentage = np.clip(self._desired_pwm_percentage, -self._max_pwm, self._max_pwm)
        # self._send_position(self._desired_position_degrees)
        
        # self._p_gain = 0.15
        # self._desired_current_amps = self._p_gain * (desired_position_rads - self._measured_position_rad)
        # self._publish_desired_current(self._desired_current_amps)
        # self._send_current(self._desired_current_amps)

        ## weak motor settings
        # self._p_gain = 0.2
        # self._max_pwm = 90
        self._publish_desired_pwm(self._desired_pwm_percentage)
        self._send_pwm(self._desired_pwm_percentage)

    def _state_callback(self):
        """Timer callback for reading and publishing motor states"""
        measured_position_msg = Float32()
        measured_position_msg.data = self._measured_position_rad
        self.measured_position_publisher.publish(measured_position_msg)

        measured_velocity_raw = self._motor.getPresentVelocity()
        self._measured_velocity_rad_per_sec = measured_velocity_raw * DYNA_TO_REV_PER_MIN / 60 * 2 * np.pi
        measured_velocity_msg = Float32()
        measured_velocity_msg.data = self._measured_velocity_rad_per_sec
        self.measured_velocity_publisher.publish(measured_velocity_msg)

        measured_current_raw = self._motor.getPresentCurrent()
        measured_current = measured_current_raw * DYNA_TO_AMP
        measured_current_msg = Float32()
        measured_current_msg.data = measured_current
        self.measured_current_publisher.publish(measured_current_msg)

        measured_pwm_raw = self._motor.getPresentPWM()
        measured_pwm = measured_pwm_raw * DYNA_TO_PWM
        measured_pwm_msg = Float32()
        measured_pwm_msg.data = measured_pwm
        self.measured_pwm_publisher.publish(measured_pwm_msg)

    def _rl_callback(self):
        self.controller.update(self._measured_position_rad, self._measured_velocity_rad_per_sec, self._desired_ee_angle_rad)
        self._desired_position_delta_rad = self.controller.forward()

    def __del__(self):
        """Cleanup on destruction"""
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
