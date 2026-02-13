import numpy as np
import torch


class TimeSeriesBuffer:
    def __init__(self, max_size, device="cpu"):
        self.buffer = torch.zeros(max_size, dtype=torch.float32, device=device)
        self.max_size = max_size

    def add(self, value: float):
        self.buffer = torch.roll(self.buffer, -1, dims=0)
        self.buffer[-1] = value

    def reset(self):
        self.buffer[:] = 0.0


class RLController:
    def __init__(self, device):
        self.device = device
        self.model = None

    def load(self, file_path: str) -> None:
        self.model = torch.load(file_path, weights_only=False, map_location=self.device)

    def init(self) -> None:
        self._num_history = 1
        # self.motor_pos = TimeSeriesBuffer(max_size=self._num_history, device=self.device)
        # self.motor_vel = TimeSeriesBuffer(max_size=self._num_history, device=self.device)
        # self.action_buffer = TimeSeriesBuffer(max_size=self._num_history, device=self.device)
        # self.prev_action_buffer = TimeSeriesBuffer(max_size=self._num_history, device=self.device)
        # self.angle_buffer = TimeSeriesBuffer(max_size=self._num_history, device=self.device)
        # self._input = torch.zeros(1, self._num_history * 5, dtype=torch.float32, device=self.device)
        self.motor_pos = 0.0
        self.motor_vel = 0.0
        self.action = 0.0
        self.prev_action = 0.0
        self.angle = 0.0
        self._input = torch.zeros(1, 5, dtype=torch.float32, device=self.device)

    def reset(self) -> None:
        # self.motor_pos.reset()
        # self.motor_vel.reset()
        # self.action_buffer.reset()
        # self.prev_action_buffer.reset()
        # self.angle_buffer.reset()

        self.motor_pos = 0.0
        self.motor_vel = 0.0
        self.action = 0.0
        self.prev_action = 0.0
        self.angle = 0.0

    def update(self, motor_pos: float, motor_vel: float, angle: float) -> None:
        # self.motor_pos.add(motor_pos)
        # self.motor_vel.add(motor_vel)
        # self.angle_buffer.add(angle)

        self.motor_pos = motor_pos
        self.motor_vel = motor_vel
        self.angle = angle

    def forward(self) -> np.ndarray:
        # idx = 0
        # self._input[0, 0 : idx + self._num_history] = self.motor_pos.buffer
        # idx += self._num_history
        # self._input[0, idx : idx + self._num_history] = self.motor_vel.buffer
        # idx += self._num_history
        # self._input[0, idx : idx + self._num_history] = self.action_buffer.buffer
        # idx += self._num_history
        # self._input[0, idx : idx + self._num_history] = self.prev_action_buffer.buffer
        # idx += self._num_history
        # self._input[0, idx : idx + self._num_history] = self.angle_buffer.buffer

        idx = 0
        self._input[0, 0] = self.motor_pos
        idx += 1
        self._input[0, idx] = self.motor_vel
        idx += 1
        self._input[0, idx] = self.action
        idx += 1
        self._input[0, idx] = self.prev_action
        idx += 1
        self._input[0, idx] = self.angle

        with torch.no_grad():
            self.prev_action = self.action
            self.action = float(self.model(self._input)[0].cpu().numpy()[0, 0])
            return self.action
