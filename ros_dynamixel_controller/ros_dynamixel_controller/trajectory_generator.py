import numpy as np


class RandomGenerator:
    def __init__(self, min_value: float, max_value: float):
        super().__init__()
        self._min_value = min_value
        self._max_value = max_value
        self._value = 0.0

    def advance(self):
        self._value = np.random.uniform(self._min_value, self._max_value)

    def generate(self) -> float:
        return self._value


class RampGenerator:
    def __init__(self, min_step_size: float, max_step_size: float):
        super().__init__()
        self._min_step_size = min_step_size
        self._max_step_size = max_step_size
        self._step_size = np.random.uniform(self._min_step_size, self._max_step_size)
        self._current_value = 0.0
        self._dir = 1.0

    def advance(self):
        self._current_value += self._dir * self._step_size
        if self._current_value >= 2.0 * np.pi:
            self._current_value = 2.0 * np.pi
            self._dir = -1.0
        elif self._current_value <= 0.0:
            self._current_value = 0.0
            self._dir = 1.0
            self._step_size = np.random.uniform(self._min_step_size, self._max_step_size)

    def generate(self) -> float:
        return self._current_value


class StepGenerator:
    def __init__(self, step_size: float):
        super().__init__()
        self._step_size = step_size
        self._current_value = 0.0
        self._dir = 1.0

    def advance(self):
        self._current_value += self._dir * self._step_size
        if self._current_value >= 2.0 * np.pi:
            self._current_value = 2.0 * np.pi
            self._dir = -1.0
        elif self._current_value <= 0.0:
            self._current_value = 0.0
            self._dir = 1.0

    def generate(self) -> float:
        return self._current_value


class SineGenerator:
    def __init__(self, amplitude: float, min_freq: float, max_freq: float, phase: float):
        super().__init__()
        self._amplitude = amplitude
        self._min_frequency = min_freq
        self._max_frequency = max_freq
        self._frequency = max_freq
        self._phase = phase
        self._offset = amplitude
        self._time = 0.0
        self._delta_time = 0.01

    def advance(self):
        self._time += self._delta_time

    def generate(self) -> float:
        if self._time > 1.0 / self._frequency:
            self._time = 0.0
            self._frequency = np.random.uniform(self._min_frequency, self._max_frequency)

        return self._offset + self._amplitude * np.sin(2.0 * np.pi * self._frequency * self._time + self._phase)
