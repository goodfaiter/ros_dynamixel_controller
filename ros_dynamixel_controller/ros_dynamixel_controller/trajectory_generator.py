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
    
class RampGenerator():
    def __init__(self, max_step_size: float):
        super().__init__()
        self._max_step_size = max_step_size
        self._step_size = np.random.uniform(0.05, self._max_step_size)
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
            self._step_size = np.random.uniform(0.05, self._max_step_size)

    def generate(self) -> float:
        return self._current_value
    
class StepGenerator():
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
