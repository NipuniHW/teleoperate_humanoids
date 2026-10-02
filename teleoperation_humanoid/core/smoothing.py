"""
teleoperation_humanoid/core/smoothing.py

First-order exponential (IIR) low-pass filter for joint angles, shared by
every robot backend.
"""


class ExponentialSmoother:
    """
    Stateful per-joint exponential smoother.

    smoothed[t] = alpha * raw[t] + (1 - alpha) * smoothed[t-1]

    alpha=1.0 -> no smoothing (raw values passed through).
    alpha=0.0 -> output never changes from the first observed value.

    Each robot backend owns its own instance (rather than sharing module-
    level state) so multiple robots can run in the same process without
    cross-contaminating each other's filter history.
    """

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self._state: dict = {}

    def smooth(self, angles: dict, alpha_overrides: dict = None) -> dict:
        smoothed = {}
        for k, val in angles.items():
            a = alpha_overrides.get(k, self.alpha) if alpha_overrides else self.alpha
            if k not in self._state:
                self._state[k] = val
            self._state[k] = a * val + (1.0 - a) * self._state[k]
            smoothed[k] = self._state[k]
        return smoothed

    def reset(self):
        self._state = {}
