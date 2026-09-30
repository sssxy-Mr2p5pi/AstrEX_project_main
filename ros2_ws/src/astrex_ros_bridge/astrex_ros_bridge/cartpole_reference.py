"""Simulation-time reference and stability helpers; no ROS or dynamics model."""

from dataclasses import dataclass, field
import math


def _stamp_ns(value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError('A simulation timestamp must be a nonnegative integer')
    return value


@dataclass(frozen=True)
class QuinticReference:
    """Sample a bounded, smooth position reference at a simulation timestamp.

    Sampling has no side effects. Duplicate timestamps return the same value.
    The caller detects real timestamp regression before sampling.
    """

    x_start: float
    target: float
    start_stamp_ns: int
    max_speed_mps: float = 0.05
    min_duration_sec: float = 2.0
    duration_sec: float = field(init=False)
    end_stamp_ns: int = field(init=False)

    def __post_init__(self) -> None:
        _stamp_ns(self.start_stamp_ns)
        scalars = (self.x_start, self.target, self.max_speed_mps, self.min_duration_sec)
        if not all(math.isfinite(value) for value in scalars):
            raise ValueError('Reference configuration must be finite')
        if self.max_speed_mps <= 0 or self.min_duration_sec <= 0:
            raise ValueError('Reference speed and minimum duration must be positive')
        delta = self.target - self.x_start
        if not math.isfinite(delta):
            raise ValueError('Reference displacement must be finite')
        # A zero-distance task already has its fixed reference at the endpoint.
        duration = (max(self.min_duration_sec, 1.875 * abs(delta) / self.max_speed_mps)
                    if delta else 0.0)
        if not math.isfinite(duration):
            raise ValueError('Reference duration must be finite')
        duration_ns = math.ceil(duration * 1e9)
        object.__setattr__(self, 'duration_sec', duration_ns / 1e9)
        object.__setattr__(self, 'end_stamp_ns', self.start_stamp_ns + duration_ns)

    def _fraction(self, stamp_ns: int) -> float:
        stamp_ns = _stamp_ns(stamp_ns)
        if stamp_ns <= self.start_stamp_ns:
            return 0.0
        if stamp_ns >= self.end_stamp_ns:
            return 1.0
        return (stamp_ns - self.start_stamp_ns) / (self.end_stamp_ns - self.start_stamp_ns)

    def position(self, stamp_ns: int) -> float:
        """Return x_ref, fixed at its endpoints outside the move interval."""
        s = self._fraction(stamp_ns)
        if self.target == self.x_start or s == 0.0:
            return self.x_start
        if s == 1.0:
            return self.target
        smooth = s**3 * (10.0 + s * (-15.0 + 6.0 * s))
        value = self.x_start + (self.target - self.x_start) * smooth
        # Prevent floating-point rounding from overshooting either endpoint.
        return min(max(value, min(self.x_start, self.target)), max(self.x_start, self.target))

    def velocity(self, stamp_ns: int) -> float:
        """Return the analytical reference velocity, not measured cart velocity."""
        s = self._fraction(stamp_ns)
        if not self.duration_sec or s in (0.0, 1.0):
            return 0.0
        return (self.target - self.x_start) * 30.0 * s**2 * (1.0 - s)**2 / self.duration_sec

    def finished(self, stamp_ns: int) -> bool:
        stamp_ns = _stamp_ns(stamp_ns)
        return stamp_ns >= self.end_stamp_ns


class ContinuousStability:
    """Count a qualified window only when strictly newer feedback arrives."""

    def __init__(self, required_sec: float, max_gap_sec: float = 0.05):
        if (not math.isfinite(required_sec) or not math.isfinite(max_gap_sec)
                or required_sec <= 0 or max_gap_sec <= 0):
            raise ValueError('Stable duration and feedback gap must be finite and positive')
        self.required_sec = required_sec
        self.max_gap_sec = max_gap_sec
        self.reset()

    def reset(self) -> None:
        self.start_stamp_ns = None
        self.last_stamp_ns = None
        self.duration_sim_sec = 0.0

    def update(self, stamp_ns: int, qualified: bool) -> bool:
        stamp_ns = _stamp_ns(stamp_ns)
        if not isinstance(qualified, bool):
            raise ValueError('Stable qualification must be a boolean')
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return False
        gap = (None if self.last_stamp_ns is None else
               (stamp_ns - self.last_stamp_ns) / 1e9)
        self.last_stamp_ns = stamp_ns
        if not qualified or (gap is not None and gap > self.max_gap_sec):
            self.start_stamp_ns = None
            self.duration_sim_sec = 0.0
        if qualified:
            if self.start_stamp_ns is None:
                self.start_stamp_ns = stamp_ns
            self.duration_sim_sec = (stamp_ns - self.start_stamp_ns) / 1e9
        return qualified and self.duration_sim_sec >= self.required_sec

    def window(self) -> dict | None:
        if self.start_stamp_ns is None:
            return None
        return {'start_stamp_ns': self.start_stamp_ns,
                'end_stamp_ns': self.last_stamp_ns,
                'duration_sim_sec': self.duration_sim_sec}
