"""Hardware-independent Keyence profile contract utilities."""

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Profile:
    stamp: float
    frame_id: str
    x_start_m: float
    x_pitch_m: float
    z_m: tuple[float, ...]

    def points(self) -> tuple[tuple[float, float, float], ...]:
        return tuple((self.x_start_m + index * self.x_pitch_m, 0.0, z)
                     for index, z in enumerate(self.z_m))


def _number(value, field, line_number):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'line {line_number}: {field} must be numeric')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'line {line_number}: {field} must be finite')
    return value


def parse_profiles(path: str | Path) -> tuple[Profile, ...]:
    """Read JSON-lines profiles in SI units; null heights become NaN."""
    profiles = []
    for line_number, line in enumerate(Path(path).read_text().splitlines(), start=1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f'line {line_number}: invalid JSON') from exc
        try:
            heights = record['z_m']
            if not isinstance(heights, list) or not heights:
                raise ValueError('z_m must be a non-empty list')
            z_m = tuple(math.nan if value is None else _number(value, 'z_m', line_number)
                        for value in heights)
            profile = Profile(
                stamp=_number(record['stamp'], 'stamp', line_number),
                frame_id=str(record.get('frame_id', 'keyence_frame')),
                x_start_m=_number(record['x_start_m'], 'x_start_m', line_number),
                x_pitch_m=_number(record['x_pitch_m'], 'x_pitch_m', line_number),
                z_m=z_m,
            )
        except KeyError as exc:
            raise ValueError(f'line {line_number}: missing {exc.args[0]}') from exc
        profiles.append(profile)
    if not profiles:
        raise ValueError('profile fixture contains no profiles')
    return tuple(profiles)
