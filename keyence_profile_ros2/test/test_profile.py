import math
from pathlib import Path

import pytest

from keyence_profile_ros2.profile import parse_profiles


FIXTURE = Path(__file__).parent / 'fixtures' / 'nominal_profiles.jsonl'


def test_parse_profile_fixture_preserves_si_contract():
    profile = parse_profiles(FIXTURE)[0]
    assert profile.stamp == 10.25
    assert profile.frame_id == 'keyence_frame'
    assert profile.x_pitch_m == 0.01
    assert profile.points()[0] == (-0.01, 0.0, 0.002)
    assert math.isnan(profile.points()[1][2])


def test_rejects_empty_profile_fixture(tmp_path):
    fixture = tmp_path / 'empty.jsonl'
    fixture.write_text('# no data\n')
    with pytest.raises(ValueError, match='contains no profiles'):
        parse_profiles(fixture)
