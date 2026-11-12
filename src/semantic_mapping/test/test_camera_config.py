import pytest
import yaml

from semantic_mapping.camera_config import (
    camera_serial_yaml_literal,
    normalize_camera_serial,
)


@pytest.mark.parametrize('serial_arg', ['327122073765', "'327122073765'", '"327122073765"'])
def test_camera_serial_is_always_passed_as_a_yaml_string(serial_arg):
    normalized = normalize_camera_serial(serial_arg)
    launch_literal = camera_serial_yaml_literal(normalized)
    assert normalized == '327122073765'
    assert yaml.safe_load(launch_literal) == normalized


def test_camera_serial_must_not_be_empty():
    with pytest.raises(ValueError):
        camera_serial_yaml_literal('  ')
