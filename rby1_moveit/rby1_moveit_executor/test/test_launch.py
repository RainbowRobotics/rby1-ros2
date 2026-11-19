"""The launch file's helpers: which MoveIt configuration a robot gets. No ROS graph."""

import importlib.util
from pathlib import Path

import pytest

LAUNCH = Path(__file__).resolve().parent.parent / 'launch' / 'moveit_executor.launch.py'
spec = importlib.util.spec_from_file_location('moveit_executor_launch', LAUNCH)
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def test_config_package_names_follow_the_driver():
    assert launch.config_package('m', 1.2) == ('rby1_moveit_m_1_2', 'RBY1_M_v1_2')
    assert launch.config_package(' A ', 1.0) == ('rby1_moveit_a_1_0', 'RBY1_A_v1_0')


def test_bad_robots_are_refused_with_what_to_do():
    with pytest.raises(ValueError, match='robot_version 0.0.*rebuild the driver'):
        launch.config_package('m', 0.0)
    with pytest.raises(ValueError, match='kind'):
        launch.config_package('x', 1.2)


def test_every_argument_has_a_default_and_a_description():
    for name, (default, text) in launch.ARGUMENTS.items():
        assert isinstance(default, str) and text, name
