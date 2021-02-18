import pytest

from thundervolt.core.command import RobotCommand, TeamCommand

def test_creation_robot_command():
    cmd = RobotCommand(3, 5)
    assert cmd.left_speed == pytest.approx(3)
    assert cmd.right_speed == pytest.approx(5)


def test_sum_robot_commands():
    cmd_one = RobotCommand(7, 5)
    cmd_two = RobotCommand(3, 4)

    sum_of_cmds = cmd_one + cmd_two

    assert sum_of_cmds.left_speed == pytest.approx(10)
    assert sum_of_cmds.right_speed == pytest.approx(9)


def test_mult_robot_command():
    cmd = RobotCommand(3, 4)

    mult_cmd = cmd * 3

    assert mult_cmd.left_speed == pytest.approx(9)
    assert mult_cmd.right_speed == pytest.approx(12)

    cmd *= 2

    assert cmd.left_speed == pytest.approx(6)
    assert cmd.right_speed == pytest.approx(8)


def test_creation_team_command():
    team_cmd = TeamCommand()

    assert type(team_cmd.commands[0]) is RobotCommand


def test_singleton_team_command():
    team_cmd_one = TeamCommand()
    team_cmd_two = TeamCommand()

    assert team_cmd_one is team_cmd_two

    team_cmd_one.commands[0].left_speed = 42

    assert team_cmd_two.commands[0].left_speed is team_cmd_one.commands[0].left_speed

    team_cmd_one.commands[2].right_speed = 24

    assert team_cmd_two.commands[2].right_speed == pytest.approx(24)

    team_cmd_three = TeamCommand()

    assert team_cmd_three.commands[0].left_speed == pytest.approx(42)


def test_team_command_reset():
    team_cmd = TeamCommand()

    for robot in team_cmd.commands:
        robot.left_speed = 43
        robot.right_speed = 24

    team_cmd.reset()

    assert team_cmd.commands[0].left_speed == pytest.approx(0)
    assert team_cmd.commands[0].right_speed == pytest.approx(0)
    assert team_cmd.commands[1].left_speed == pytest.approx(0)
    assert team_cmd.commands[1].right_speed == pytest.approx(0)
    assert team_cmd.commands[2].left_speed == pytest.approx(0)
    assert team_cmd.commands[2].right_speed == pytest.approx(0)
