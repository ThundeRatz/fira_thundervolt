from thundervolt.coach import Coach
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand


def test_closest_to_point():
    team_command = TeamCommand()
    field_data = FieldData()
    coach = Coach(field_data, team_command)

    robot_0_positions = [(0, 0), (1, 0),     (-3, 4), (2, 3), (25, 0)]
    robot_1_positions = [(3, -3), (123, 123), (5, 3), (8, -9), (1, 2)]
    robot_2_positions = [(2, 4), (50, -3),    (7, 2), (-4, 3), (6, 7)]

    robots = [robot_0_positions, robot_1_positions, robot_2_positions]

    point_positions = [(0, 1), (100, 100), (3, -2), (10, 1), (9, 18)]

    expected_results = [(0, 2, 1),
                        (1, 2, 0),
                        (1, 2, 0),
                        (0, 1, 2),
                        (2, 1, 0)]

    for i in range(len(point_positions)):
        print(f"TEST NUMBER: {i}")

        for j in range(3):
            field_data.robots[j].position.x = robots[j][i][0]
            field_data.robots[j].position.y = robots[j][i][1]


        assert coach._ordered_closest_robots_to_point(point_positions[i]) == expected_results[i]

