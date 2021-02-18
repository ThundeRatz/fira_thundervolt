import pytest
from thundervolt.core.data import Pose2D, EntityData, FieldData

import numpy as np

def test_creation_pose2d():
    data = Pose2D()
    assert data.x == pytest.approx(0.0)
    assert data.y == pytest.approx(0.0)
    assert data.theta == pytest.approx(0.0)


def test_creation_entity():
    data = EntityData()
    assert type(data.position) is Pose2D
    assert type(data.velocity) is Pose2D


def test_entity_from_dict():
    entity_dict = {}

    entity_dict['x'] = 4
    entity_dict['y'] = 2
    entity_dict['orientation'] = 1
    entity_dict['vx'] = 8
    entity_dict['vy'] = 4
    entity_dict['vorientation'] = 3

    data = EntityData()

    data._from_dict(entity_dict)

    assert data.position.x == pytest.approx(4)
    assert data.position.y == pytest.approx(2)
    assert data.position.theta == pytest.approx(1)
    assert data.velocity.x == pytest.approx(8)
    assert data.velocity.y == pytest.approx(4)
    assert data.velocity.theta == pytest.approx(3)


def test_entity_from_dict_with_rotation():
    entity_dict = {}

    entity_dict['x'] = 5
    entity_dict['y'] = 7
    entity_dict['orientation'] = np.pi / 2
    entity_dict['vx'] = 34
    entity_dict['vy'] = 42
    entity_dict['vorientation'] = 56

    data = EntityData()

    data._from_dict(entity_dict, True)

    assert data.position.x == pytest.approx(-5)
    assert data.position.y == pytest.approx(-7)
    assert data.position.theta == pytest.approx((- np.pi / 2))
    assert data.velocity.x == pytest.approx(-34)
    assert data.velocity.y == pytest.approx(-42)
    assert data.velocity.theta == pytest.approx(56)


def test_creation_field_data():
    data = FieldData()
    assert type(data.robots[0]) is EntityData
    assert type(data.foes[0]) is EntityData
    assert type(data.ball) is EntityData


def test_field_from_dict():
    field_dict = {
        'ball': {
            'x': -0.375,
            'z': 0.021
        },
        'robotsYellow': [
            {
                'x': 0.68,
                'orientation': 2.37
            }
        ],
        'robotsBlue': [
            {
                'vy': 2.29e-05,
                'vorientation': 4.70
            }
        ]
    }

    test_field_data = FieldData(team_color='blue')

    test_field_data.from_vision_raw(field_dict)

    print(data)

    assert test_field_data.ball.position.x == pytest.approx(-0.375)
    assert test_field_data.ball.position.theta == pytest.approx(0)
    assert test_field_data.foes[0].position.x == pytest.approx(0.68)
    assert test_field_data.foes[0].position.theta == pytest.approx(2.37)
    assert test_field_data.robots[0].velocity.y == pytest.approx(2.29e-05)
    assert test_field_data.robots[0].velocity.theta == pytest.approx(4.70)
