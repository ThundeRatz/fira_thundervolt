import pytest
import numpy as np
from thundervolt.comm.vision import FiraVision
from thundervolt.core.data import EntityData, FieldData


def test_entity_from_dict():
    vision = FiraVision(team_color_yellow=False)
    entity_dict = {}

    entity_dict['x'] = 4
    entity_dict['y'] = 2
    entity_dict['orientation'] = 1
    entity_dict['vx'] = 8
    entity_dict['vy'] = 4
    entity_dict['vorientation'] = 3

    data = EntityData()

    vision._entity_from_dict(data, entity_dict)

    assert data.position.x == pytest.approx(4)
    assert data.position.y == pytest.approx(2)
    assert data.position.theta == pytest.approx(1)
    assert data.velocity.x == pytest.approx(8)
    assert data.velocity.y == pytest.approx(4)
    assert data.velocity.theta == pytest.approx(3)


def test_entity_from_dict_with_rotation():
    vision = FiraVision(team_color_yellow=True)
    entity_dict = {}

    entity_dict['x'] = 5
    entity_dict['y'] = 7
    entity_dict['orientation'] = np.pi / 2
    entity_dict['vx'] = 34
    entity_dict['vy'] = 42
    entity_dict['vorientation'] = 56

    data = EntityData()

    vision._entity_from_dict(data, entity_dict, True)

    assert data.position.x == pytest.approx(-5)
    assert data.position.y == pytest.approx(-7)
    assert data.position.theta == pytest.approx((- np.pi / 2))
    assert data.velocity.x == pytest.approx(-34)
    assert data.velocity.y == pytest.approx(-42)
    assert data.velocity.theta == pytest.approx(56)


def test_field_from_dict():
    vision = FiraVision(team_color_yellow=False)

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

    test_field_data = FieldData()

    vision._field_data_from_dict(test_field_data, field_dict)

    assert test_field_data.ball.position.x == pytest.approx(-0.375)
    assert test_field_data.ball.position.theta == pytest.approx(0)
    assert test_field_data.foes[0].position.x == pytest.approx(0.68)
    assert test_field_data.foes[0].position.theta == pytest.approx(2.37)
    assert test_field_data.robots[0].velocity.y == pytest.approx(2.29e-05)
    assert test_field_data.robots[0].velocity.theta == pytest.approx(4.70)


def test_field_from_dict_with_rotation():
    vision = FiraVision(True, None, 0, 0)

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

    test_field_data = FieldData()

    vision._field_data_from_dict(test_field_data, field_dict)

    assert test_field_data.ball.position.x == pytest.approx(0.375)
    assert test_field_data.ball.position.theta == (pytest.approx(np.pi))
    assert test_field_data.robots[0].position.x == pytest.approx(-0.68)
    assert test_field_data.robots[0].position.theta == pytest.approx(-0.771592654)
    assert test_field_data.foes[0].velocity.y == pytest.approx(-2.29e-05)
    assert test_field_data.foes[0].velocity.theta == pytest.approx(4.70)
