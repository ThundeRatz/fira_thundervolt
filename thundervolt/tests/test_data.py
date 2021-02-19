import pytest
from thundervolt.core.data import Pose2D, EntityData, FieldData

def test_creation_pose2d():
    data = Pose2D()
    assert data.x == pytest.approx(0.0)
    assert data.y == pytest.approx(0.0)
    assert data.theta == pytest.approx(0.0)


def test_creation_entity():
    data = EntityData()
    assert type(data.position) is Pose2D
    assert type(data.velocity) is Pose2D


def test_creation_field_data():
    data = FieldData()
    assert type(data.robots[0]) is EntityData
    assert type(data.foes[0]) is EntityData
    assert type(data.ball) is EntityData

