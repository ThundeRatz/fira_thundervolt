# import test_base  # pylint: disable=import-error

import pytest
from thundervolt.core.data import EntityData


def test_creation_entity():
    data = EntityData()
    assert data.position.x == pytest.approx(0.0)
    assert data.position.y == pytest.approx(0.0)
    assert data.position.theta == pytest.approx(0.0)
