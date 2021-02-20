import pytest
import numpy as np

from thundervolt.core import utils

def test_versor_return():
    vec = (0,1.2)
    unit_vec = utils.versor(vec)

    assert type(unit_vec) is np.ndarray
    assert unit_vec.shape == (2,)

    vec = [50.8, 60.075]
    unit_vec = utils.versor(vec)

    assert type(unit_vec) is np.ndarray
    assert unit_vec.shape == (2,)

    vec = np.array([7.4, 6.75])
    unit_vec = utils.versor(vec)

    assert type(unit_vec) is np.ndarray
    assert unit_vec.shape == (2,)

def test_versor():
    vec = np.array([1, 2])
    unit_vec = utils.versor(vec)

    assert unit_vec[0] == pytest.approx(1 / np.sqrt(5))
    assert unit_vec[1] == pytest.approx(2 / np.sqrt(5))
    assert np.linalg.norm(unit_vec) == pytest.approx(1)

    vec = np.array([12, 16])
    unit_vec = utils.versor(vec)

    assert unit_vec[0] == pytest.approx(12 / 20)
    assert unit_vec[1] == pytest.approx(16 / 20)
    assert np.linalg.norm(unit_vec) == pytest.approx(1)

def test_rotate_return():
    vec = (0,0)
    rotated_vec = utils.rotate(vec, 3.875)

    assert type(rotated_vec) is np.ndarray
    assert rotated_vec.shape == (2,)

    vec = [545, 7544]
    rotated_vec = utils.rotate(vec, 9.547)

    assert type(rotated_vec) is np.ndarray
    assert rotated_vec.shape == (2,)

    vec = np.array([50, 60])
    rotated_vec = utils.rotate(vec, 100)

    assert type(rotated_vec) is np.ndarray
    assert rotated_vec.shape == (2,)

def test_rotate():
    vec = np.array([1, 2])
    rotated_vec = utils.rotate(vec, np.pi / 2)

    assert rotated_vec[0] == pytest.approx(-vec[1])
    assert rotated_vec[1] == pytest.approx(vec[0])
    assert np.linalg.norm(rotated_vec) == pytest.approx(np.linalg.norm(vec))

    vec = np.array([1, 2])
    rotated_vec = utils.rotate(vec, np.pi)
    rotated_vec_2 = utils.rotate(vec, -np.pi)

    assert rotated_vec[0] == pytest.approx(-vec[0])
    assert rotated_vec[1] == pytest.approx(-vec[1])
    assert np.linalg.norm(rotated_vec) == pytest.approx(np.linalg.norm(vec))
    assert rotated_vec[0] == pytest.approx(rotated_vec_2[0])
    assert rotated_vec[1] == pytest.approx(rotated_vec_2[1])

def test_assert_angle():
    assert utils.assert_angle(2 * np.pi) == pytest.approx(0)
    assert utils.assert_angle(np.pi) == pytest.approx(np.pi)
    assert utils.assert_angle(3 * np.pi) == pytest.approx(np.pi)
    assert utils.assert_angle(5 * np.pi) == pytest.approx(np.pi)
    assert utils.assert_angle(5 * np.pi + np.pi / 4) == pytest.approx(- 3 * np.pi / 4)
    assert utils.assert_angle(- 7 * np.pi / 2) == pytest.approx(np.pi / 2)

