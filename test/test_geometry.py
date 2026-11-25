import numpy as np

from apriltag_localization.localizer_node import (make_T, quat_to_rot, 
                                                  rot_to_quat, average_quats,
                                                  invert_T)

def test_make_T_puts_translation_in_last_column():
    T = make_T(np.eye(3), [1.0, 2.0, 3.0])
    assert T[0, 3] == 1.0


def test_rot_quat_round_trip():
    angle = np.radians(30)
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0 , 1]]) # 30 degrees about the z axis
    R_back = quat_to_rot(rot_to_quat(R))
    np.testing.assert_allclose(R_back, R, atol=1e-9)


def test_rot_quat_round_trip_180_about_x():
    R = np.diag([1.0, -1.0, -1.0])  # 180 degrees about the x axis
    R_back = quat_to_rot(rot_to_quat(R))
    np.testing.assert_allclose(R_back, R, atol=1e-9)


def test_invert_T_gives_identity():
    angle = np.radians(30)
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0 , 1]])
    T = make_T(R,[1.0, 2.0, 3.0]) # rotation and translation
    np.testing.assert_allclose(T @ invert_T(T), np.eye(4), atol=1e-9)


def test_average_quats_single_quaternion_returns_itself():
    angle = np.radians(30)
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])
    q = rot_to_quat(R)  # [x, y, z, w], unit norm by construction (30 deg about z)
    result = average_quats([q], [1.0])
    np.testing.assert_allclose(result, q, atol=1e-9)


def test_average_quats_empty_input_returns_identity():
    result = average_quats([], [])
    np.testing.assert_allclose(result, np.array([0, 0, 0, 1]), atol=1e-9)