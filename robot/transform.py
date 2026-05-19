import math
from typing import List, Sequence


def _rotvec_to_matrix(r: Sequence[float]) -> List[List[float]]:
    """Convert a rotation vector (axis-angle) to a 3x3 rotation matrix."""
    angle = math.sqrt(r[0]**2 + r[1]**2 + r[2]**2)
    if angle < 1e-10:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    ax, ay, az = r[0] / angle, r[1] / angle, r[2] / angle
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [
        [t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay],
        [t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax],
        [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c   ],
    ]


def _matrix_to_rotvec(R: List[List[float]]) -> List[float]:
    """Convert a 3x3 rotation matrix to a rotation vector (axis-angle)."""
    trace = R[0][0] + R[1][1] + R[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    if angle < 1e-10:
        return [0.0, 0.0, 0.0]
    s = 2.0 * math.sin(angle)
    axis = [
        (R[2][1] - R[1][2]) / s,
        (R[0][2] - R[2][0]) / s,
        (R[1][0] - R[0][1]) / s,
    ]
    return [axis[0] * angle, axis[1] * angle, axis[2] * angle]


def _mat_mul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    """3x3 matrix multiplication."""
    return [
        [sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _mat_vec(R: List[List[float]], v: List[float]) -> List[float]:
    """Multiply a 3x3 rotation matrix by a 3-vector."""
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def _euler_to_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> List[List[float]]:
    """Build a rotation matrix from extrinsic X, Y, Z rotations (degrees).
    Each rotation is always around the fixed world axis, independent of the others.
    Equivalent to the combined matrix: Rz @ Ry @ Rx (applied right-to-left).
    """
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    Rx = _rotvec_to_matrix([rx, 0, 0])
    Ry = _rotvec_to_matrix([0, ry, 0])
    Rz = _rotvec_to_matrix([0, 0, rz])
    return _mat_mul(Rz, _mat_mul(Ry, Rx))


def transform_robot_coordinates(
    coordinates: Sequence[Sequence[float]],
    scale: Sequence[float] | None = None,
    rotation: Sequence[float] | None = None,
    translation: Sequence[float] | None = None,
) -> List[List[float]]:
    """
    Transform 6-axis robot coordinates:
        [x, y, z, rx, ry, rz]

    Transformations are applied in this order:
        1. Scale position (x, y, z)
        2. Rotate position and orientation using intrinsic X→Y→Z Euler angles
        3. Translate position

    Rotations use extrinsic (fixed-axis) composition — rx always rotates
    around the user-frame X axis, ry around Y, rz around Z, regardless of
    the calibration rotation. TCP orientation is expressed in the user/camera
    frame and is mapped to the robot frame via frame-change conjugation:
    R_robot = R_cal @ R_user @ R_cal^T.

    Parameters
    ----------
    coordinates : list of [x, y, z, rx, ry, rz]
        rx, ry, rz are orientation angles in **degrees**.

    scale : [sx, sy, sz], optional
        Scaling factors for x/y/z.
        Default: [1, 1, 1]

    rotation : [rx_deg, ry_deg, rz_deg], optional
        Rotation angles in degrees (extrinsic, fixed world axes).
        rx always rotates around world X, ry around world Y, rz around world Z.
        Applied to both position and orientation.
        Default: [0, 0, 0]

    translation : [tx, ty, tz], optional
        Translation offsets.
        Default: [0, 0, 0]

    Returns
    -------
    list of transformed coordinates [x, y, z, rx, ry, rz]
        rx, ry, rz are in **radians** (as required by the robot controller).
    """

    sx, sy, sz = scale if scale is not None else [1.0, 1.0, 1.0]
    rot = rotation if rotation is not None else [0.0, 0.0, 0.0]
    tx, ty, tz = translation if translation is not None else [0.0, 0.0, 0.0]

    R = _euler_to_matrix(rot[0], rot[1], rot[2])

    transformed = []

    for coord in coordinates:
        x, y, z, rx, ry, rz = coord

        # 1. SCALE POSITION
        x *= sx
        y *= sy
        z *= sz

        # 2. ROTATE POSITION — around fixed world axes (extrinsic)
        x, y, z = _mat_vec(R, [x, y, z])

        # 3. TRANSLATE POSITION
        x += tx
        y += ty
        z += tz

        # 4. TRANSFORM ORIENTATION through the calibration rotation.
        # The input rx/ry/rz describe the TCP orientation in the *user/camera* frame.
        # Expressing the same orientation in the robot frame requires:
        #   R_robot = R_cal @ R_user @ R_cal^T  (frame-change conjugation)
        # This ensures rx always rotates around the user-frame X axis normal,
        # ry around Y, and rz around Z, regardless of the calibration rotation.
        R_tcp_user = _euler_to_matrix(rx, ry, rz)
        R_cal_T = [[R[j][i] for j in range(3)] for i in range(3)]
        R_tcp_robot = _mat_mul(R, _mat_mul(R_tcp_user, R_cal_T))
        rx_out, ry_out, rz_out = _matrix_to_rotvec(R_tcp_robot)

        transformed.append([x, y, z, rx_out, ry_out, rz_out])

    return transformed


if __name__ == "__main__":
    robot_points = [
        [100, 200, 300, 0, 0, 0],
        [150, 250, 350, 10, 20, 30],
    ]

    result = transform_robot_coordinates(
        robot_points,
        scale=[1.2, 1.2, 1.0],
        rotation=[0, 0, 45],
        translation=[1000, 0, 500],
    )

    for r in result:
        print(r)
