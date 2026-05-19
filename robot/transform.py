import math
from typing import List, Sequence


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
        2. Rotate position around X/Y/Z axes
        3. Translate position
        4. Rotate orientation (rx, ry, rz)

    Parameters
    ----------
    coordinates : list of [x, y, z, rx, ry, rz]
        rx, ry, rz are orientation angles in **degrees**.

    scale : [sx, sy, sz], optional
        Scaling factors for x/y/z.
        Default: [1, 1, 1]

    rotation : [rx_deg, ry_deg, rz_deg], optional
        Rotation angles in degrees.
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

    # Defaults
    sx, sy, sz = scale if scale is not None else [1.0, 1.0, 1.0]
    rot = rotation if rotation is not None else [0.0, 0.0, 0.0]
    tx, ty, tz = translation if translation is not None else [0.0, 0.0, 0.0]

    rx_rad = math.radians(rot[0])
    ry_rad = math.radians(rot[1])
    rz_rad = math.radians(rot[2])

    transformed = []

    for coord in coordinates:
        x, y, z, rx, ry, rz = coord

        # Convert input orientation from degrees to radians
        rx = math.radians(rx)
        ry = math.radians(ry)
        rz = math.radians(rz)

        # -------------------------------------------------
        # 1. SCALE POSITION
        # -------------------------------------------------
        x *= sx
        y *= sy
        z *= sz

        # -------------------------------------------------
        # 2. ROTATE POSITION AROUND X
        # -------------------------------------------------
        y, z = (
            y * math.cos(rx_rad) - z * math.sin(rx_rad),
            y * math.sin(rx_rad) + z * math.cos(rx_rad),
        )

        # -------------------------------------------------
        # 3. ROTATE POSITION AROUND Y
        # -------------------------------------------------
        x, z = (
            x * math.cos(ry_rad) + z * math.sin(ry_rad),
            -x * math.sin(ry_rad) + z * math.cos(ry_rad),
        )

        # -------------------------------------------------
        # 4. ROTATE POSITION AROUND Z
        # -------------------------------------------------
        x, y = (
            x * math.cos(rz_rad) - y * math.sin(rz_rad),
            x * math.sin(rz_rad) + y * math.cos(rz_rad),
        )

        # -------------------------------------------------
        # 5. TRANSLATE POSITION
        # -------------------------------------------------
        x += tx
        y += ty
        z += tz

        # -------------------------------------------------
        # 6. ROTATE ORIENTATION
        # Simple additive orientation transform
        # -------------------------------------------------
        rx += rx_rad
        ry += ry_rad
        rz += rz_rad

        transformed.append([x, y, z, rx, ry, rz])

    return transformed


# Example usage
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