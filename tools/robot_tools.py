# Dette er filen der vi definerer grensesnittet ("ordboken") Gemini får lov til å bruke.

class RobotActionTools:
    """Samler opp funksjonskallene Gemini kan bruke."""

    def __init__(self, robot_controller, coordinate_converter):
        self.robot = robot_controller
        self.homography = coordinate_converter
        self.z_height_hover = 0.150  # 15 cm over bordet: "Hover-høyde"
        self.z_height_pick = 0.050   # 5 cm over bordet: "Plukk-høyde"

    def _current_rotation(self) -> list:
        """Leser nåværende TCP-orientering slik at vi bevarer verktøyretningen under XY-forflytninger."""
        if self.robot.connected:
            pose = self.robot.get_pose()
            if pose:
                return list(pose[3:6])
        return [0.0, 0.0, 0.0]

    def move_to_object(self, normalized_y: int, normalized_x: int):
        """
        Gemini kaller denne når den vil at armen skal flytte seg til over objektets senter.
        :param normalized_y: Y koordinaten i piksel grid (0-1000)
        :param normalized_x: X koordinaten i piksel grid (0-1000)
        """
        print(f"[Tools] Gemini vil flytte til bilde-punkt: Y={normalized_y}, X={normalized_x}")

        rx, ry = self.homography.convert_gemini_to_robot(normalized_y, normalized_x)
        print(f"[Tools] Beregnet Robot-Koordinat: X={rx:.3f}, Y={ry:.3f}")

        rot = self._current_rotation()
        self.robot.move_to_xyz([rx, ry, self.z_height_hover, *rot])

        return {"status": "success", "message": f"Flyttet til X:{rx:.3f}, Y:{ry:.3f}"}

    def pick_object_at(self, normalized_y: int, normalized_x: int):
        """
        Utfører en full pick-operasjon (hover → åpne → ned → klem → opp).
        :param normalized_y: Y koordinaten objektets senter (0-1000)
        :param normalized_x: X koordinaten objektets senter (0-1000)
        """
        print(f"[Tools] Utfører Pick Operasjon på bildepunkt Y:{normalized_y}, X:{normalized_x}")
        rx, ry = self.homography.convert_gemini_to_robot(normalized_y, normalized_x)
        rot = self._current_rotation()

        # 1. Flytt OVER objektet
        self.robot.move_to_xyz([rx, ry, self.z_height_hover, *rot])

        # 2. Åpne griperen
        self.robot.release_object()

        # 3. Gå NED til objektet
        self.robot.move_to_xyz([rx, ry, self.z_height_pick, *rot])

        # 4. Klem
        self.robot.grab_object()

        # 5. Gå OPP igjen
        self.robot.move_to_xyz([rx, ry, self.z_height_hover, *rot])

        return {"status": "success", "message": "Objekt plukket opp."}

    def place_object_at(self, normalized_y: int, normalized_x: int):
        """
        Plasserer holdt objekt på angitt posisjon og slipper det.
        :param normalized_y: Y koordinaten i piksel grid (0-1000)
        :param normalized_x: X koordinaten i piksel grid (0-1000)
        """
        print(f"[Tools] Plasserer objekt på bildepunkt Y:{normalized_y}, X:{normalized_x}")
        rx, ry = self.homography.convert_gemini_to_robot(normalized_y, normalized_x)
        rot = self._current_rotation()

        # 1. Flytt OVER destinasjonen
        self.robot.move_to_xyz([rx, ry, self.z_height_hover, *rot])

        # 2. Gå NED til slipphøyde
        self.robot.move_to_xyz([rx, ry, self.z_height_pick, *rot])

        # 3. Slipp
        self.robot.release_object()

        # 4. Gå OPP igjen
        self.robot.move_to_xyz([rx, ry, self.z_height_hover, *rot])

        return {"status": "success", "message": "Objekt plassert."}

    def get_registered_tools(self):
        """Returnerer funksjonene vi vil gi til Gen AI SDK-en sin `tools` parameter."""
        return [self.move_to_object, self.pick_object_at, self.place_object_at]
