# Dette er filen der vi definerer grensesnittet ("ordboken") Gemini fpr lov til å bruke.

class RobotActionTools:
    """Samler opp funksjonskallene Gemini kan bruke."""
    
    def __init__(self, robot_controller, coordinate_converter):
        self.robot = robot_controller
        self.homography = coordinate_converter
        self.z_height_hover = 0.150  # 15 cm over bordet: "Hover-høyde"
        self.z_height_pick = 0.050   # 5 cm over bordet: "Plukk-høyde"

    def move_to_object(self, normalized_y: int, normalized_x: int):
        """
        Gemini kaller denne når den vil at armen skal flytte seg til over objektet sitt senter.
        :param normalized_y: Y koordinaten i piksel grid (0-1000)
        :param normalized_x: X koordinaten i piksel grid (0-1000)
        """
        print(f"[Tools] Gemini vil flytte til bilde-punkt: Y={normalized_y}, X={normalized_x}")
        
        # Konverter fra [y, x] 0-1000 til faktiske millimeter / meter på bordet
        # vha Homografi-matrisen fra calibrate()
        rx, ry = self.homography.convert_gemini_to_robot(normalized_y, normalized_x)
        
        print(f"[Tools] Beregnet Robot-Koordinat: X={rx:.3f}, Y={ry:.3f}")
        
        # Kjør UR3-koden asynkront eller synkront. (Her bruker vi Hover height slik
        # at vi ikke dunker i ting på vei bort)
        self.robot.move_to_xyz(x=rx, y=ry, z=self.z_height_hover)
        
        # Agent response
        return {"status": "success", "message": f"Flyttet til X:{rx:.3f}, Y:{ry:.3f}"}

    def pick_object_at(self, normalized_y: int, normalized_x: int):
        """
        Gemini kaller denne funksjonen for å utføre en fult pick-operasjon (Sentre -> gå ned -> klyp).
        :param normalized_y: Y koordinaten objektets senter (0-1000)
        :param normalized_x: X koordinaten objektets senter (0-1000)
        """
        print(f"[Tools] Utfører Pick Operasjon på bildepunkt Y:{normalized_y}, X:{normalized_x}")
        rx, ry = self.homography.convert_gemini_to_robot(normalized_y, normalized_x)
        
        # 1. Flytt OVER objektet
        self.robot.move_to_xyz(rx, ry, self.z_height_hover)
        
        # 2. Åpne griperen
        self.robot.release_object()
        
        # 3. Gå NEd til objektet
        self.robot.move_to_xyz(rx, ry, self.z_height_pick)
        
        # 4. Klem
        self.robot.grab_object()
        
        # 5. Gå OPP igjen
        self.robot.move_to_xyz(rx, ry, self.z_height_hover)
        
        return {"status": "success", "message": "Objekt plukket opp."}
    
    def place_object_at(self, normalized_y: int, normalized_x: int):
        """
        Plasserer holdt objekt på angitt posisjon og slipper det.
        :param normalized_y: Y koordinaten i piksel grid (0-1000)
        :param normalized_x: X koordinaten i piksel grid (0-1000)
        """
        print(f"[Tools] Plasserer objekt på bildepunkt Y:{normalized_y}, X:{normalized_x}")
        rx, ry = self.homography.convert_gemini_to_robot(normalized_y, normalized_x)

        # 1. Flytt OVER destinasjonen
        self.robot.move_to_xyz(rx, ry, self.z_height_hover)

        # 2. Gå NED til slipphøyde
        self.robot.move_to_xyz(rx, ry, self.z_height_pick)

        # 3. Slipp
        self.robot.release_object()

        # 4. Gå OPP igjen
        self.robot.move_to_xyz(rx, ry, self.z_height_hover)

        return {"status": "success", "message": "Objekt plassert."}

    def get_registered_tools(self):
        """Returnerer funksjonene vi vil gi til Gen AI SDK-en sin `tools` parameter."""
        return [self.move_to_object, self.pick_object_at, self.place_object_at]