from enes100 import enes100
import math_utils



#Testing Aruco Marker Reading
def test_aruco_readings():
        theta_deg = rad_to_deg(enes100.theta)
        print(f"x: {enes100.x}, y: {enes100.y}, theta: {theta_deg}, visible: {enes100.visible}")
        time.sleep(1)