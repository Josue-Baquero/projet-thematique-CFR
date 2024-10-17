import cv2
import numpy as np
from cv2 import aruco

def generate_single_aruco_marker(marker_id, marker_size=400):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker_img = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    
    border_size = 50
    img_with_border = np.ones((marker_size + 2*border_size, marker_size + 2*border_size), dtype=np.uint8) * 255
    img_with_border[border_size:border_size+marker_size, border_size:border_size+marker_size] = marker_img
    
    return img_with_border

if __name__ == "__main__":
    robot_marker_id = 36
    robot_marker = generate_single_aruco_marker(robot_marker_id)
    
    cv2.imshow("Robot ArUco Marker", robot_marker)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    cv2.imwrite("images/markers/robot_marker.png", robot_marker)
    print("Le marqueur ArUco du robot a été sauvegardé sous 'images/markers/robot_marker.png'")