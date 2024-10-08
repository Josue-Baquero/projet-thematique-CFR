import cv2
import numpy as np
from cv2 import aruco

def generate_single_aruco_marker(marker_id, marker_size=400):
    # Définir le dictionnaire ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    # Générer le marqueur
    marker_img = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    
    # Ajouter une bordure blanche pour une meilleure visibilité
    border_size = 50
    img_with_border = np.ones((marker_size + 2*border_size, marker_size + 2*border_size), dtype=np.uint8) * 255
    img_with_border[border_size:border_size+marker_size, border_size:border_size+marker_size] = marker_img
    
    return img_with_border

def display_marker(img):
    cv2.namedWindow("Robot ArUco Marker", cv2.WINDOW_NORMAL)
    cv2.imshow("Robot ArUco Marker", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # ID du marqueur pour le robot (différent des marqueurs fixes)
    robot_marker_id = 36
    
    # Générer l'image du marqueur
    robot_marker = generate_single_aruco_marker(robot_marker_id)
    
    # Afficher le marqueur
    display_marker(robot_marker)
    
    # Sauvegarder l'image du marqueur
    cv2.imwrite("robot_aruco_marker.png", robot_marker)
    print("Le marqueur ArUco du robot a été sauvegardé sous 'robot_aruco_marker.png'")