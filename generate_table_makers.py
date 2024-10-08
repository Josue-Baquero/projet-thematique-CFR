import cv2
import numpy as np
from cv2 import aruco

def create_aruco_board():
    # Définir le dictionnaire ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    # Définir les IDs des marqueurs
    marker_ids = np.array([20, 21, 22, 23], dtype=np.int32)
    
    # Définir les positions des marqueurs (en mètres)
    marker_length = 0.2  # 20 cm
    marker_positions = [
        np.array([
            [0, 0, 0],
            [marker_length, 0, 0],
            [marker_length, marker_length, 0],
            [0, marker_length, 0]
        ], dtype=np.float32) for _ in range(4)
    ]
    
    # Ajuster les positions pour chaque marqueur
    marker_positions[1][:, 0] += 0.8  # Déplacer le deuxième marqueur vers la droite
    marker_positions[2][:, 1] += 0.8  # Déplacer le troisième marqueur vers le bas
    marker_positions[3][:, 0] += 0.8  # Déplacer le quatrième marqueur en diagonale
    marker_positions[3][:, 1] += 0.8
    
    # Créer le Board
    board = aruco.Board(marker_positions, aruco_dict, marker_ids)
    
    return board

def display_board(board):
    # Générer l'image du Board
    img_size = (1000, 1000)  # Augmenter la taille de l'image
    board_image = board.generateImage(img_size)
    
    # Ajouter une bordure blanche
    border_size = 20
    board_image_with_border = cv2.copyMakeBorder(
        board_image, 
        border_size, border_size, border_size, border_size, 
        cv2.BORDER_CONSTANT, 
        value=[255, 255, 255]
    )
    
    # Afficher l'image
    cv2.imshow("ArUco Board", board_image_with_border)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Sauvegarder l'image
    cv2.imwrite("aruco_board.png", board_image_with_border)

if __name__ == "__main__":
    board = create_aruco_board()