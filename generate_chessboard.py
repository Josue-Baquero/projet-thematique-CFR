import numpy as np
import cv2

def generate_chessboard(square_size, width, height):
    """
    Génère une image d'échiquier.
    
    :param square_size: Taille de chaque carré en pixels
    :param width: Nombre de carrés en largeur
    :param height: Nombre de carrés en hauteur
    :return: Image de l'échiquier
    """
    board = np.zeros((height*square_size, width*square_size), dtype=np.uint8)
    
    for i in range(height):
        for j in range(width):
            if (i + j) % 2 == 0:
                board[i*square_size:(i+1)*square_size, j*square_size:(j+1)*square_size] = 255
    
    return board

# Paramètres de l'échiquier
square_size = 100  # Taille de chaque carré en pixels
width = 10  # Nombre de carrés en largeur
height = 7  # Nombre de carrés en hauteur

# Générer l'échiquier
chessboard = generate_chessboard(square_size, width, height)

# Sauvegarder l'image
cv2.imwrite('chessboard.png', chessboard)

# Afficher l'image (optionnel)
cv2.imshow('Chessboard', chessboard)
cv2.waitKey(0)
cv2.destroyAllWindows()