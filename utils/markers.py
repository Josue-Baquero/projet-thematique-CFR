import cv2
import numpy as np

class MarkerBoards:
    """
    Classe qui gère la configuration spatiale des marqueurs ArUco pour la localisation.
    
    Cette classe a pour objectif de :
    - Définir la géométrie du plateau de jeu avec ses 4 marqueurs de référence aux coins
    - Définir la géométrie du marqueur unique placé sur le robot
    - Fournir ces configurations à OpenCV pour permettre le calcul précis de :
        * La position et orientation du plateau dans l'espace (via les 4 marqueurs fixes)
        * La position et orientation du robot dans l'espace (via son marqueur)
    """

    def __init__(self, aruco_dict, marker_length, fixed_marker_ids, robot_marker_id, use_real_board):
        self.aruco_dict = aruco_dict # Dictionnaire de marqueurs ArUco
        self.marker_length = marker_length # Taille physique du marqueur DU ROBOT (en mm)
        self.fixed_marker_ids = fixed_marker_ids # Id des marqueurs du plateau 
        self.robot_marker_id = robot_marker_id    # Id du marqueur du robot
        self.board = self.setup_board(use_real_board)  # Configure le plateau principal
        self.robot_board = self.setup_robot_board()  # Configure le marqueur du robot

    def setup_board(self, use_real_board):
        """
        Configure le plateau avec 4 marqueurs ArUco.
        L'origine est au centre du plateau.
        Les marqueurs sont disposés aux coins avec :
            - Marqueur 20 : haut gauche
            - Marqueur 21 : haut droite
            - Marqueur 22 : bas gauche
            - Marqueur 23 : bas droite
        
        Args:
            use_real_board (bool): Si True, utilise le plateau Coupe de France 
                                Si False, utilise le petit plateau (marqueurs imprimés)
        
        Distances pour le plateau Coupe de France :
            - Taille des ArUcos : 100 mm. Pour la changer il faut modifier objPoints
            - Distance entre ArUcos voisins : 850mm dans la longueur et 350mm dans la hauteur 
        """
        if use_real_board:
            objPoints = np.array([
                [[-950, 450, 0], [-850, 450, 0], [-850, 350, 0], [-950, 350, 0]],  # Marker 20
                [[850, 450, 0], [950, 450, 0], [950, 350, 0], [850, 350, 0]],    # Marker 21
                [[-950, -350, 0], [-850, -350, 0], [-850, -450, 0], [-950, -450, 0]],    # Marker 22
                [[850, -350, 0], [950, -350, 0], [950, -450, 0], [850, -450, 0]]       # Marker 23
            ], dtype=np.float32)
        else:
            objPoints = np.array([
                [[-97.75, 97.75, 0], [-58.75, 97.75, 0], [-58.75, 58.75, 0], [-97.75, 58.75, 0]], # Marqueur 20
                [[58.75, 97.75, 0], [97.75, 97.75, 0], [97.75, 58.75, 0], [58.75, 58.75, 0]], # Marqueur 21
                [[-97.75, -58.75, 0], [-58.75, -58.75, 0], [-58.75, -97.75, 0], [-97.75, -97.75, 0]], # Marqueur 22
                [[58.75, -58.75, 0], [97.75, -58.75, 0], [97.75, -97.75, 0], [58.75, -97.75, 0]] # Marqueur 23
            ], dtype=np.float32)
            
        return cv2.aruco.Board(objPoints, self.aruco_dict, np.array(self.fixed_marker_ids))

    def setup_robot_board(self):
            # Création d'un plateau unique pour le robot
            # Définit un carré centré à l'origine avec la taille du marqueur
            objPoints = np.array([
                [-self.marker_length/2, self.marker_length/2, 0],  # Coin supérieur gauche
                [self.marker_length/2, self.marker_length/2, 0],   # Coin supérieur droit
                [self.marker_length/2, -self.marker_length/2, 0],  # Coin inférieur droit
                [-self.marker_length/2, -self.marker_length/2, 0]  # Coin inférieur gauche
            ], dtype=np.float32).reshape((1, 4, 3))
            return cv2.aruco.Board(objPoints, self.aruco_dict, np.array([self.robot_marker_id]))  # Crée le plateau avec l'ID 36

    def get_board(self):
        return self.board

    def get_robot_board(self):
        return self.robot_board