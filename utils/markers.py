import cv2
import numpy as np

class MarkerBoards:
    def __init__(self, aruco_dict, marker_length):
        self.aruco_dict = aruco_dict
        self.marker_length = marker_length
        self.board = self.setup_board()
        self.robot_board = self.setup_robot_board()

    def setup_board(self, use_real_board=True):
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
            - Taille des ArUcos : 100 mm
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
                # Marqueur 20
                [[-97.75, 97.75, 0], [-58.75, 97.75, 0], [-58.75, 58.75, 0], [-97.75, 58.75, 0]],
                # Marqueur 21
                [[58.75, 97.75, 0], [97.75, 97.75, 0], [97.75, 58.75, 0], [58.75, 58.75, 0]],
                # Marqueur 22
                [[-97.75, -58.75, 0], [-58.75, -58.75, 0], [-58.75, -97.75, 0], [-97.75, -97.75, 0]],
                # Marqueur 23
                [[58.75, -58.75, 0], [97.75, -58.75, 0], [97.75, -97.75, 0], [58.75, -97.75, 0]]
            ], dtype=np.float32)
            
        return cv2.aruco.Board(objPoints, self.aruco_dict, np.array([20, 21, 22, 23]))

    def setup_robot_board(self):
        objPoints = np.array([
            [-self.marker_length/2, self.marker_length/2, 0],
            [self.marker_length/2, self.marker_length/2, 0],
            [self.marker_length/2, -self.marker_length/2, 0],
            [-self.marker_length/2, -self.marker_length/2, 0]
        ], dtype=np.float32).reshape((1, 4, 3))
        return cv2.aruco.Board(objPoints, self.aruco_dict, np.array([36]))  # Assuming 36 is the robot marker ID

    def get_board(self):
        return self.board

    def get_robot_board(self):
        return self.robot_board