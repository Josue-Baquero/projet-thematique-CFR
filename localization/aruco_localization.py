import cv2
import numpy as np
from .robot_pose import RobotPose
from utils.config import Config
from utils.markers import MarkerBoards
from visualization.display import display_info, resize_frame, setup_window, display_pose
from camera.camera_utils import get_ip_webcam_frame
from processing import robot_mode, board_mode, all_mode

class ArucoLocalization:
    """Classe principale pour la détection et localisation des marqueurs ArUco"""
    
    def __init__(self, config: Config):
        """Initialisation du système de localisation"""
        # Chargement de la configuration
        self.config = config
        # Chargement des paramètres de calibration de la caméra
        self.camera_matrix = np.load(config.camera_matrix_file)
        self.dist_coeffs = np.load(config.dist_coeffs_file)
        
        # Configuration du détecteur ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Création des objets pour le plateau et le robot
        self.marker_boards = MarkerBoards(self.aruco_dict, config.marker_length, 
                                          config.fixed_marker_ids, config.robot_marker_id, 
                                          config.use_real_board)
        self.board = self.marker_boards.get_board()
        self.robot_board = self.marker_boards.get_robot_board()
        
        # Configuration de la fenêtre d'affichage
        self.window_name = 'ArUco Localization'
        setup_window(self.window_name)
        cv2.waitKey(1)  # Assure la création de la fenêtre

    def process_frame(self, frame: np.ndarray, mode: str) -> RobotPose:
        """Traitement d'une image selon le mode choisi"""
        # Détection des marqueurs dans l'image
        corners, ids, _ = self.aruco_detector.detectMarkers(frame)

        # Vérification si des marqueurs sont détectés
        if ids is None or len(ids) == 0:
            display_info(frame, ["No markers detected"], color=(0, 0, 255))
            return None
        
        # Traitement selon le mode choisi
        if mode == "robot":
            return robot_mode.process(frame, corners, ids, self.camera_matrix, self.dist_coeffs,
                                      self.config, self.robot_board)
        elif mode == "board":
            return board_mode.process(frame, corners, ids, self.camera_matrix, self.dist_coeffs,
                                      self.config, self.board)
        elif mode == "all":
            return all_mode.process(frame, corners, ids, self.camera_matrix, self.dist_coeffs, 
                                    self.config, self.board, self.robot_board)

    def run(self, mode: str):
        """Boucle principale de traitement"""
        while True:
            # Capture d'une image depuis la caméra IP
            ret, frame = get_ip_webcam_frame(self.config.camera_url)
            if not ret:
                print("Failed to grab frame")
                continue

            # Traitement de l'image et affichage des résultats
            pose = self.process_frame(frame, mode)
            if pose:
                display_pose(pose, mode)

            # Redimensionnement et affichage de l'image
            resized_frame = resize_frame(frame, self.window_name)
            cv2.imshow(self.window_name, resized_frame)

            # Sortie si 'q' est pressé
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()