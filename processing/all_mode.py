import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from localization.robot_pose import RobotPose
from visualization.display import display_info

def process(frame, corners, ids, camera_matrix, dist_coeffs, config, board, robot_board):
   """
   Traite une image pour détecter et calculer les poses du plateau et du robot.
   
   Args:
       frame: Image à traiter
       corners: Coins des marqueurs ArUco détectés
       ids: IDs des marqueurs détectés
       camera_matrix: Matrice de calibration de la caméra
       dist_coeffs: Coefficients de distorsion
       config: Configuration du système
       board: Objet représentant le plateau
       robot_board: Objet représentant le robot
   
   Returns:
       RobotPose: Pose du robot relative au plateau, ou None si erreur
   """
   # Identification des marqueurs du plateau
   board_indices = np.isin(ids, config.fixed_marker_ids)
   if not np.any(board_indices):
       display_info(frame, ["No board markers detected"], color=(0, 0, 255))
       return None
  
   # Extraction des coins et IDs des marqueurs du plateau
   board_corners = [corners[i] for i in range(len(ids)) if board_indices[i]]
   board_ids = ids[board_indices]

   try:
       # Calcul de la pose du plateau
       obj_points, img_points = board.matchImagePoints(board_corners, board_ids)
       success_board, rvec_board, tvec_board = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
       if not success_board:
           display_info(frame, ["Failed to estimate board pose"], color=(0, 0, 255))
           return None
   except Exception as e:
       display_info(frame, [f"Error estimating board pose: {e}"], color=(0, 0, 255))
       return None
  
   # Recherche du marqueur du robot
   robot_indices = np.where(ids == config.robot_marker_id)[0]
   if len(robot_indices) == 0:
       display_info(frame, [f"Robot marker (ID: {config.robot_marker_id}) not detected"], color=(0, 0, 255))
       return None
  
   try:
       # Calcul de la pose du robot
       obj_points = robot_board.getObjPoints()
       success_robot, rvec_robot, tvec_robot = cv2.solvePnP(
           obj_points[0],
           corners[robot_indices[0]],
           camera_matrix,
           dist_coeffs
       )
       if not success_robot:
           display_info(frame, ["Failed to estimate robot pose"], color=(0, 0, 255))
           return None
   except Exception as e:
       error_msg = f"Error estimating robot pose: {str(e)}"
       print(error_msg)  # Log to console
       display_info(frame, [error_msg], color=(0, 0, 255))
       return None
  
   # Construction des matrices de transformation homogènes
   # T_cw : transformation caméra -> plateau (world)
   # T_cr : transformation caméra -> robot
   R_cw = R.from_rotvec(rvec_board.flatten())
   t_cw = tvec_board.flatten()
   T_cw = np.eye(4)
   T_cw[:3, :3] = R_cw.as_matrix()
   T_cw[:3, 3] = t_cw

   R_cr = R.from_rotvec(rvec_robot.flatten())
   t_cr = tvec_robot.flatten()
   T_cr = np.eye(4)
   T_cr[:3, :3] = R_cr.as_matrix()
   T_cr[:3, 3] = t_cr

   # Calcul de la transformation plateau -> robot (T_wr)
   T_wr = np.linalg.inv(T_cw) @ T_cr

   # Extraction de la position et orientation relatives
   t_wr = T_wr[:3, 3]  # Position du robot dans le repère du plateau
   R_wr = R.from_matrix(T_wr[:3, :3])  # Orientation du robot dans le repère du plateau

   # Affichage des informations de pose
   info = [
       "All mode: Board and Robot poses estimated",
       f"Robot position (board frame): [{t_wr[0]:.3f}, {t_wr[1]:.3f}, {t_wr[2]:.3f}]",
       f"Robot orientation (board frame): [{R_wr.as_euler('xyz', degrees=True)[0]:.2f}, {R_wr.as_euler('xyz', degrees=True)[1]:.2f}, {R_wr.as_euler('xyz', degrees=True)[2]:.2f}]"
   ]
   display_info(frame, info)

   # Visualisation des repères sur l'image
   cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_board, tvec_board, config.axis_length)
   cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_robot, tvec_robot, config.axis_length)
  
   # Retourne la pose relative
   return RobotPose(tuple(t_wr), R_wr)