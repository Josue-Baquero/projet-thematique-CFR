import cv2
import numpy as np
from screeninfo import get_monitors
from localization.robot_pose import RobotPose 
from localization.heading import get_heading_angle

def display_info(frame, info_list, color=(0, 255, 0)):
   """
   Affiche des informations textuelles sur l'image avec un fond semi-transparent
   
   Args:
       frame: Image sur laquelle afficher
       info_list: Liste des textes à afficher
       color: Couleur du texte (B,G,R), vert par défaut
   """
   # Créer une copie pour le fond semi-transparent
   overlay = frame.copy()
   
   # Dessiner un rectangle noir semi-transparent
   cv2.rectangle(overlay, (10, 10), (600, 30 * len(info_list) + 20), (0, 0, 0), -1)
   cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
  
   # Afficher chaque ligne de texte
   for i, info in enumerate(info_list):
       cv2.putText(frame, info, (20, 40 + 30*i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

def setup_window(window_name):
   """
   Configure une fenêtre OpenCV redimensionnable
   
   Args:
       window_name: Nom de la fenêtre à créer
   """
   # Créer une fenêtre redimensionnable
   cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
   
   # Obtenir les dimensions de l'écran
   screen = get_monitors()[0]
   window_width = int(screen.width * 0.8)
   window_height = int(screen.height * 0.8)
   
   # Redimensionner la fenêtre à 80% de l'écran
   cv2.resizeWindow(window_name, window_width, window_height)
   
   # Afficher une image noire initiale
   black_image = np.zeros((window_height, window_width, 3), dtype=np.uint8)
   cv2.imshow(window_name, black_image)
   cv2.waitKey(1)

def resize_frame(frame, window_name):
   """
   Redimensionne l'image à la taille de la fenêtre
   
   Args:
       frame: Image à redimensionner
       window_name: Nom de la fenêtre
       
   Returns:
       Image redimensionnée ou image originale si la fenêtre n'existe pas
   """
   try:
       # Obtenir les dimensions actuelles de la fenêtre
       window_rect = cv2.getWindowImageRect(window_name)
       if window_rect is not None:
           window_width, window_height = window_rect[2], window_rect[3]
           return cv2.resize(frame, (window_width, window_height))
   except cv2.error:
       pass  # La fenêtre n'existe pas encore
   return frame  # Retourner le frame original si la fenêtre n'existe pas

def display_pose(pose: RobotPose, mode: str):
   """
   Affiche les informations de pose selon le mode
   
   Args:
       pose: Objet RobotPose contenant position et orientation
       mode: Mode d'affichage ("robot", "board" ou "all")
   """
   if mode == "robot":
       # Affichage de la pose du robot dans le repère caméra
       print(f"\nRobot pose in camera frame:")
       print(f"Position: ({pose.position[0]:.6f}, {pose.position[1]:.6f}, {pose.position[2]:.6f})")
       print(f"Orientation (Euler angles): {pose.orientation.as_euler('xyz', degrees=True)}")
       heading_angles = get_heading_angle(pose.orientation)
       print(f"Heading (degrees):")
       print(f"  Quaternion: {heading_angles['quaternion']:.2f}")
       print(f"  Matrix: {heading_angles['matrix']:.2f}")
       print(f"  Euler: {heading_angles['euler']:.2f}")
       
   elif mode == "board":
       # Affichage de la pose du plateau dans le repère caméra
       print(f"\nBoard pose in camera frame:")
       print(f"Position: ({pose.position[0]:.6f}, {pose.position[1]:.6f}, {pose.position[2]:.6f})")
       print(f"Orientation (Euler angles): {pose.orientation.as_euler('xyz', degrees=True)}")
       
   elif mode == "all":
       # Affichage de la pose du robot dans le repère du plateau
       print(f"\nRobot pose in board frame:")
       print(f"Position: ({pose.position[0]:.6f}, {pose.position[1]:.6f}, {pose.position[2]:.6f})")
       print(f"Orientation (Euler angles): {pose.orientation.as_euler('xyz', degrees=True)}")
       heading_angles = get_heading_angle(pose.orientation)
       print(f"Heading (degrees):")
       print(f"  Quaternion: {heading_angles['quaternion']:.2f}")
       print(f"  Matrix: {heading_angles['matrix']:.2f}")
       print(f"  Euler: {heading_angles['euler']:.2f}")