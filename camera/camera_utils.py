import cv2
import numpy as np
import requests
from screeninfo import get_monitors

def get_ip_webcam_frame(camera_url):
    try:
        response = requests.get(camera_url, stream=True)
        if response.status_code == 200:
            bytes_array = bytes(response.content)
            numpy_array = np.frombuffer(bytes_array, dtype=np.uint8)
            frame = cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)
            return True, frame
        else:
            print(f"Erreur lors de la récupération de l'image : {response.status_code}")
            return False, None
    except requests.RequestException as e:
        print(f"Erreur de connexion à IP Webcam : {e}")
        return False, None

def setup_window(window_name):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    screen = get_monitors()[0]
    window_width = int(screen.width * 0.8)
    window_height = int(screen.height * 0.8)
    cv2.resizeWindow(window_name, window_width, window_height)
    # Ajoutez ces lignes pour afficher une image noire initiale
    black_image = np.zeros((window_height, window_width, 3), dtype=np.uint8)
    cv2.imshow(window_name, black_image)
    cv2.waitKey(1)