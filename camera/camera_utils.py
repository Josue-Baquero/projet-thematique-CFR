import cv2
import numpy as np
import requests

def get_ip_webcam_frame(camera_url):
    """
    Récupère une image depuis une caméra IP (comme l'app IP Webcam)
    
    Args:
        camera_url (str): URL de la caméra IP (ex: "http://192.168.1.100:8080/shot.jpg")
    
    Returns:
        tuple: (bool, numpy.ndarray) - (succès, image)
               Si succès est False, image est None
    """
    try:
        # Envoi de la requête HTTP à la caméra IP
        response = requests.get(camera_url, stream=True)
        
        # Vérification si la requête a réussi
        if response.status_code == 200:
            # Conversion des données reçues en image
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