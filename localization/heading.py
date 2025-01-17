from scipy.spatial.transform import Rotation as R
import numpy as np

def get_heading_angle(orientation: R) -> dict:
    # Conversion en quaternion et calcul du cap (yaw) à partir des quaternions
    quat = orientation.as_quat()
    heading_quat = np.arctan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]),
                              1 - 2 * (quat[1]**2 + quat[2]**2))

    # Calcul du cap à partir de la matrice de rotation
    rot_mat = orientation.as_matrix()
    heading_matrix = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])

    # Récupération du cap depuis les angles d'Euler (dans l'ordre z-y-x)
    # Le premier angle (z) correspond au cap
    euler = orientation.as_euler('zyx', degrees=True)
    heading_euler = euler[0]

    # Retourne les différentes représentations du cap en degrés
    return {
        'quaternion': np.degrees(heading_quat),
        'matrix': np.degrees(heading_matrix),
        'euler': heading_euler,
    }