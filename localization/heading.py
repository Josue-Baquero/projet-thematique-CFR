from scipy.spatial.transform import Rotation as R
import numpy as np

def get_heading_angle(orientation: R) -> dict:
    quat = orientation.as_quat()
    heading_quat = np.arctan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]),
                              1 - 2 * (quat[1]**2 + quat[2]**2))

    rot_mat = orientation.as_matrix()
    heading_matrix = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])

    euler = orientation.as_euler('zyx', degrees=True)
    heading_euler = euler[0]

    rotvec = orientation.as_rotvec()
    heading_rotvec = np.arctan2(rotvec[1], rotvec[0])

    return {
        'quaternion': np.degrees(heading_quat),
        'matrix': np.degrees(heading_matrix),
        'euler': heading_euler,
        'rotvec': np.degrees(heading_rotvec)
    }