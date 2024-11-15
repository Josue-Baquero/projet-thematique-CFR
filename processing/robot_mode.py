import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from localization.robot_pose import RobotPose
from visualization.display import display_info
import time

def process(frame, corners, ids, camera_matrix, dist_coeffs, config, robot_board):
    robot_indices = np.where(ids == config.robot_marker_id)[0]
    if len(robot_indices) == 0:
        display_info(frame, [f"Robot marker (ID: {config.robot_marker_id}) not detected"], color=(0, 0, 255))
        return None
    
    try:
        obj_points = robot_board.getObjPoints()
        success, rvec_robot, tvec_robot = cv2.solvePnP(
            obj_points[0],
            corners[robot_indices[0]], 
            camera_matrix, 
            dist_coeffs
        )
        if not success:
            display_info(frame, ["Failed to estimate robot pose"], color=(0, 0, 255))
            return None

        R_cr = R.from_rotvec(rvec_robot.flatten())
        t_cr = tvec_robot.flatten()

        info = [
            f"Robot marker detected! ID: {config.robot_marker_id}",
            f"Position: [{t_cr[0]:.3f}, {t_cr[1]:.3f}, {t_cr[2]:.3f}]",
            f"Orientation: [{R_cr.as_euler('xyz', degrees=True)[0]:.2f}, {R_cr.as_euler('xyz', degrees=True)[1]:.2f}, {R_cr.as_euler('xyz', degrees=True)[2]:.2f}]"
        ]
        display_info(frame, info)

        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_robot, tvec_robot, 100)

        return RobotPose(tuple(t_cr), R_cr, time.time())
    except Exception as e:
        error_msg = f"Error estimating robot pose: {str(e)}"
        print(error_msg)  # Log to console
        display_info(frame, [error_msg], color=(0, 0, 255))
        return None