import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from localization.robot_pose import RobotPose
from visualization.display import display_info
import time

def process(frame, corners, ids, camera_matrix, dist_coeffs, config, board):
    board_indices = np.isin(ids, config.fixed_marker_ids)
    if not np.any(board_indices):
        display_info(frame, ["No board markers detected"], color=(0, 0, 255))
        return None
    
    board_corners = [corners[i] for i in range(len(ids)) if board_indices[i]]
    board_ids = ids[board_indices]

    try:
        obj_points, img_points = board.matchImagePoints(board_corners, board_ids)
        if obj_points is None or len(obj_points) == 0:
            display_info(frame, ["Not enough matches found with the board"], color=(0, 0, 255))
            return None
    except cv2.error as e:
        display_info(frame, [f"Error matching points: {e}"], color=(0, 0, 255))
        return None
    
    try:
        success, rvec_board, tvec_board = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
        if not success:
            display_info(frame, ["Failed to estimate board pose"], color=(0, 0, 255))
            return None
    except cv2.error as e:
        error_msg = f"Error estimating robot pose: {str(e)}"
        print(error_msg)  # Log to console
        display_info(frame, [error_msg], color=(0, 0, 255))
        return None
    
    R_cw = R.from_rotvec(rvec_board.flatten())
    t_cw = tvec_board.flatten()

    info = [
        "Board pose estimated successfully",
        f"Position: [{t_cw[0]:.3f}, {t_cw[1]:.3f}, {t_cw[2]:.3f}]",
        f"Orientation: [{R_cw.as_euler('xyz', degrees=True)[0]:.2f}, {R_cw.as_euler('xyz', degrees=True)[1]:.2f}, {R_cw.as_euler('xyz', degrees=True)[2]:.2f}]"
    ]
    display_info(frame, info)

    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_board, tvec_board, 100)

    return RobotPose(tuple(t_cw), R_cw, time.time())