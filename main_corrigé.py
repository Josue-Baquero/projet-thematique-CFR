import cv2
import numpy as np
import time
from dataclasses import dataclass
from typing import Tuple, Optional
import yaml
from scipy.spatial.transform import Rotation as R
import argparse
import requests
from screeninfo import get_monitors

@dataclass
class RobotPose:
    position: Tuple[float, float, float]
    orientation: R
    timestamp: float

class Config:
    def __init__(self, config_file: str):
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        self.camera_matrix_file = config['camera_matrix_file']
        self.dist_coeffs_file = config['dist_coeffs_file']
        self.marker_length = config['marker_length']
        self.fixed_marker_ids = config['fixed_marker_ids']
        self.robot_marker_id = config['robot_marker_id']
        self.camera_url = config['camera_url']
        self.use_serial = config.get('use_serial', False)
        if self.use_serial:
            self.serial_port = config['serial_port']
            self.serial_baudrate = config['serial_baudrate']

class ArucoLocalization:
    def __init__(self, config: Config):
        self.config = config
        self.camera_matrix = np.load(config.camera_matrix_file)
        self.dist_coeffs = np.load(config.dist_coeffs_file)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.board = self.setup_board()
        if self.config.use_serial:
            self.ser = self.setup_serial()
        else:
            self.ser = None
        self.window_name = 'ArUco Localization'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.adjust_window_size()

    def adjust_window_size(self):
        # Obtenir la taille de l'écran principal
        screen = get_monitors()[0]
        screen_width, screen_height = screen.width, screen.height

        # Calculer la taille de la fenêtre (80% de la taille de l'écran)
        window_width = int(screen_width * 0.8)
        window_height = int(screen_height * 0.8)

        # Définir la taille de la fenêtre
        cv2.resizeWindow(self.window_name, window_width, window_height)

    def resize_frame(self, frame):
        # Obtenir la taille actuelle de la fenêtre
        window_width = cv2.getWindowImageRect(self.window_name)[2]
        window_height = cv2.getWindowImageRect(self.window_name)[3]

        # Redimensionner l'image pour qu'elle s'adapte à la fenêtre
        return cv2.resize(frame, (window_width, window_height))

    def setup_board(self):
        objPoints = np.array([
            [[-0.1, -0.1, 0], [0.1, -0.1, 0], [0.1, 0.1, 0], [-0.1, 0.1, 0]],  # Marker 20
            [[0.2, -0.1, 0], [0.4, -0.1, 0], [0.4, 0.1, 0], [0.2, 0.1, 0]],    # Marker 21
            [[-0.1, 0.2, 0], [0.1, 0.2, 0], [0.1, 0.4, 0], [-0.1, 0.4, 0]],    # Marker 22
            [[0.2, 0.2, 0], [0.4, 0.2, 0], [0.4, 0.4, 0], [0.2, 0.4, 0]]       # Marker 23
        ], dtype=np.float32)
        return cv2.aruco.Board(objPoints, self.aruco_dict, np.array([20, 21, 22, 23]))

    def setup_serial(self):
        import serial
        try:
            return serial.Serial(self.config.serial_port, self.config.serial_baudrate)
        except serial.SerialException as e:
            print(f"Erreur lors de l'ouverture du port série : {e}")
            print("Le programme continuera sans communication série.")
            return None

    def get_ip_webcam_frame(self):
        try:
            response = requests.get(self.config.camera_url, stream=True)
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

    def display_info(self, frame, info_list, color=(0, 255, 0)):
        # Add semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (600, 30 * len(info_list) + 20), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Display information
        for i, info in enumerate(info_list):
            cv2.putText(frame, info, (20, 40 + 30*i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    def process_frame(self, frame: np.ndarray) -> Optional[RobotPose]:
        corners, ids, rejected = self.aruco_detector.detectMarkers(frame)
        
        if ids is None or len(ids) == 0:
            self.display_info(frame, ["No markers detected"], color=(0, 0, 255))
            return None

        if args.mode == "robot":
            robot_indices = np.where(ids == self.config.robot_marker_id)[0]
            if len(robot_indices) == 0:
                self.display_info(frame, [f"Robot marker (ID: {self.config.robot_marker_id}) not detected"], color=(0, 0, 255))
                return None
            corners = [corners[i] for i in robot_indices]
            ids = ids[robot_indices]
            
            try:
                success, rvec_robot, tvec_robot = cv2.solvePnP(
                    self.board.getObjPoints()[0],
                    corners[0], 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                if not success:
                    self.display_info(frame, ["Failed to estimate robot pose"], color=(0, 0, 255))
                    return None
                
                R_cr = R.from_rotvec(rvec_robot.flatten())
                t_cr = tvec_robot.flatten()
                
                self.last_rvec_robot = rvec_robot
                self.last_tvec_robot = tvec_robot
                
                info = [
                    f"Robot marker detected! ID: {self.config.robot_marker_id}",
                    f"Position: [{t_cr[0]:.3f}, {t_cr[1]:.3f}, {t_cr[2]:.3f}]",
                    f"Orientation: [{R_cr.as_euler('xyz', degrees=True)[0]:.2f}, {R_cr.as_euler('xyz', degrees=True)[1]:.2f}, {R_cr.as_euler('xyz', degrees=True)[2]:.2f}]"
                ]
                self.display_info(frame, info)
                
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec_robot, tvec_robot, 0.1)
                
                return RobotPose(tuple(t_cr), R_cr, time.time())
            except Exception as e:
                self.display_info(frame, [f"Error estimating robot pose: {e}"], color=(0, 0, 255))
                return None
        
        elif args.mode == "table":
            table_indices = np.isin(ids, self.config.fixed_marker_ids)
            if not np.any(table_indices):
                self.display_info(frame, ["No table markers detected"], color=(0, 0, 255))
                return None
            corners = [corners[i] for i in range(len(ids)) if table_indices[i]]
            ids = ids[table_indices]

            try:
                obj_points, img_points = self.board.matchImagePoints(corners, ids)
                if obj_points is None or len(obj_points) == 0:
                    self.display_info(frame, ["Not enough matches found with the board"], color=(0, 0, 255))
                    return None
            except cv2.error as e:
                self.display_info(frame, [f"Error matching points: {e}"], color=(0, 0, 255))
                return None
            
            try:
                success, rvec_board, tvec_board = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)
                if not success:
                    self.display_info(frame, ["Failed to estimate board pose"], color=(0, 0, 255))
                    return None
            except cv2.error as e:
                self.display_info(frame, [f"Error estimating board pose: {e}"], color=(0, 0, 255))
                return None

            R_cw = R.from_rotvec(rvec_board.flatten())
            t_cw = tvec_board.flatten()

            self.last_rvec_board = rvec_board
            self.last_tvec_board = tvec_board

            info = [
                "Board pose estimated successfully",
                f"Position: [{t_cw[0]:.3f}, {t_cw[1]:.3f}, {t_cw[2]:.3f}]",
                f"Orientation: [{R_cw.as_euler('xyz', degrees=True)[0]:.2f}, {R_cw.as_euler('xyz', degrees=True)[1]:.2f}, {R_cw.as_euler('xyz', degrees=True)[2]:.2f}]"
            ]
            self.display_info(frame, info)

            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec_board, tvec_board, 0.1)

            return RobotPose(tuple(t_cw), R_cw, time.time())

        elif args.mode == "all":
            # Estimer la pose du board
            board_indices = np.isin(ids, self.config.fixed_marker_ids)
            if not np.any(board_indices):
                self.display_info(frame, ["No table markers detected"], color=(0, 0, 255))
                return None

            board_corners = [corners[i] for i in range(len(ids)) if board_indices[i]]
            board_ids = ids[board_indices]

            try:
                obj_points, img_points = self.board.matchImagePoints(board_corners, board_ids)
                success_board, rvec_board, tvec_board = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)

                if not success_board:
                    self.display_info(frame, ["Failed to estimate board pose"], color=(0, 0, 255))
                    return None
            except Exception as e:
                self.display_info(frame, [f"Error estimating board pose: {e}"], color=(0, 0, 255))
                return None

            # Estimer la pose du robot
            robot_indices = np.where(ids == self.config.robot_marker_id)[0]
            if len(robot_indices) == 0:
                self.display_info(frame, [f"Robot marker (ID: {self.config.robot_marker_id}) not detected"], color=(0, 0, 255))
                return None

            try:
                success_robot, rvec_robot, tvec_robot = cv2.solvePnP(
                    self.board.getObjPoints()[0],
                    corners[robot_indices[0]], 
                    self.camera_matrix, 
                    self.dist_coeffs
                )

                if not success_robot:
                    self.display_info(frame, ["Failed to estimate robot pose"], color=(0, 0, 255))
                    return None
            except Exception as e:
                self.display_info(frame, [f"Error estimating robot pose: {e}"], color=(0, 0, 255))
                return None

            # Calculer la pose relative du robot par rapport au board
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

            T_wr = np.linalg.inv(T_cw) @ T_cr

            # Extraire la position et l'orientation relatives
            t_wr = T_wr[:3, 3]
            R_wr = R.from_matrix(T_wr[:3, :3])

            # Calculer le heading du robot par rapport au board
            heading_wr = self.get_heading_angle(R_wr)

            # Préparer les informations à afficher
            info = [
                "All mode: Board and Robot poses estimated",
                f"Robot position (board frame): [{t_wr[0]:.3f}, {t_wr[1]:.3f}, {t_wr[2]:.3f}]",
                f"Robot orientation (board frame): [{R_wr.as_euler('xyz', degrees=True)[0]:.2f}, {R_wr.as_euler('xyz', degrees=True)[1]:.2f}, {R_wr.as_euler('xyz', degrees=True)[2]:.2f}]",
                f"Robot heading (board frame): {heading_wr['euler']:.2f} degrees"
            ]
            self.display_info(frame, info)

            # Dessiner les axes pour le board et le robot
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec_board, tvec_board, 0.1)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec_robot, tvec_robot, 0.1)

            return RobotPose(tuple(t_wr), R_wr, time.time())


    def get_heading_angle(self, orientation: R) -> dict:
        # Method 1: Using quaternions
        quat = orientation.as_quat()
        heading_quat = np.arctan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]),
                                  1 - 2 * (quat[1]**2 + quat[2]**2))

        # Method 2: Using rotation matrix
        rot_mat = orientation.as_matrix()
        heading_matrix = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])

        # Method 3: Using Euler angles
        euler = orientation.as_euler('zyx', degrees=True)
        heading_euler = euler[0]

        # Method 4: Using rotation vector
        rotvec = orientation.as_rotvec()
        heading_rotvec = np.arctan2(rotvec[1], rotvec[0])

        return {
            'quaternion': np.degrees(heading_quat),
            'matrix': np.degrees(heading_matrix),
            'euler': heading_euler,
            'rotvec': np.degrees(heading_rotvec)
        }

    def run(self):
        while True:
            ret, frame = self.get_ip_webcam_frame()
            if not ret:
                print("Failed to grab frame")
                continue

            pose = self.process_frame(frame)
            if pose:
                if args.mode == "robot":
                    self.send_pose(pose)
                    heading_angles = self.get_heading_angle(pose.orientation)
                    print(f"\nRobot pose in camera frame:")
                    print(f"Position: ({pose.position[0]:.6f}, {pose.position[1]:.6f}, {pose.position[2]:.6f})")
                    print(f"Orientation (Euler angles): {pose.orientation.as_euler('xyz', degrees=True)}")
                    print(f"Heading (degrees):")
                    print(f"  Quaternion: {heading_angles['quaternion']:.2f}")
                    print(f"  Matrix: {heading_angles['matrix']:.2f}")
                    print(f"  Euler: {heading_angles['euler']:.2f}")
                    print(f"  Rotvec: {heading_angles['rotvec']:.2f}")
                
                elif args.mode == "table":
                    print(f"\nBoard pose in camera frame:")
                    print(f"Position: ({pose.position[0]:.6f}, {pose.position[1]:.6f}, {pose.position[2]:.6f})")
                    print(f"Orientation (Euler angles): {pose.orientation.as_euler('xyz', degrees=True)}")
                
                elif args.mode == "all":
                    print(f"\nRobot pose in board frame:")
                    print(f"Position: ({pose.position[0]:.6f}, {pose.position[1]:.6f}, {pose.position[2]:.6f})")
                    print(f"Orientation (Euler angles): {pose.orientation.as_euler('xyz', degrees=True)}")
                    heading_angles = self.get_heading_angle(pose.orientation)
                    print(f"Heading (degrees):")
                    print(f"  Quaternion: {heading_angles['quaternion']:.2f}")
                    print(f"  Matrix: {heading_angles['matrix']:.2f}")
                    print(f"  Euler: {heading_angles['euler']:.2f}")
                    print(f"  Rotvec: {heading_angles['rotvec']:.2f}")
                
                # Draw the axis for the ArUco marker
                if args.mode == "robot" and hasattr(self, 'last_rvec_robot') and hasattr(self, 'last_tvec_robot'):
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, 
                                    self.last_rvec_robot, self.last_tvec_robot, 0.1)
                elif args.mode == "table" and hasattr(self, 'last_rvec_board') and hasattr(self, 'last_tvec_board'):
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, 
                                    self.last_rvec_board, self.last_tvec_board, 0.1)

            # Redimensionner le frame pour s'adapter à la taille de la fenêtre
            resized_frame = self.resize_frame(frame)
            cv2.imshow(self.window_name, resized_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('f'):
                # Basculer en mode plein écran
                if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN) == cv2.WINDOW_FULLSCREEN:
                    cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                else:
                    cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        cv2.destroyAllWindows()
        if self.ser:
            self.ser.close()

    def send_pose(self, pose: RobotPose):
        if self.ser is None:
            return
        heading_angles = self.get_heading_angle(pose.orientation)
        data = f"{pose.position[0]:.3f};{pose.position[1]:.3f};{pose.position[2]:.3f};" \
               f"{heading_angles['quaternion']:.3f};{heading_angles['matrix']:.3f};" \
               f"{heading_angles['euler']:.3f};{heading_angles['rotvec']:.3f}"
        self.ser.write(data.encode())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ArUco marker detection")
    parser.add_argument("--mode", choices=["robot", "table", "all"], default="all", 
                        help="Detection mode: robot (only robot marker), table (only table markers), or all (default)")
    args = parser.parse_args()

    config = Config('config.yaml')
    localization = ArucoLocalization(config)
    localization.run()