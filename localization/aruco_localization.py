import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from .robot_pose import RobotPose
from utils.config import Config
from utils.markers import MarkerBoards
from visualization.display import display_info, resize_frame
from camera.camera_utils import get_ip_webcam_frame, setup_window
from processing import robot_mode, board_mode, all_mode
from .heading import get_heading_angle


class ArucoLocalization:
    def __init__(self, config: Config):
        self.config = config
        self.camera_matrix = np.load(config.camera_matrix_file)
        self.dist_coeffs = np.load(config.dist_coeffs_file)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.marker_boards = MarkerBoards(self.aruco_dict, config.marker_length)
        self.board = self.marker_boards.get_board()
        self.robot_board = self.marker_boards.get_robot_board()
        self.ser = self.setup_serial() if self.config.use_serial else None
        self.window_name = 'ArUco Localization'
        setup_window(self.window_name)
        cv2.waitKey(1)  # Ajoutez cette ligne pour s'assurer que la fenêtre est créée

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

    def process_frame(self, frame: np.ndarray, mode: str) -> RobotPose:
        corners, ids, _ = self.aruco_detector.detectMarkers(frame)
        
        if ids is None or len(ids) == 0:
            display_info(frame, ["No markers detected"], color=(0, 0, 255))
            return None

        if mode == "robot":
            return robot_mode.process(frame, corners, ids, self.camera_matrix, self.dist_coeffs,
                                      self.config, self.robot_board)
        elif mode == "board":
            return board_mode.process(frame, corners, ids, self.camera_matrix, self.dist_coeffs,
                                      self.config, self.board)
        elif mode == "all":
            return all_mode.process(frame, corners, ids, self.camera_matrix, self.dist_coeffs, 
                                    self.config, self.board, self.robot_board)

    def display_pose(self, pose: RobotPose, mode: str):
        if mode == "robot":
            print(f"\nRobot pose in camera frame:")
            print(f"Position: ({pose.position[0]:.6f}, {pose.position[1]:.6f}, {pose.position[2]:.6f})")
            print(f"Orientation (Euler angles): {pose.orientation.as_euler('xyz', degrees=True)}")
            heading_angles = get_heading_angle(pose.orientation)
            print(f"Heading (degrees):")
            print(f"  Quaternion: {heading_angles['quaternion']:.2f}")
            print(f"  Matrix: {heading_angles['matrix']:.2f}")
            print(f"  Euler: {heading_angles['euler']:.2f}")
            print(f"  Rotvec: {heading_angles['rotvec']:.2f}")
        
        elif mode == "board":
            print(f"\nBoard pose in camera frame:")
            print(f"Position: ({pose.position[0]:.6f}, {pose.position[1]:.6f}, {pose.position[2]:.6f})")
            print(f"Orientation (Euler angles): {pose.orientation.as_euler('xyz', degrees=True)}")
        
        elif mode == "all":
            print(f"\nRobot pose in board frame:")
            print(f"Position: ({pose.position[0]:.6f}, {pose.position[1]:.6f}, {pose.position[2]:.6f})")
            print(f"Orientation (Euler angles): {pose.orientation.as_euler('xyz', degrees=True)}")
            heading_angles = get_heading_angle(pose.orientation)
            print(f"Heading (degrees):")
            print(f"  Quaternion: {heading_angles['quaternion']:.2f}")
            print(f"  Matrix: {heading_angles['matrix']:.2f}")
            print(f"  Euler: {heading_angles['euler']:.2f}")
            print(f"  Rotvec: {heading_angles['rotvec']:.2f}")

    def send_pose(self, pose: RobotPose):
        if self.ser is None:
            return
        heading_angles = get_heading_angle(pose.orientation)
        data = f"{pose.position[0]:.3f};{pose.position[1]:.3f};{pose.position[2]:.3f};" \
               f"{heading_angles['quaternion']:.3f};{heading_angles['matrix']:.3f};" \
               f"{heading_angles['euler']:.3f};{heading_angles['rotvec']:.3f}"
        self.ser.write(data.encode())

    def run(self, mode: str):
        while True:
            ret, frame = get_ip_webcam_frame(self.config.camera_url)
            if not ret:
                print("Failed to grab frame")
                continue

            pose = self.process_frame(frame, mode)
            if pose:
                self.display_pose(pose, mode)
                if mode == "robot":
                    self.send_pose(pose)

            resized_frame = resize_frame(frame, self.window_name)
            cv2.imshow(self.window_name, resized_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        if self.ser:
            self.ser.close()
