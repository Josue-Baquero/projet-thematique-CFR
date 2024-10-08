import cv2
import numpy as np
import time
from dataclasses import dataclass
from typing import Tuple, Optional
import yaml
from scipy.spatial.transform import Rotation as R
import argparse
import requests

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

    def process_frame(self, frame: np.ndarray) -> Optional[RobotPose]:
        corners, ids, rejected = self.aruco_detector.detectMarkers(frame)
        
        if ids is None or len(ids) == 0:
            print("Aucun marqueur détecté")
            return None

        # Filter markers based on mode
        if args.mode == "robot":
            robot_indices = np.where(ids == self.config.robot_marker_id)[0]
            if len(robot_indices) == 0:
                print(f"Marqueur robot (ID: {self.config.robot_marker_id}) non détecté")
                return None
            corners = [corners[i] for i in robot_indices]
            ids = ids[robot_indices]
            
            # Estimate robot pose directly without using the board
            try:
                success, rvec_robot, tvec_robot = cv2.solvePnP(
                    self.board.getObjPoints()[0],  # Assuming the first object point set is for the robot
                    corners[0], 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                if not success:
                    print("Échec de l'estimation de la pose du robot")
                    return None
                
                R_cr = R.from_rotvec(rvec_robot.flatten())
                t_cr = tvec_robot.flatten()
                
                print(f"Marqueur du robot détecté ! ID: {self.config.robot_marker_id}")
                print(f"Position (camera frame): {t_cr}")
                print(f"Orientation (camera frame): {R_cr.as_euler('xyz', degrees=True)}")
                
                return RobotPose(tuple(t_cr), R_cr, time.time())
            except Exception as e:
                print(f"Erreur lors de l'estimation de la pose du robot : {e}")
                return None
            
        elif args.mode == "table":
            table_indices = np.isin(ids, self.config.fixed_marker_ids)
            if not np.any(table_indices):
                print("Aucun marqueur de table détecté")
                return None
            corners = [corners[i] for i in range(len(ids)) if table_indices[i]]
            ids = ids[table_indices]

        # Estimate board pose (for table mode)
        try:
            obj_points, img_points = self.board.matchImagePoints(corners, ids)
            if obj_points is None or len(obj_points) == 0:
                print("Pas assez de correspondances trouvées avec le board")
                return None
        except cv2.error as e:
            print(f"Erreur lors de la correspondance des points : {e}")
            return None
        
        try:
            success, rvec_board, tvec_board = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)
            if not success:
                print("Échec de l'estimation de la pose du board")
                return None
        except cv2.error as e:
            print(f"Erreur lors de l'estimation de la pose du board : {e}")
            return None

        R_cw = R.from_rotvec(rvec_board.flatten())
        t_cw = tvec_board.flatten()

        print("Pose du board estimée avec succès")
        print(f"Position du board (camera frame): {t_cw}")
        print(f"Orientation du board (camera frame): {R_cw.as_euler('xyz', degrees=True)}")

        return RobotPose(tuple(t_cw), R_cw, time.time())

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

    def send_pose(self, pose: RobotPose):
        if self.ser is None:
            return
        heading_angles = self.get_heading_angle(pose.orientation)
        data = f"{pose.position[0]:.3f};{pose.position[1]:.3f};{pose.position[2]:.3f};" \
               f"{heading_angles['quaternion']:.3f};{heading_angles['matrix']:.3f};" \
               f"{heading_angles['euler']:.3f};{heading_angles['rotvec']:.3f}"
        self.ser.write(data.encode())

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
                print(f"Heading (degrees):")
                print(f"  Quaternion: {heading_angles['quaternion']:.2f}")
                print(f"  Matrix: {heading_angles['matrix']:.2f}")
                print(f"  Euler: {heading_angles['euler']:.2f}")
                print(f"  Rotvec: {heading_angles['rotvec']:.2f}")
                
                # Draw the axis for the ArUco marker
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, 
                                pose.orientation.as_rotvec(), pose.position, 0.1)
                
                # Draw marker information on the frame
                label = "Robot" if args.mode == "robot" else "Board"
                cv2.putText(frame, f"{label} Pose", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Pos: {pose.position[0]:.2f}, {pose.position[1]:.2f}, {pose.position[2]:.2f}", 
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Angle: {heading_angles['euler']:.2f}", (10, 90), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        if self.ser:
            self.ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ArUco marker detection")
    parser.add_argument("--mode", choices=["robot", "table", "all"], default="all", 
                        help="Detection mode: robot (only robot marker), table (only table markers), or all (default)")
    args = parser.parse_args()

    config = Config('config.yaml')
    localization = ArucoLocalization(config)
    localization.run()