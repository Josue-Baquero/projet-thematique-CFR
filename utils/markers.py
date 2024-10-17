import cv2
import numpy as np

class MarkerBoards:
    def __init__(self, aruco_dict, marker_length):
        self.aruco_dict = aruco_dict
        self.marker_length = marker_length
        self.board = self.setup_board()
        self.robot_board = self.setup_robot_board()

    def setup_board(self):
        objPoints = np.array([
            [[-0.1, -0.1, 0], [0.1, -0.1, 0], [0.1, 0.1, 0], [-0.1, 0.1, 0]],  # Marker 20
            [[0.2, -0.1, 0], [0.4, -0.1, 0], [0.4, 0.1, 0], [0.2, 0.1, 0]],    # Marker 21
            [[-0.1, 0.2, 0], [0.1, 0.2, 0], [0.1, 0.4, 0], [-0.1, 0.4, 0]],    # Marker 22
            [[0.2, 0.2, 0], [0.4, 0.2, 0], [0.4, 0.4, 0], [0.2, 0.4, 0]]       # Marker 23
        ], dtype=np.float32)
        return cv2.aruco.Board(objPoints, self.aruco_dict, np.array([20, 21, 22, 23]))

    def setup_robot_board(self):
        objPoints = np.array([
            [-self.marker_length/2, self.marker_length/2, 0],
            [self.marker_length/2, self.marker_length/2, 0],
            [self.marker_length/2, -self.marker_length/2, 0],
            [-self.marker_length/2, -self.marker_length/2, 0]
        ], dtype=np.float32).reshape((1, 4, 3))
        return cv2.aruco.Board(objPoints, self.aruco_dict, np.array([36]))  # Assuming 36 is the robot marker ID

    def get_board(self):
        return self.board

    def get_robot_board(self):
        return self.robot_board