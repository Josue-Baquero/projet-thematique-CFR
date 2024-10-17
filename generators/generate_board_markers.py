import cv2
import numpy as np
from cv2 import aruco

def create_aruco_board():
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    marker_ids = np.array([20, 21, 22, 23], dtype=np.int32)
    
    marker_length = 0.2
    marker_positions = [
        np.array([
            [0, 0, 0],
            [marker_length, 0, 0],
            [marker_length, marker_length, 0],
            [0, marker_length, 0]
        ], dtype=np.float32) for _ in range(4)
    ]
    
    marker_positions[1][:, 0] += 0.8
    marker_positions[2][:, 1] += 0.8
    marker_positions[3][:, 0] += 0.8
    marker_positions[3][:, 1] += 0.8
    
    board = aruco.Board(marker_positions, aruco_dict, marker_ids)
    
    return board

def display_board(board):
    img_size = (1000, 1000)
    board_image = board.generateImage(img_size)
    
    border_size = 20
    board_image_with_border = cv2.copyMakeBorder(
        board_image, 
        border_size, border_size, border_size, border_size, 
        cv2.BORDER_CONSTANT, 
        value=[255, 255, 255]
    )
    
    cv2.imshow("ArUco Board", board_image_with_border)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    cv2.imwrite("images/markers/board_markers.png", board_image_with_border)

if __name__ == "__main__":
    board = create_aruco_board()
    display_board(board)