import numpy as np
import cv2

def generate_chessboard(square_size, width, height):
    board = np.zeros((height*square_size, width*square_size), dtype=np.uint8)
    
    for i in range(height):
        for j in range(width):
            if (i + j) % 2 == 0:
                board[i*square_size:(i+1)*square_size, j*square_size:(j+1)*square_size] = 255
    
    return board

if __name__ == "__main__":
    square_size = 100
    width = 10
    height = 7

    chessboard = generate_chessboard(square_size, width, height)

    cv2.imwrite('images/markers/chessboard.png', chessboard)

    cv2.imshow('Chessboard', chessboard)
    cv2.waitKey(0)
    cv2.destroyAllWindows()