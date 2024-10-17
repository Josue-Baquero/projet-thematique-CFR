import cv2
import numpy as np

def display_info(frame, info_list, color=(0, 255, 0)):
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (600, 30 * len(info_list) + 20), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    
    for i, info in enumerate(info_list):
        cv2.putText(frame, info, (20, 40 + 30*i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

def resize_frame(frame, window_name):
    try:
        window_rect = cv2.getWindowImageRect(window_name)
        if window_rect is not None:
            window_width, window_height = window_rect[2], window_rect[3]
            return cv2.resize(frame, (window_width, window_height))
    except cv2.error:
        pass  # La fenêtre n'existe pas encore
    return frame  # Retourner le frame original si la fenêtre n'existe pas