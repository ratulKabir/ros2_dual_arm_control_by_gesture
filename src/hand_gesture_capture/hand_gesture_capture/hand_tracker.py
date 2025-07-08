import numpy as np
import cv2
from collections import deque
import mediapipe as mp
from utils.utils_v2 import get_idx_to_coordinates, rescale_frame

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

def main():
    hands = mp_hands.Hands(
        max_num_hands=2,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7)

    hand_landmark_drawing_spec = mp_drawing.DrawingSpec(thickness=5, circle_radius=5)
    hand_connection_drawing_spec = mp_drawing.DrawingSpec(thickness=2, circle_radius=2)

    cap = cv2.VideoCapture(0)

    # Trails for each fingertip per hand
    pts_left_index = deque(maxlen=64)
    pts_left_thumb = deque(maxlen=64)
    pts_right_index = deque(maxlen=64)
    pts_right_thumb = deque(maxlen=64)

    while cap.isOpened():
        ret, image = cap.read()
        if not ret:
            break

        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)

        image.flags.writeable = True
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                label = handedness.classification[0].label  # 'Left' or 'Right'

                mp_drawing.draw_landmarks(
                    image=image,
                    landmark_list=hand_landmarks,
                    connections=mp_hands.HAND_CONNECTIONS,
                    landmark_drawing_spec=hand_landmark_drawing_spec,
                    connection_drawing_spec=hand_connection_drawing_spec)

                idx_to_coordinates = get_idx_to_coordinates(image, hand_landmarks)

                if 8 in idx_to_coordinates:  # Index tip
                    if label == 'Left':
                        pts_left_index.appendleft(idx_to_coordinates[8])
                    else:
                        pts_right_index.appendleft(idx_to_coordinates[8])
                if 4 in idx_to_coordinates:  # Thumb tip
                    if label == 'Left':
                        pts_left_thumb.appendleft(idx_to_coordinates[4])
                    else:
                        pts_right_thumb.appendleft(idx_to_coordinates[4])

        # Draw trails
        trails = [
            (pts_left_index, (255, 0, 0)),   # Blue
            (pts_left_thumb, (255, 0, 0)),
            (pts_right_index, (0, 255, 0)),  # Green
            (pts_right_thumb, (0, 255, 0)),
        ]

        for pts, color in trails:
            for i in range(1, len(pts)):
                if pts[i - 1] is None or pts[i] is None:
                    continue
                thickness = int(np.sqrt(len(pts) / float(i + 1)) * 4.5)
                cv2.line(image, pts[i - 1], pts[i], color, thickness)

        cv2.imshow("Hand Tracker - Index + Thumb", rescale_frame(image, percent=130))

        if cv2.waitKey(5) & 0xFF == 27:
            break

    hands.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()