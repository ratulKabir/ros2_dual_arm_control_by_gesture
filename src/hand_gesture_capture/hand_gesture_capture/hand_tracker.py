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

    pts_left = deque(maxlen=64)
    pts_right = deque(maxlen=64)

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

                if 8 in idx_to_coordinates:
                    if label == 'Left':
                        pts_left.appendleft(idx_to_coordinates[8])
                    elif label == 'Right':
                        pts_right.appendleft(idx_to_coordinates[8])

        # Draw trail lines
        for pts, color in zip([pts_left, pts_right], [(255, 0, 0), (0, 255, 0)]):  # Blue: Left, Green: Right
            for i in range(1, len(pts)):
                if pts[i - 1] is None or pts[i] is None:
                    continue
                thickness = int(np.sqrt(len(pts) / float(i + 1)) * 4.5)
                cv2.line(image, pts[i - 1], pts[i], color, thickness)

        cv2.imshow("Hands Tracker", rescale_frame(image, percent=130))

        if cv2.waitKey(5) & 0xFF == 27:
            break

    hands.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()