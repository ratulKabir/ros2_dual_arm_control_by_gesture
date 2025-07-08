import numpy as np
import cv2
from collections import deque
import mediapipe as mp
from utils.utils_v2 import get_idx_to_coordinates, rescale_frame

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands


def midpoint(p1, p2):
    return int((p1[0] + p2[0]) / 2), int((p1[1] + p2[1]) / 2)


def euclidean(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def main():
    hands = mp_hands.Hands(
        max_num_hands=2,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7)

    cap = cv2.VideoCapture(0)

    # Center trails
    pts_left = deque(maxlen=64)
    pts_right = deque(maxlen=64)

    # Colors
    COLOR_LEFT = (255, 0, 0)
    COLOR_RIGHT = (0, 255, 0)
    GRIP_DISTANCE_THRESHOLD = 40  # pixels

    while cap.isOpened():
        ret, image = cap.read()
        if not ret:
            break

        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)

        image.flags.writeable = True
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        grip_status = {"Left": "Open", "Right": "Open"}

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                label = handedness.classification[0].label  # 'Left' or 'Right'
                idx_to_coordinates = get_idx_to_coordinates(image, hand_landmarks)

                if 8 in idx_to_coordinates and 4 in idx_to_coordinates:
                    p_index = idx_to_coordinates[8]
                    p_thumb = idx_to_coordinates[4]
                    center = midpoint(p_index, p_thumb)

                    dist = euclidean(p_index, p_thumb)
                    if dist < GRIP_DISTANCE_THRESHOLD:
                        grip_status[label] = "Grip"

                    if label == 'Left':
                        pts_left.appendleft(center)
                    else:
                        pts_right.appendleft(center)

        # Draw trails and centerpoints
        for pts, color, label in [(pts_left, COLOR_LEFT, "Left"), (pts_right, COLOR_RIGHT, "Right")]:
            for i in range(1, len(pts)):
                if pts[i - 1] is None or pts[i] is None:
                    continue
                thickness = int(np.sqrt(len(pts) / float(i + 1)) * 4.5)
                cv2.line(image, pts[i - 1], pts[i], color, thickness)

            # Draw current center
            if pts and pts[0] is not None:
                cv2.circle(image, pts[0], 8, color, -1)
                cv2.putText(image, f"{label}: {grip_status[label]}", (pts[0][0] + 10, pts[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        cv2.imshow("Hand Center Tracker", rescale_frame(image, percent=130))

        if cv2.waitKey(5) & 0xFF == 27:
            break

    hands.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()