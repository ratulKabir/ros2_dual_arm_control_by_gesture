import cv2
import numpy as np
from collections import deque
import mediapipe as mp
from hand_gesture_capture.utils.utils_v2 import get_idx_to_coordinates

mp_hands = mp.solutions.hands


def midpoint(p1, p2):
    return int((p1[0] + p2[0]) / 2), int((p1[1] + p2[1]) / 2)


def euclidean(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def normalize_point(i, j, width=640, height=480, range_val=3.5):
    x_norm = range_val * (2 * i - width) / width
    y_norm = range_val * (2 * j - height) / height
    return x_norm, y_norm


class HandTracker:
    def __init__(self, on_update_callback=None, threshold=40, visualize=True):
        self.hands = mp_hands.Hands(max_num_hands=2,
                                     min_detection_confidence=0.7,
                                     min_tracking_confidence=0.7)
        self.on_update_callback = on_update_callback
        self.threshold = threshold
        self.visualize = visualize
        self.cap = cv2.VideoCapture(0)

        # For visualization
        self.pts_left = deque(maxlen=64)
        self.pts_right = deque(maxlen=64)
        self.COLOR_LEFT = (255, 0, 0)
        self.COLOR_RIGHT = (0, 255, 0)

    def run(self):
        while self.cap.isOpened():
            ret, image = self.cap.read()
            if not ret:
                break
            image = cv2.flip(image, 1)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.hands.process(image_rgb)
            image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

            height, width, _ = image.shape
            hand_data = {}

            if results.multi_hand_landmarks and results.multi_handedness:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    label = handedness.classification[0].label  # 'Left' or 'Right'
                    idx_to_coordinates = get_idx_to_coordinates(image, hand_landmarks)

                    if 8 in idx_to_coordinates and 4 in idx_to_coordinates:
                        p_index = idx_to_coordinates[8]
                        p_thumb = idx_to_coordinates[4]
                        center = midpoint(p_index, p_thumb)
                        dist = euclidean(p_index, p_thumb)
                        grip = dist < self.threshold
                        x_norm, y_norm = normalize_point(center[0], center[1], width=width, height=height)
                        hand_data[label] = {
                            "center": (x_norm, y_norm),
                            "grip": grip,
                            "normalized": (x_norm, y_norm)
                        }

                        if self.visualize:
                            if label == 'Left':
                                self.pts_left.appendleft(center)
                            else:
                                self.pts_right.appendleft(center)

            if self.on_update_callback:
                self.on_update_callback(hand_data)

            if self.visualize:
                # Draw trails + current point
                for pts, color, label in [(self.pts_left, self.COLOR_LEFT, 'Left'),
                                          (self.pts_right, self.COLOR_RIGHT, 'Right')]:
                    for i in range(1, len(pts)):
                        if pts[i - 1] is None or pts[i] is None:
                            continue
                        thickness = int(np.sqrt(len(pts) / float(i + 1)) * 4.5)
                        cv2.line(image, pts[i - 1], pts[i], color, thickness)
                    if pts and pts[0] is not None:
                        cv2.circle(image, pts[0], 8, color, -1)
                        grip_text = hand_data.get(label, {}).get("grip", False)
                        status = "Grip" if grip_text else "Open"
                        cv2.putText(image, f"{label}: {status}", (pts[0][0] + 10, pts[0][1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                        # Draw normalized coordinates
                        norm_xy = hand_data.get(label, {}).get("normalized", None)
                        if norm_xy:
                            norm_text = f"({norm_xy[0]:.2f}, {norm_xy[1]:.2f})"
                            cv2.putText(image, norm_text, (pts[0][0] + 10, pts[0][1] + 20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                cv2.imshow("Hand Tracker", image)
                if cv2.waitKey(5) & 0xFF == 27:
                    break

        self.hands.close()
        self.cap.release()
        if self.visualize:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    def print_hand_data(data):
        print(data)

    tracker = HandTracker(on_update_callback=print_hand_data, visualize=True)
    tracker.run()
