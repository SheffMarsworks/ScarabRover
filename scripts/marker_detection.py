import cv2
import numpy as np

# Initialize webcam
capture = cv2.VideoCapture(0)
frame_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Camera Frame Size: {frame_width}x{frame_height}")

# Load ArUco dictionary and detector parameters
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
parameters = cv2.aruco.DetectorParameters()

# Aruco detector
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

while capture.isOpened():
    ret, frame = capture.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(ids)):
            # Get the four corner points
            c = corners[i][0]  # Shape: (4,2) -> 4 points (x, y)

            # Compute the center of the marker
            center_x = int(np.mean(c[:, 0]))  # Average of all x-coordinates
            center_y = int(np.mean(c[:, 1]))  # Average of all y-coordinates

            # Draw the center of the marker
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Find its position
            if center_x<(frame_width//2-100):
                print("Marker", ids[i], "detected on the left")
            elif center_x>(frame_width//2+100):
                print("Marker", ids[i], "detected on the right")
            else:
                print("Marker", ids[i], "detected in the center")

    # Center Lines
    cv2.line(frame, (frame_width//2-100, 0), (frame_width//2-100, frame_height), (0, 255, 0), 2)
    cv2.line(frame, (frame_width//2+100, 0), (frame_width//2+100, frame_height), (0, 255, 0), 2)

    cv2.imshow("Aruco Detection", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
        break

capture.release()
cv2.destroyAllWindows()
