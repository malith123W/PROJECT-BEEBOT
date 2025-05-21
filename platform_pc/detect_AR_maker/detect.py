import cv2
import numpy as np

cam = cv2.VideoCapture("http://192.168.137.177:4747/video")

# Parameters for saving the video (optional)
frameRate = 21
dispWidth = 640
dispHeight = 480

# Load the predefined dictionary (new method)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Initialize the detector parameters (new method)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Optional: Video writer (uncomment if needed)
# outVid = cv2.VideoWriter('videos/recordings.avi', cv2.VideoWriter_fourcc(*'XVID'), frameRate, (dispWidth, dispHeight))

while True:
    ret, frame = cam.read()
    if not ret:
        break

    # Detect markers (new method)
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
    print(markerIds)

    # Draw detected markers
    if markerCorners:
        for corners in markerCorners:
            pts = corners.reshape((-1, 1, 2)).astype(np.int32)
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 255), thickness=2)

    cv2.imshow('Cam', frame)

    # Optional: Save frame (uncomment if needed)
    # outVid.write(frame)

    if cv2.waitKey(1) == ord('q'):
        break

cam.release()
# outVid.release()  # Uncomment if saving video
cv2.destroyAllWindows()