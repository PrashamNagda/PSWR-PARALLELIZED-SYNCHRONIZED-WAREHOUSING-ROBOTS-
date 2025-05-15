import cv2
import cv2.aruco as aruco

# Define the ArUco marker IDs for the start and end points
START_POINT_MARKER_ID = 23
END_POINT_MARKER_ID = 24

# Initialize the ArUco dictionary and parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Initialize the video capture (use 0 for the default camera)
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Loop through all detected markers
        for i in range(len(ids)):
            marker_id = ids[i][0]
            marker_corners = corners[i][0]

            # Draw the detected marker
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            # Check if the detected marker is the "start point"
            if marker_id == START_POINT_MARKER_ID:
                # Draw a bounding box around the marker
                cv2.polylines(frame, [marker_corners.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)

                # Display a message
                cv2.putText(frame, "Start Point Detected!", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Print to console
                print("Start Point Detected!")

            # Check if the detected marker is the "end point"
            elif marker_id == END_POINT_MARKER_ID:
                # Draw a bounding box around the marker
                cv2.polylines(frame, [marker_corners.astype(int)], isClosed=True, color=(0, 0, 255), thickness=2)

                # Display a message
                cv2.putText(frame, "End Point Detected!", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Print to console
                print("End Point Detected!")

    # Display the frame
    cv2.imshow("ArUco Marker Detection", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()