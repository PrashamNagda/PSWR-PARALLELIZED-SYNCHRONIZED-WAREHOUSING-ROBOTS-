import cv2
import cv2.aruco as aruco

# Define the parameters for the second ArUco marker
marker_id = 24  # New ID for the second marker (0-1023 for 4x4 markers)
marker_size = 200  # Size of the marker in pixels
output_file = r"D:\CAPSTONE PROJECT\ArucoTags\aruco_marker_24.png"  # New output file name with the complete path

# Create a dictionary of ArUco markers
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Generate the marker image
marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# Save the marker image to a file
cv2.imwrite(output_file, marker_image)

# Display the marker image
cv2.imshow("ArUco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(f"ArUco marker saved as {output_file}")