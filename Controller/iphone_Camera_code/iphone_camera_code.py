import cv2
import numpy as np

def add_5x5_grid(frame):
    """
    Adds a 5x5 grid overlay to the frame.
    
    :param frame: The frame to which the grid will be added.
    :return: The frame with the 5x5 grid overlay.
    """
    height, width, _ = frame.shape
    
    # Calculate the spacing for the grid lines
    x_spacing = width // 5
    y_spacing = height // 5
    
    # Draw vertical lines
    for x in range(0, width, x_spacing):
        cv2.line(frame, (x, 0), (x, height), (0, 255, 0), 1)
    
    # Draw horizontal lines
    for y in range(0, height, y_spacing):
        cv2.line(frame, (0, y), (width, y), (0, 255, 0), 1)
    
    return frame

def detect_x_marks(frame, template):
    """
    Detects 'X' marks in the frame using template matching and prints their locations.
    
    :param frame: The frame in which to detect 'X' marks.
    :param template: The template image of the 'X' mark.
    :return: The frame with 'X' marks highlighted.
    """
    # Convert both the frame and template to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    
    # Get the dimensions of the template
    h, w = gray_template.shape
    
    # Perform template matching
    result = cv2.matchTemplate(gray_frame, gray_template, cv2.TM_CCOEFF_NORMED)
    
    # Set a threshold for detection
    threshold = 0.8
    locations = np.where(result >= threshold)
    
    # Draw rectangles around detected 'X' marks
    for pt in zip(*locations[::-1]):  # Switch columns and rows
        cv2.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)
        print(f"Found 'X' mark at: ({pt[0] + w // 2}, {pt[1] + h // 2})")
    
    return frame

# Load the template image of the 'X' mark
template = cv2.imread('D:\CAPSTONE PROJECT\download.jpg')  # Replace with the path to your 'X' mark template

if template is None:
    print("Error: Could not load template image.")
    exit()

# Attempt to open EpocCam (try different indices if necessary)
cap = cv2.VideoCapture(2)  # Replace 0 with 1, 2, etc., if multiple cameras are connected

if not cap.isOpened():
    print("Error: Cannot access the camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read frame.")
        break

    # Add 5x5 grid to the frame
    frame_with_grid = add_5x5_grid(frame)

    # Detect 'X' marks in the frame
    frame_with_x_marks = detect_x_marks(frame_with_grid, template)

    # Display the video frame with the grid and 'X' marks
    cv2.imshow("EpocCam Stream with 5x5 Grid and X Marks", frame_with_x_marks)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()