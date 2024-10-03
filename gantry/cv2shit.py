import cv2
import numpy as np


def cvshit(ipt):
    # Load the input image
    image_path = ipt
    frame = cv2.imread(image_path)
    # cap = cv2.VideoCapture(0)  # Change the index to 1 or 2 if the webcam is not working with 0
    # if not cap.isOpened():
    #     print("Error: Could not open the camera.")
    #     exit()
    # ret, frame = cap.read()

    # Check if the image is loaded successfully
    if frame is None:
        print("Error: Could not load the image.")
        exit()

    # Convert image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Use GaussianBlur to reduce noise and improve contour detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Perform edge detection
    edges = cv2.Canny(blurred, threshold1=30, threshold2=150)

    # Dilate the edges to close gaps
    dilated = cv2.dilate(edges, None, iterations=2)

    # Find contours
    contours, _ = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes around each contour
    for contour in contours:
        # Apply contour approximation to reduce the number of points
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Get the bounding rectangle
        x, y, w, h = cv2.boundingRect(approx)

        # Draw a green rectangle around each detected object
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # Prepare the text to display
        text = f"x={x}, y={y}"

        # Calculate the position for the text (above the rectangle)
        text_position = (x, y - 10)

        # Draw the text on the frame
        cv2.putText(frame, text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 0), 3)


    # Create a window that can be resized
    # cv2.namedWindow("Object Detection", cv2.WINDOW_NORMAL)

    # Resize the window to fit the entire image
    # cv2.imshow("Object Detection", frame)

    # Wait until any key is pressed
    #cv2.waitKey(0)

    # Close the window
    #cv2.destroyAllWindows()

    # Convert image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Use GaussianBlur to reduce noise and improve contour detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Perform edge detection
    edges = cv2.Canny(blurred, threshold1=30, threshold2=150)

    # Dilate the edges to close gaps
    dilated = cv2.dilate(edges, None, iterations=2)

    # Find contours
    contours, _ = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes around each contour
    for contour in contours:
        # Apply contour approximation to reduce the number of points
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Get the bounding rectangle
        x, y, w, h = cv2.boundingRect(approx)

        # Draw a green rectangle around each detected object
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Save the processed image to a file
    output_filename = "new4.jpeg"
    cv2.imwrite(output_filename, frame)
    print(f"Image saved as {output_filename}")

    # # Create a window that can be resized
    # cv2.namedWindow("Object Detection", cv2.WINDOW_NORMAL)

    # # Resize the window to fit the entire image
    # cv2.imshow("Object Detection", frame)

    # # Wait until any key is pressed
    # cv2.waitKey(0)

    # # Close the window
    # cv2.destroyAllWindows()