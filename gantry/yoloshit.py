import cv2
import numpy as np

# Load YOLO model
net = cv2.dnn.readNet(r"darknet/yolov3.weights", r"darknet/cfg/yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Load class labels (COCO dataset)
with open(r"darknet/cfg/coco.data", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Initialize webcam (change 0 or 1 based on your camera)
ip_camera_url = "http://<ip-address>:<port>/video" 
cap = cv2.VideoCapture(ip_camera_url)  # Change the index to 1 or 2 if the webcam is not working with 0

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open the camera.")
    exit()

while True:
    # Capture frame from webcam
    ret, frame = cap.read()
    
    if not ret:
        print("Warning: Failed to grab frame. Retrying...")
        continue  # Skip to the next iteration if grabbing frame fails
    
    height, width, channels = frame.shape

    # Prepare the frame for YOLO
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Initialize lists to hold detection results
    class_ids = []
    confidences = []
    boxes = []

    # Process each detection
    for out in outs:
        for detection in out:
            scores = detection[5:]  # Object detection scores for each class
            class_id = np.argmax(scores)  # Get the highest scoring class
            confidence = scores[class_id]
            if confidence > 0.5:  # Only consider objects with confidence > 50%
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Calculate coordinates for bounding box
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Apply non-max suppression to remove duplicate boxes
    indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    # Draw bounding boxes
    if len(indices) > 0:
        indices = indices.flatten()  # Flatten indices if it's a list of lists
        for i in indices:
            box = boxes[i]
            x, y, w, h = box[0], box[1], box[2], box[3]

            # Draw bounding box and label
            label = str(classes[class_ids[i]])
            color = (0, 255, 0)  # Green bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # Display the result in a window
    cv2.imshow("Object Detection", frame)

    # Break the loop if the user presses the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()