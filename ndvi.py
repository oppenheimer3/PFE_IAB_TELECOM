import cv2
import numpy as np

# Capture video from default camera (index 0)
cap = cv2.VideoCapture(0)

while True:
    # Read frame from the camera
    ret, frame = cap.read()

    # Split the frame into individual color channels
    blue, green, red = cv2.split(frame)

    # Calculate the (blue - red) / (blue + red) ratio with handling division by zero
    with np.errstate(divide='ignore', invalid='ignore'):
        ratio = np.divide((blue.astype(np.float32) - red.astype(np.float32)), (blue.astype(np.float32) + red.astype(np.float32)))
        ratio[np.isnan(ratio)] = 0  # Replace NaN (result of 0/0) with 0

    # Normalize the ratio to the range [0, 255]
    ratio_normalized = cv2.normalize(ratio, None, 0, 255, cv2.NORM_MINMAX)

    # Apply a color map to enhance visualization
    ratio_colormap = cv2.applyColorMap(ratio_normalized.astype(np.uint8), cv2.COLORMAP_JET )

    # Display the frame with the calculated ratio
    cv2.imshow('Frame', ratio_colormap)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
