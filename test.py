import cv2
cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)


ret1, frame1 = cap1.read()
ret2, frame2 = cap2.read()

# Check if the frame was captured successfully
if not ret1:
    print("Failed to capture frame.")


else: cv2.imwrite('cam0.png', frame1)
cap1.release()

if not ret2:
    print("Failed to capture frame.")
else: cv2.imwrite('cam1.png', frame2)
cap2.release()