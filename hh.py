import cv2
import numpy as np
 
img = cv2.imread("C://Users//PC//Desktop//dataset//project//odm_orthophoto//odm_orthophoto.tif")
# Split the frame into individual color channels
blue, green, red = cv2.split(img)

# Calculate the (blue - red) / (blue + red) ratio with handling division by zero
with np.errstate(divide='ignore', invalid='ignore'):
    ratio = np.divide(-(blue.astype(np.float32) - red.astype(np.float32)), (blue.astype(np.float32) + red.astype(np.float32)))
    ratio[np.isnan(ratio)] = 0  # Replace NaN (result of 0/0) with 0

# Normalize the ratio to the range [0, 255]
ratio_normalized = cv2.normalize(ratio, None, 0, 255, cv2.NORM_MINMAX)

# Apply a color map to enhance visualization
ratio_colormap = cv2.applyColorMap(ratio_normalized.astype(np.uint8), cv2.COLORMAP_JET )

# Display the frame with the calculated ratio
cv2.imwrite('Frame.png', ratio_colormap)


