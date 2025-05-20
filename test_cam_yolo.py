import cv2
import numpy as np

img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.putText(img, "Test OpenCV GUI", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

cv2.imshow("Test GUI", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
