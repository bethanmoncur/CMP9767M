import cv2
import numpy as np
from matplotlib import pyplot as plt


# --- import the grapes image and convert to HSV ---
image = cv2.imread("grape_image.jpg")
# apply blur to image
frame = cv2.GaussianBlur(image, (3, 3), 0)
# convert BGR to HSV for filtering
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# --- filter the image by colour of the grapes ---
# define the range of colour of the grapes in HSV
lower_blue = np.array([70, 20, 0])
upper_blue = np.array([235, 255, 255])
# threshold the image to get only the colour of the grapes
mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
# extract the grapes from the image
result = cv2.bitwise_and(image, image, mask=mask_blue)
# invert the mask to use it for contouring
mask_inv = 255 - mask_blue

# --- grape detection ---
# establish minimum area for a grape bunch
min_area = 150
# initialise variables to be outputted
found_grapes = False
count = 0
# convert from BGR to RGB for plotting
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
# get the contours using the mask from colour filtering
contours, hierarchy = cv2.findContours(mask_inv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(image, contours, -1, (255, 0, 0), 1)

# --- grape bunch counting ---
# iterate through the list of contours
for i, contour in enumerate(contours):
    # ignore boundary of image
    if hierarchy[0][i][3] >= 0:
        # if the contour is larger than the minimum area for a grape bunch
        if cv2.contourArea(contour) >= min_area:
            found_grapes = True
            grape_contour = contour
            # Get the coordinates of the grape centre
            M = cv2.moments(grape_contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            count += 1
            # Plot the grape centre point
            cv2.circle(image_rgb, (cx, cy), 4, (255, 0, 0), -1)

print(f"Number of grape bunches: {count}")

# # uncomment to display the colour mask, contours and detected grape bunches
# fig = plt.figure(figsize=(7, 7))
# fig.add_subplot(3, 1, 1)
# plt.imshow(mask_blue)
# plt.title(f"Colour thresholding using the inRange() function ")
# plt.axis('off')
# fig.add_subplot(3, 1, 2)
# plt.imshow(image)
# plt.title(f"Contours using the findContours() function")
# plt.axis('off')
# fig.add_subplot(3, 1, 3)
# plt.imshow(image_rgb)
# plt.title(f"Number of grapes: {count}")
# plt.axis('off')
# plt.show()
# cv2.waitKey(0)
# cv2.destroyAllWindows()


