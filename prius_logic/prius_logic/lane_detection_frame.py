####
# - Main purpse is Process A single frame to extract
#   line and frame mid points using image processing techniques
# - This scripts is Simple opencv Python based no ROS involved 
#
#  Written by Muhammad Luqman
#  ros2,FOXY
#  13/6/21
#
###
#!/usr/bin/env python3
import cv2
import numpy 

## Reading the image
image = cv2.imread('/home/utk/ros2_ws/src/my_prius/screenshot/left.png')
    ##### Segmentation
## Defining the color Upper bound and Lower bound limits to extract 
light_line = numpy.array([50,50,50])
dark_line = numpy.array([200,200,200])
mask = cv2.inRange(image, light_line,dark_line)

    ##### Boundaries Extraction
## applying the canny edge dector function
canny= cv2.Canny(mask,40,10)
## Cropping the image
r1=280;c1=0
width = 640
height = 110
img = canny[r1:r1 + height, c1:c1 + width]



   ##### Finding Line  Mid point
# Visually finding the perfect Row to fix for mid points
# by setting all values to be light
edge=[]
for i in range (width):
    if(img[int(height/2),i]==255):
        edge.append(i)
print(edge)

# Ensure there are enough edge points
if len(edge) < 2:
    print("Left_Turn")
    mid_point = None
else:
    # We only need two points: one from left line and one from right line
    if len(edge) == 4:
        edge[0] = edge[0]
        edge[1] = edge[2]
    elif len(edge) == 3:
        if edge[1] - edge[0] > 5:  # meaning idx(0) and idx(1) are ok [193, 506, 507]
            edge[0] = edge[0]
            edge[1] = edge[1]
        else:  # [193, 194, 507]
            edge[0] = edge[0]
            edge[1] = edge[2]

    # Calculate mid_area and mid_point only if edge has 2 elements
    if len(edge) >= 2:
        mid_area = (edge[1] - edge[0])
        mid_point = edge[0] + (mid_area / 2)
        img[int(height/2), int(mid_point)] = 255
    else:
        print("Left_Turn")
        mid_point = None

# Finding Frame Midpoint
frame_mid = width / 2

# Controlling the car process
if mid_point is not None:
    error = frame_mid - mid_point
    action = "Go Right" if error < 0 else "Go Left"
else:
    error = None
    action = "Left_Turn"

# More apparent mid frame pixel
img[int(height/2), int(frame_mid)] = 255
img[int(height/2)-1, int(frame_mid)] = 255
img[int(height/2)+1, int(frame_mid)] = 255

# Writing on the Frame as output for better understanding
f_image = cv2.putText(img, action, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

print("Error", error, " / Frame_mid", frame_mid, " / Mid_point", mid_point)

cv2.imshow('output image', f_image)
cv2.waitKey(0)
cv2.destroyAllWindows()