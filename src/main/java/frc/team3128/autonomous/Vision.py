import cv2
import numpy as np

# global variables go here:
testVar = 0

# To change a global variable inside a function,
# re-declare it with the 'global' keyword


text = "Hello, OpenCV!"
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
font_color = (255, 255, 255)  # White color in BGR format
thickness = 2


# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    # convert the input image to the HSV color space
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # convert the hsv to a binary image by removing any pixels
    # that do not fall within the following HSV Min/Max values
    r1_low = np.array([0,120,120]) # [0,120,120]  [0,30,20]
    r1_high = np.array([8,255,255]) # [8,255,255]   [20,255,255]
    img_threshold1 = cv2.inRange(img_hsv, r1_low, r1_high)


    r2_low = np.array([3,75,230]) # [3,75,230]
    r2_high = np.array([20,200,255]) # [20,160,255]
    img_threshold2 = cv2.inRange(img_hsv, r2_low, r2_high)

    r3_low = np.array([140,100,120]) # [177,100,120]   [160,100,20]
    r3_high = np.array([179,255,255]) # [179,255,255]   [179,255,255]
    img_threshold3 = cv2.inRange(img_hsv, r3_low, r3_high)


    img_threshold = cv2.bitwise_or(img_threshold1, img_threshold2)
    img_threshold = cv2.bitwise_or(img_threshold, img_threshold3)
    # find contours in the new binary image
    contourso, _ = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    sorted_contours = sorted(contourso, key=cv2.contourArea, reverse=True)

    largestContour = np.array([[]])
    # initialize an empty array of values to send back to the robot
    llpython = [0,0,0,0,0,0,0,0]
    # if contours have been detected, draw them
    for contour in sorted_contours:
        cv2.drawContours(image, contourso, -1, (224, 227, 39), 1)
        if cv2.contourArea(contour) <= 500:
            break
        
        # record the largest contour
        # largestContour = max(contourso, key=cv2.contourArea)

        # get the unrotated bounding box that surrounds the contour
        x,y,w,h = cv2.boundingRect(contour)

        # draw the unrotated bounding box
        # cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)

        polygonContour = cv2.convexHull(contour)

        if (len(polygonContour) >= 10):
            
            ellipse = cv2.fitEllipse(polygonContour)
            best_fit_ellipse_area = np.pi  * (ellipse[1][0] / 2) * (ellipse[1][1] / 2)

            # using approxPolyDP
            epsilon = 0.01 * cv2.arcLength(contour, True)  
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # area = cv2.contourArea(polygonContour) / best_fit_ellipse_area
            area = cv2.contourArea(approx) / best_fit_ellipse_area

            if (area >= 0.4):
                cv2.putText(image, str(area), (x, y), font, font_scale, font_color, thickness)
            
                # cv2.drawContours(image, [polygonContour], -1, (100,255,255), 2)
            
                cv2.ellipse(image, ellipse, (255, 0, 255), 2)

                # Draw the polygon
                cv2.polylines(image, [approx], True, (0, 30, 0), 2)
                llpython = [0,x,y,w,h,9,8,7]
                largestContour = contour
                break

    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return largestContour, img_threshold, llpython