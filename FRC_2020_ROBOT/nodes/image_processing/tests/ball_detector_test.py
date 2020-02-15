import cv2 as cv
import numpy as np
import imutils

global h_sum
global calc_count

h_sum = 35
calc_count = 1


def calculate_circle_area(r):
    return float(((r[3] + r[2]) / 4) ** 2) * np.pi


def detect_ball(mat, display=True):
    """
    input: RGB opencv mat
    output: x, y, radius of ball in pixels.
    """
    # Basic processing
    # frame_ = cv.resize(frame, (320, 240))
    hsv = cv.cvtColor(mat, cv.COLOR_RGB2HSV)
    frame_gray = cv.cvtColor(hsv, cv.COLOR_BGR2GRAY)

    # Detecting using opencv-haar-cascade-classifier
    ball_cascade = 'ball_cascade.xml'
    cascade_classifier = cv.CascadeClassifier()
    if not cascade_classifier.load(cv.samples.findFile(ball_cascade)):
        print('cascade not found')

    balls = cascade_classifier.detectMultiScale(frame_gray,
                                                scaleFactor=1.1,
                                                minNeighbors=5,
                                                minSize=(16, 16),
                                                flags=cv.CASCADE_SCALE_IMAGE)

    if len(balls) == 0:
        if display:
            show_images([mat])
        return None

    ball = max(balls, key=calculate_circle_area)

    x, y, w, h = ball
    cROI = hsv[y:y + h, x:x + w]
    avg_color_per_row = np.average(cROI, axis=0)
    avg_color = np.average(avg_color_per_row, axis=0)
    avg_h = avg_color[0]

    h = 85
    offset = 20
    mask = cv.inRange(hsv, (h - offset, 86, 6), (h + offset, 255, 255))
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)

    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,
                           cv.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(c)
        M = cv.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv.circle(mat, (int(x), int(y)), int(radius),
                      (0, 255, 255), 2)
            cv.circle(mat, center, 5, (0, 0, 255), -1)
    # cv.circle(frame, (ball[0], ball[1]), int((ball[3] + ball[2]) / 4), (255, 0, 0))
    if display:
        show_images([mat, mask])


def process(mat):
    lower = (15, 86, 6)
    upper = (50, 255, 255)

    hsv = cv.cvtColor(mat, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    mask = cv.erode(mask, (3, 3), iterations=3)
    mask = cv.dilate(mask, (3, 3), iterations=3)

    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts) > 0:
        c = max(cnts, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(c)

        M = cv.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        cv.circle(mat, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        cv.circle(mat, center, 5, (0, 0, 255), -1)
    show_images([mat, hsv, mask])


def show_images(images):
    for i, image in enumerate(images):
        if image is not None:
            try:
                cv.imshow(str(i), image)
            except:
                pass
    cv.waitKey(1)


if __name__ == "__main__":
    cap = cv.VideoCapture(0)

    while 1:
        ret, frame = cap.read()
        detect_ball(frame)
