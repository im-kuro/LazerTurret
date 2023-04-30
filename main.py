import cv2, time

# capturing video from webcam
capture = cv2.VideoCapture(0)
# set window size and name
cv2.namedWindow("Detecting Motion...", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Detecting Motion...", 1200, 900)


while capture.isOpened():
    # to read frame by frame
    _, img_1 = capture.read()
    _, img_2 = capture.read()

    # find difference between two frames
    diff = cv2.absdiff(img_1, img_2)

    # to convert the frame to grayscale
    diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)

    # apply some blur to smoothen the frame
    diff_blur = cv2.GaussianBlur(diff_gray, (5, 5), 0)

    # to get the binary image
    _, thresh_bin = cv2.threshold(diff_blur, 20, 255, cv2.THRESH_BINARY)

    # to find contours
    contours, hierarchy = cv2.findContours(thresh_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # to draw the bounding box when the motion is detected
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if cv2.contourArea(contour) > 3500:
            cv2.rectangle(img_1, (x, y), (x+w, y+h), (205, 55, 0), 2)
            cv2.putText(img_1, "Status: {}".format('Movement'), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
    # display the output
    cv2.imshow("Detecting Motion...", img_1)
    if cv2.waitKey(100) == 13:
        exit()

# release the capture and destroy all windows
capture.release()
cv2.destroyAllWindows()