import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Set up the GPIO pins for the servos
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

servo_x = GPIO.PWM(17, 50)  # X servo on pin 17 (50Hz PWM)
servo_y = GPIO.PWM(27, 50)  # Y servo on pin 27 (50Hz PWM)

servo_x.start(7.5)  # Initial position
servo_y.start(7.5)  # Initial position

# Function to convert servo angle to duty cycle
def angle_to_duty_cycle(angle):
    return (angle / 180.0) * 10 + 2.5

# Function to detect box using OpenCV
def detect_box(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply a Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detect edges using Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours from the detected edges
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the contour with the largest area (assuming this is the box)
    if contours:
        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        return max_contour

    return None

# Capture video from the camera
cap = cv2.VideoCapture(0)

try:
    while True:
        # Read a frame from the video feed
        ret, frame = cap.read()

        if not ret:
            break

        # Detect the box
        box_contour = detect_box(frame)

        if box_contour is not None:
            # Calculate the center of the box
            M = cv2.moments(box_contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Draw the detected box and its center
            cv2.drawContours(frame, [box_contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)

            # Calculate servo angles to aim away from the box
            x_ratio = cX / frame.shape[1]
            y_ratio = cY / frame.shape[0]

            x_angle = 180 - 180 * x_ratio
            y_angle = 180 * y_ratio

            # Update servo positions
            servo_x.ChangeDutyCycle(angle_to_duty_cycle(x_angle))
            servo_y.ChangeDutyCycle(angle_to_duty_cycle(y_angle))


        # Show the frame in a window
        cv2.imshow("Detecting Motion...", frame)

        # Update the previous frame
        prev_frame = gray

        # Wait for a key press and check if the 'q' key was pressed
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        capture.release()
        cv2.destroyAllWindows()

        stop_pwm(servo1_pwm)
        stop_pwm(servo2_pwm)

        GPIO.cleanup()
               
except KeyboardInterrupt:
    pass
finally:
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    servo_x.stop()
    servo_y.stop()
    GPIO.cleanup()
