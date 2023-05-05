import cv2
import RPi.GPIO as GPIO
import time
"""
# Define the GPIO pins for the servos
servo1_pin = 27
servo2_pin = 17

# Set up the GPIO pins for the servos
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)

# Define the ranges of servo angles
SERVO1_RANGE = (0, 180)
SERVO2_RANGE = (0, 180)

# Define the PWM frequency and duty cycle range
PWM_FREQUENCY = 50
PWM_DUTY_CYCLE_RANGE = (2.5, 12.5)

# Define the function to initialize the PWM signals for the servos
def init_pwm(pin, freq, duty_cycle_range):
    pwm = GPIO.PWM(pin, freq)
    pwm.start(0)
    pwm.ChangeDutyCycle(duty_cycle_range[0])
    time.sleep(0.5)
    return pwm

# Initialize the PWM signals for the servos
servo1_pwm = init_pwm(servo1_pin, PWM_FREQUENCY, PWM_DUTY_CYCLE_RANGE)
servo2_pwm = init_pwm(servo2_pin, PWM_FREQUENCY, PWM_DUTY_CYCLE_RANGE)

# Define the function to map a value from one range to another
def map_value(value, from_range, to_range):
    return (value - from_range[0]) * (to_range[1] - to_range[0]) / (from_range[1] - from_range[0]) + to_range[0]

# Define the function to gradually move a servo from its current position to a target position
def move_servo_smoothly(pwm, current_pos, target_pos, step):
    while current_pos != target_pos:
        if current_pos < target_pos:
            current_pos = min(current_pos + step, target_pos)
        else:
            current_pos = max(current_pos - step, target_pos)
        pwm.ChangeDutyCycle(map_value(current_pos, SERVO1_RANGE, PWM_DUTY_CYCLE_RANGE))
        time.sleep(0.05)
    return current_pos

# Define the function to stop the PWM signal for a given servo
def stop_pwm(pwm):
    pwm.ChangeDutyCycle(0)
    time.sleep(0.5)
    pwm.stop()

# Define the function to move a servo to a specific angle
def set_servo_position(pwm, pos, delay=0.5):
    pwm.ChangeDutyCycle(map_value(pos, SERVO1_RANGE, PWM_DUTY_CYCLE_RANGE))
    time.sleep(delay)

# Capturing video from webcam
capture = cv2.VideoCapture(0)

# Set window size and name
cv2.namedWindow("Detecting Motion...", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Detecting Motion...", 1200, 900)

while capture.isOpened():
    ret, frame = capture.read()
        # Convert the frame to grayscale and apply a Gaussian blur to reduce noise
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    # Store the previous frame
    if 'prev_frame' not in globals():
        prev_frame = gray

    # Compute the absolute difference between the current and previous frames
    frame_diff = cv2.absdiff(prev_frame, gray)

    # Apply a threshold to the frame difference
    thresh = cv2.threshold(frame_diff, 30, 255, cv2.THRESH_BINARY)[1]

    # Dilate the thresholded image to fill in holes
    thresh = cv2.dilate(thresh, None, iterations=2)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over the contours
    for contour in contours:
        # Compute the bounding box for the contour
        (x, y, w, h) = cv2.boundingRect(contour)

        # Draw a rectangle around the contour
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Move the servos based on the position of the object in the frame
        # Center of the object in the frame
        center_x = x + w // 2
        center_y = y + h // 2

        # Convert the center coordinates to servo angles
        servo1_angle = int(map_value(center_x, (0, frame.shape[1]), SERVO1_RANGE))
        servo2_angle = int(map_value(center_y, (0, frame.shape[0]), SERVO2_RANGE))

        # Move the servos smoothly to the target positions
        move_servo_smoothly(servo1_pwm, servo1_pwm.current_pos, servo1_angle, 1)
        move_servo_smoothly(servo2_pwm, servo2_pwm.current_pos, servo2_angle, 1)

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
        
  """
        

import time

# Define pins for servos
x_pin = 17
y_pin = 27

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(x_pin, GPIO.OUT)
GPIO.setup(y_pin, GPIO.OUT)

# Set up PWM frequency and initial duty cycle
frequency = 50
duty_cycle_x = 7.5
duty_cycle_y = 7.5

# Create PWM instances for servos
x_servo = GPIO.PWM(x_pin, frequency)
y_servo = GPIO.PWM(y_pin, frequency)

# Start PWM
x_servo.start(duty_cycle_x)
y_servo.start(duty_cycle_y)

# Move servos to different positions
while True:
    # Move x_servo to the left
    x_servo.ChangeDutyCycle(10)
    time.sleep(1)

    # Move y_servo up
    y_servo.ChangeDutyCycle(10)
    time.sleep(1)

    # Move x_servo to the center
    x_servo.ChangeDutyCycle(7.5)
    time.sleep(1)

    # Move y_servo down
    y_servo.ChangeDutyCycle(5)
    time.sleep(1)

# Clean up GPIO pins
GPIO.cleanup()
