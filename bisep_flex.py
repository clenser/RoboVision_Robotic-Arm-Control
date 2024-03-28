import cv2
import mediapipe as mp
import math
from pyfirmata2 import Arduino, util
import time

# Define constants
SERVO_PIN = 9  # Define the pin number for the servo motor
COM_PORT = 'COM5'  # Define the serial port

# Connect to Arduino board
board = Arduino(COM_PORT)
servo_pin = board.get_pin(f'd:{SERVO_PIN}:s')


# Function to calculate angle between three points
def calculate_angle(a, b, c):
    radians = math.atan2(c.y - b.y, c.x - b.x) - math.atan2(a.y - b.y, a.x - b.x)
    angle = math.degrees(radians)
    return angle + 360 if angle < 0 else angle


# Initialize MediaPipe and OpenCV
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 700)

# Get frame dimensions
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define constants for smoother movement
SMOOTH_FACTOR = 0.1  # Adjust as needed
previous_angle = 90  # Initial position

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Flip the frame horizontally
    frame = cv2.flip(frame, 1)
    # Perform pose detection
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frame_rgb)

    if results.pose_landmarks:
        # Extract relevant landmarks for left arm positions
        left_shoulder, left_elbow, left_wrist = [results.pose_landmarks.landmark[i] for i in
                                                 [mp_pose.PoseLandmark.LEFT_SHOULDER, mp_pose.PoseLandmark.LEFT_ELBOW,
                                                  mp_pose.PoseLandmark.LEFT_WRIST]]

        # Calculate the angle between the left shoulder, elbow, and wrist
        angle_between_arms = calculate_angle(left_shoulder, left_elbow, left_wrist)

        # Smooth movement
        smoothed_angle = (1 - SMOOTH_FACTOR) * previous_angle + SMOOTH_FACTOR * angle_between_arms

        # Set the servo position to the smoothed angle
        servo_pin.write(int(smoothed_angle))

        # Update previous angle
        previous_angle = smoothed_angle

        # Draw landmarks on the frame
        for landmark in [left_shoulder, left_elbow, left_wrist]:
            cv2.circle(frame, (int(landmark.x * frame_width), int(landmark.y * frame_height)), 5, (0, 255, 0), -1)

        # Draw lines between the landmarks
        for p1, p2 in zip([left_shoulder, left_elbow], [left_elbow, left_wrist]):
            cv2.line(frame, (int(p1.x * frame_width), int(p1.y * frame_height)),
                     (int(p2.x * frame_width), int(p2.y * frame_height)), (0, 255, 255), 2)

        # Display the angle on the frame
        cv2.putText(frame, f'Angle: {smoothed_angle:.2f} degrees', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),
                    2, cv2.LINE_AA)

    # Display the flipped frame with annotations (if needed)
    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
        break

    # Add a small delay for smoother movement
    time.sleep(0.01)

# Release the camera and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()
