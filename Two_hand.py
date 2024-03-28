import cv2
import mediapipe as mp
import math
import pyfirmata2
import numpy as np

board = pyfirmata2.Arduino('COM5')
servo_pin1 = board.get_pin('d:8:s')
servo_pin2 = board.get_pin('d:7:s')
servo_pin3 = board.get_pin('d:6:s')
servo_pin4 = board.get_pin('d:5:s')

servo_pin1.write(90)
servo_pin2.write(90)
servo_pin3.write(90)
servo_pin4.write(90)

# Function to calculate angle between three points
def calculate_angle(a, b, c):
    radians = math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(a[1] - b[1], a[0] - b[0])
    angle = math.degrees(radians)
    if angle < 0:
        angle += 360
    return angle

# Function to update servo positions synchronously
def update_servos(angle1, angle2, angle3, angle4):
    if angle1 <= 180:
        servo_pin1.write(angle1)
    if angle2 <= 180:
        servo_pin2.write(angle2)
    if angle3 <= 180:
        servo_pin3.write(angle3)
    if angle4 <= 180:
        servo_pin4.write(angle4)

# Function for smooth interpolation
def interpolate_smoothly(start_angle, end_angle, steps):
    angles = np.linspace(start_angle, end_angle, steps)
    smoothed_angles = np.sin(np.linspace(0, np.pi, steps)) * (end_angle - start_angle) / 2 + (start_angle + end_angle) / 2
    return smoothed_angles

# Initialize MediaPipe Pose model
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Initialize video capture
cap = cv2.VideoCapture('https://192.168.0.7:8080/video')

# Set window dimensions
WINDOW_WIDTH = 1500
WINDOW_HEIGHT = 700

# Initialize current servo positions
current_servo1_position = 90  # Example initial position
current_servo2_position = 90  # Example initial position
current_servo3_position = 90  # Example initial position
current_servo4_position = 90  # Example initial position

while cap.isOpened():
    # Read a frame from the webcam feed
    ret, frame = cap.read()
    if not ret:
        break

    # Flip the frame horizontally
    frame = cv2.flip(frame, 1)

    # Resize the frame
    frame = cv2.resize(frame, (WINDOW_WIDTH, WINDOW_HEIGHT))

    # Convert the image to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process image with MediaPipe Pose model
    results = pose.process(frame_rgb)

    # Check if landmarks are detected
    if results.pose_landmarks:
        # Extract landmark coordinates
        landmarks = results.pose_landmarks.landmark
        left_shoulder = (int(landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x * WINDOW_WIDTH),
                         int(landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y * WINDOW_HEIGHT))
        left_hip = (int(landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x * WINDOW_WIDTH),
                    int(landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y * WINDOW_HEIGHT))
        left_elbow = (int(landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x * WINDOW_WIDTH),
                      int(landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y * WINDOW_HEIGHT))
        left_wrist = (int(landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x * WINDOW_WIDTH),
                      int(landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y * WINDOW_HEIGHT))

        right_shoulder = (int(landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x * WINDOW_WIDTH),
                          int(landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y * WINDOW_HEIGHT))
        right_hip = (int(landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x * WINDOW_WIDTH),
                     int(landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y * WINDOW_HEIGHT))
        right_elbow = (int(landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x * WINDOW_WIDTH),
                       int(landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y * WINDOW_HEIGHT))
        right_wrist = (int(landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x * WINDOW_WIDTH),
                       int(landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y * WINDOW_HEIGHT))

        # Calculate servo angles
        servo1 = calculate_angle(left_hip, left_shoulder, left_elbow) - 356
        servo1 = (servo1 * -1)
        servo2 = calculate_angle(left_shoulder, left_elbow, left_wrist)
        servo3 = calculate_angle(right_hip, right_shoulder, right_elbow)
        #servo3 = (servo3 * -1)
        servo4 = ((calculate_angle(right_shoulder, right_elbow, right_wrist)-360)*-1)

        # Interpolate smoothly between current and target angles
        steps = 15  # Adjust the number of steps for smoother or more abrupt movement
        servo1_smoothed = interpolate_smoothly(current_servo1_position, servo1, steps)
        servo2_smoothed = interpolate_smoothly(current_servo2_position, servo2, steps)
        servo3_smoothed = interpolate_smoothly(current_servo3_position, servo3, steps)
        servo4_smoothed = interpolate_smoothly(current_servo4_position, servo4, steps)

        # Update servo positions synchronously
        update_servos(servo1_smoothed[-1], servo2_smoothed[-1], servo3_smoothed[-1], servo4_smoothed[-1])

        # Update current servo positions
        current_servo1_position = servo1_smoothed[-1]
        current_servo2_position = servo2_smoothed[-1]
        current_servo3_position = servo3_smoothed[-1]
        current_servo4_position = servo4_smoothed[-1]

        # Draw lines between landmarks for left arm
        cv2.line(frame, left_shoulder, left_hip, (0, 255, 0), 3)  # Left shoulder to left hip
        cv2.line(frame, left_shoulder, left_elbow, (0, 255, 0), 3)  # Left shoulder to left elbow
        cv2.line(frame, left_elbow, left_wrist, (0, 255, 0),3)  # Left elbow to left wrist

        # Draw circles on landmarks for left arm
        cv2.circle(frame, left_shoulder, 5, (0, 0, 255), -1)  # Left shoulder
        cv2.circle(frame, left_hip, 5, (0, 0, 255), -1)  # Left hip
        cv2.circle(frame, left_elbow, 5, (0, 0, 255), -1)  # Left elbow
        cv2.circle(frame, left_wrist, 5, (0, 0, 255), -1)  # Left wrist

        # Draw lines between landmarks for right arm
        cv2.line(frame, right_shoulder, right_hip, (0, 255, 0), 3)  # Right shoulder to right hip
        cv2.line(frame, right_shoulder, right_elbow, (0, 255, 0), 3)  # Right shoulder to right elbow
        cv2.line(frame, right_elbow, right_wrist, (0, 255, 0), 3)  # Right elbow to right wrist

        # Draw circles on landmarks for right arm
        cv2.circle(frame, right_shoulder, 5, (0, 0, 255), -1)  # Right shoulder
        cv2.circle(frame, right_hip, 5, (0, 0, 255), -1)  # Right hip
        cv2.circle(frame, right_elbow, 5, (0, 0, 255), -1)  # Right elbow
        cv2.circle(frame, right_wrist, 5, (0, 0, 255), -1)  # Right wrist

        # Display angles
        cv2.putText(frame, f"Left Arm Angle: {servo1:.2f} degrees", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Left Forearm Angle: {servo2:.2f} degrees", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Right Arm Angle: {servo3:.2f} degrees", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Right Forearm Angle: {servo4:.2f} degrees", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display frame
    cv2.imshow('Pose Detection', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture and close all windows
cap.release()
cv2.destroyAllWindows()

