import cv2
import numpy as np
import mediapipe as mp
import socket
import time
import threading

# IP address and port of the ESP32 access point
ESP32_IP = "192.168.4.1"
ESP32_PORT = 80

client_socket = None

def connect_to_esp32():
    global client_socket
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((ESP32_IP, ESP32_PORT))
            print("Connected to ESP32")
            return
        except Exception as e:
            print(f"Failed to connect to ESP32: {e}")
            time.sleep(5)  # Wait before retrying

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Initialize hand tracking
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)

def count_fingers(landmarks):
    fingertip_indices = [4, 8, 12, 16, 20]
    finger_base_indices = [2, 5, 9, 13, 17]

    finger_status = []

    thumb_tip_idx = fingertip_indices[0]
    thumb_base_idx = finger_base_indices[0]
    thumb_tip = landmarks[thumb_tip_idx]
    thumb_base = landmarks[thumb_base_idx]
    index_finger_base = landmarks[finger_base_indices[1]]

    vector_thumb_base_tip = np.array(thumb_tip) - np.array(thumb_base)
    vector_thumb_base_index = np.array(index_finger_base) - np.array(thumb_base)

    dot_product = np.dot(vector_thumb_base_tip, vector_thumb_base_index)

    mag_thumb_base_tip = np.linalg.norm(vector_thumb_base_tip)
    mag_thumb_base_index = np.linalg.norm(vector_thumb_base_index)

    angle_rad = np.arccos(dot_product / (mag_thumb_base_tip * mag_thumb_base_index))
    angle_deg = np.degrees(angle_rad)

    if angle_deg < 90:
        finger_status.append(0)
    else:
        finger_status.append(1)

    for idx, (tip_idx, base_idx) in enumerate(zip(fingertip_indices[1:], finger_base_indices[1:])):
        if landmarks[tip_idx][1] < landmarks[base_idx][1]:
            finger_status.append(1)
        else:
            finger_status.append(0)

    return finger_status

def send_data():
    global finger_data, client_socket
    previous_finger_data = None
    while True:
        try:
            if finger_data and finger_data != previous_finger_data:
                # Send finger status to ESP32
                client_socket.sendall(finger_data.encode())
                previous_finger_data = finger_data
        except Exception as e:
            print(f"Error sending data: {e}")
            connect_to_esp32()
        time.sleep(1)

cap = cv2.VideoCapture(0)

# Connect to ESP32 initially
connect_to_esp32()

send_thread = threading.Thread(target=send_data)
send_thread.start()

finger_data = "no_hand"

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        continue

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            landmarks = np.array([[lmk.x, lmk.y, lmk.z] for lmk in hand_landmarks.landmark])
            finger_status = count_fingers(landmarks)

            finger_data_str = ''.join(map(str, finger_status))

            if finger_data_str != finger_data:
                finger_data = finger_data_str

            for idx, status in enumerate(finger_status):
                cv2.putText(frame, f'Finger {idx+1}: {status}', (10, 30 + 30 * idx), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        # If no hand is detected, send "no_hand" message
        if finger_data != "no_hand":
            finger_data = "no_hand"
    # Display the resulting frame
    cv2.imshow('Hand Tracking', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()

# Close the connection
client_socket.close()
