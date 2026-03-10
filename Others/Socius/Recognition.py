import face_recognition
import cv2
import os
import numpy as np

# === Load Known Faces ===
known_faces_dir = 'known_faces'
known_encodings = []
known_names = []

for person_name in os.listdir(known_faces_dir):
    person_path = os.path.join(known_faces_dir, person_name)
    if not os.path.isdir(person_path):
        continue

    for filename in os.listdir(person_path):
        if filename.lower().endswith(('.jpg', '.png')):
            filepath = os.path.join(person_path, filename)
            image = face_recognition.load_image_file(filepath)
            encodings = face_recognition.face_encodings(image)
            if encodings:
                known_encodings.append(encodings[0])
                known_names.append(person_name)
            else:
                print(f"[WARN] Could not encode face from: {filename}")

# === Initialize Webcam ===
video = cv2.VideoCapture(0)

print("[INFO] Starting webcam. Press ESC to exit.")

while True:
    ret, frame = video.read()
    if not ret:
        print("[ERROR] Failed to grab frame.")
        break

    # Resize for faster processing
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb_small = small_frame[:, :, ::-1]  # BGR to RGB

    # Detect faces and attempt encoding
    face_locations = face_recognition.face_locations(rgb_small)
    face_encodings = []

    for face_location in face_locations:
        encoding = face_recognition.face_encodings(rgb_small, [face_location])
        if encoding:
            face_encodings.append((encoding[0], face_location))
        else:
            print("[WARN] Skipped a face due to encoding failure.")

    for encoding, (top, right, bottom, left) in face_encodings:
        name = "Unknown"
        distances = face_recognition.face_distance(known_encodings, encoding)
        if len(distances) > 0:
            best_match_index = np.argmin(distances)
            if distances[best_match_index] < 0.5:
                name = known_names[best_match_index]

        # Scale up face locations to match original frame
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4

        # Draw bounding box and label
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(frame, name, (left, top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Face Recognition", frame)

    # Press ESC to exit
    if cv2.waitKey(1) & 0xFF == 27:
        break

video.release()
cv2.destroyAllWindows()
