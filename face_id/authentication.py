# authentication.py
# source for faceID: https://www.youtube.com/watch?v=

import cv2
from face_id.simple_facerec import SimpleFacerec
import shutil
import os
import time

# Initialize the SimpleFacerec object globally
sfr = SimpleFacerec()
sfr.load_encoding_images("face_id/images/")

def authenticate(callback):
    global sfr
    cap = cv2.VideoCapture(0)
    is_recognised = 0 

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        face_locations, face_names, is_recognised = sfr.detect_known_faces(frame)
        for face_loc, name in zip(face_locations, face_names):
            y1, x2, y2, x1 = face_loc
            cv2.putText(frame, name, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 200), 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 200), 4)
                
            if name != "Unknown":
                break  # Recognised the person

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        if is_recognised == 1:
            time.sleep(5)  # Keep frame open for 2 more seconds after recognition
            break

    cap.release()
    cv2.destroyWindow("Frame")
    callback(is_recognised)

def add_face_id(image_path):
    # Copy the image to the directory
    destination = "face_id/images/" + os.path.basename(image_path)
    shutil.copy(image_path, destination)

    # Reload the images to include the new face ID
    global sfr
    sfr.load_encoding_images("face_id/images/")
if __name__ == "__main__":
    def my_callback(is_recognised):
        print("Face recognised!" if is_recognised else "Face not recognised or something went wrong.")

    authenticate(my_callback)
