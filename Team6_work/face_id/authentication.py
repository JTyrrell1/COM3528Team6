import cv2
import os
import time
from Team6_work.face_id.simple_facerec import SimpleFacerec

def authenticate(callback=None):
    sfr = SimpleFacerec()
    sfr.load_encoding_images("Team6_work/face_id/images/")

    cap = cv2.VideoCapture(0)

    attempt_counter = 0
    recognised_counter = 0
    recognised_name = None
    consistent_threshold = 5

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        attempt_counter += 1
        print(f"Attempt {attempt_counter}: ", end="")

        face_locations, face_names, is_recognised = sfr.detect_known_faces(frame)
        print(f"Found {len(face_locations)} face(s)")

        for (top, right, bottom, left), name in zip(face_locations, face_names):
            if name != "Unknown":
                print(f"Recognised: {name}")
                if name == recognised_name:
                    recognised_counter += 1
                else:
                    recognised_name = name
                    recognised_counter = 1

                if recognised_counter >= consistent_threshold:
                    print("Confirmed identity:", name)
                    cv2.putText(frame, f"{name}", (left, top - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                    cv2.imshow("Face ID Authentication", frame)
                    if callback:
                        callback(name)
                    time.sleep(5)
                    cap.release()
                    cv2.destroyAllWindows()
                    return
            else:
                print("Unrecognised")

            # Draw red box regardless
            cv2.putText(frame, f"{name}", (left, top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        cv2.imshow("Face ID Authentication", frame)

        if cv2.waitKey(1) == ord('q') or attempt_counter >= 100:
            print("Face not recognised or something went wrong.")
            break

    cap.release()
    cv2.destroyAllWindows()

def my_callback(name):
    print("Face recognised!")
    # You can call your MIRO function here
    # launch_reminiscence(name)

if __name__ == "__main__":
    authenticate(my_callback)
