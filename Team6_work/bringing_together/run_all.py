import threading
import tkinter as tk
from Team6_work.bringing_together.shared_state import AppState
from Team6_work.start_screen import ReminiscenceTherapyGUI
from Team6_work.miro_emotions import miro_emotion_loop

def start_gui(app_state):
    root = tk.Tk()
    gui = ReminiscenceTherapyGUI(root, patient_info={"name": "Alex Smith", "age": "73"}, shared_state=app_state)
    root.mainloop()

def start_miro_emotions(app_state):
    miro_emotion_loop(app_state)

if __name__ == "__main__":
    state = AppState()

    # Start MiRo loop in a background thread
    miro_thread = threading.Thread(target=start_miro_emotions, args=(state,), daemon=True)
    miro_thread.start()

    # Launch GUI (blocking, main thread)
    start_gui(state)
