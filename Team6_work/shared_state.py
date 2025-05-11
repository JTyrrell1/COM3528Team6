class AppState:
    """
    A shared state class accessible across components.
    """
    def __init__(self):
        self.current_mode = "idle"   # idle, happy, sad, speaking, listening
        self.last_user_message = ""
        self.assistant_speaking = False
        self.therapy_started = False
