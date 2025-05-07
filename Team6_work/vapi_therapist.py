#from vapi_python import Vapi

from Team6_work.client_sdk_python_main.vapi_python.vapi_python import Vapi      # you should use this one if there is buffering problems which i am having on uni laptop - i went in and changed CHUNK_SIZE in daily_call.py in the downloaded vapi code and that sorted it iut but made convo slightly slower but not really too bad at all 

class Vapi_TheRapist:
    def __init__(self, image_description, patient_history):
        api_key = "6aa03be6-329f-44e6-b777-33df6a7605fa"

        # Initialise Vapi client
        self.assistant = Vapi(api_key=api_key)
        self.image_description = image_description
        self.patient_history = patient_history
    
    def create_and_start_assistant(self):
        assistant_overrides = {
            "recordingEnabled": False,
            "variableValues": {
                "image_description": self.image_description,
                "patient_history_pdf": self.patient_history
            }
        }

        self.assistant.start(
            assistant_id='ba05d6d9-8f92-4065-b88b-ecef1ea39d69',
            assistant_overrides=assistant_overrides
        )
