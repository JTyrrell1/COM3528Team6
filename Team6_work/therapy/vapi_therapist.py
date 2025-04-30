from vapi_python import Vapi

vapi = Vapi(api_key='')

assistant_overrides = {
    "recordingEnabled": False,
    "variableValues": {
        "name": "Alex",
        "likes": "apple juice, hates all other types of juice"
    }
}


vapi.start(assistant_id='ba05d6d9-8f92-4065-b88b-ecef1ea39d69', assistant_overrides=assistant_overrides)


