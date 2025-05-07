from fastapi import FastAPI, Request
from openai import OpenAI
import os
from dotenv import load_dotenv

load_dotenv()  # This loads .env from the current directory
openai_api_key = os.getenv("OPENAI_API_KEY")


client = OpenAI(api_key="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")  # ← Replace with your key

app = FastAPI()

def detect_emotion_from_text(text: str) -> str:
    prompt = f"""You're an emotion classifier. Label the emotion in this message. Return only ONE word from this list: happy, sad, angry, frustrated, excited, calm, anxious, confused, neutral.
                Text: "{text}"
                Emotion:"""

    try:
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=1,
            temperature=0,
        )

        return response.choices[0].message.content.strip().lower()
    
    except Exception as e:
        print("OpenAI Error:", e)
        return "unknown"

# it checks not jsut after every prompt but all the time ? -> maybe we can force when the new promt in text ( from msg.get("content", "") ) changes ? 
# or even on a time delay - so check every 10 seconds and if it changes then change emotion response but seems less real ? 

@app.post("/vapi-webhook")
async def vapi_webhook(request: Request):
    payload = await request.json()
    print("Payload received:", payload)  # this was for testing purposes but it is useful to see everything we can access from the response

    msg = payload.get("message", {})
    msg_type = msg.get("type")

    if msg_type == "transcript":
        text = msg.get("content", "")
        print(f"Transcript: {text}")
        emotion = detect_emotion_from_text(text)
        print(f"Detected Emotion: {emotion}")

    elif msg_type in ["conversation-update", "end-of-call-report"]:
        conversation = msg.get("conversation", [])
        for entry in reversed(conversation):
            if entry.get("role") == "user":
                text = entry.get("content", "")
                print(f"Last User Message: {text}")
                emotion = detect_emotion_from_text(text)
                print(f"Detected Emotion: {emotion}")
                break

    return {"status": "ok"}