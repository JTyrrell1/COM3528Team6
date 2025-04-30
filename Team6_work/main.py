from fastapi import FastAPI, Request
from openai import OpenAI
import os

client = OpenAI(api_key="")  # ← Replace with your key

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