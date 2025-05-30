from fastapi import FastAPI, Request
from openai import OpenAI
import os
import json

openai_api_key = os.getenv("OPENAI_API_KEY")

client = OpenAI(api_key="")  # ← Replace with your key

app = FastAPI()

# Store the last processed user message to prevent redundant emotion detection
last_processed_user_text = ""

def detect_emotion_from_text(text: str) -> str:
    prompt = f"""You are a therapist’s assistant trained to detect the emotional tone of a patient's message.
                Your task is to classify the message into one of exactly three emotional valence categories.

                You must return only one of the following words, exactly as written:

                    positive

                    negative

                    neutral

                Definitions:

                    positive: Optimistic, happy, calm, grateful, content, or emotionally warm.

                    negative: Sad, angry, anxious, confused, frustrated, or emotionally distressed.

                    neutral: Emotionally flat, vague, factual, non-committal, or ambiguous.
                    This includes short or unclear responses like “okay”, “sure”, “I don’t know”, “maybe”, or similar.

                Important rules:

                    Return only one word: positive, negative, or neutral.

                    Do not include any punctuation, explanation, justification, or variation in formatting.

                    If unsure, return neutral.

                Message:
                "{text}"

                Emotion:"""

    try:
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[{"role": "user", "content": prompt}],
            temperature=0,
        )

        return response.choices[0].message.content.strip().lower()
    
    except Exception as e:
        print("OpenAI Error:", e)
        return "unknown"

@app.post("/vapi-webhook")
async def vapi_webhook(request: Request):
    global last_processed_user_text

    payload = await request.json()
    #print("Payload received:", json.dumps(payload, indent=2))  # nicely formatted log

    msg = payload.get("message", {})
    msg_type = msg.get("type")

    if msg_type == "transcript":
        text = msg.get("content", "")
        print(f"[TRANSCRIPT] User said: {text}")

        emotion = detect_emotion_from_text(text)
        print(f"[EMOTION DETECTED] {emotion}")

    elif msg_type == "speech-update":
        status = msg.get("status", "")
        role = msg.get("role", "")

        if role == "assistant":
            if status == "started":
                print("[MIRO SPEAKING] Assistant has started speaking.")
            elif status == "stopped":
                print("[MIRO DONE] Assistant has finished speaking.")

    elif msg_type in ["conversation-update", "end-of-call-report"]:
        conversation = msg.get("conversation", [])
        for entry in reversed(conversation):
            if entry.get("role") == "user":
                text = entry.get("content", "")
                if text != last_processed_user_text:
                    print(f"[CONVERSATION-UPDATE] Last user message: {text}")
                    emotion = detect_emotion_from_text(text)
                    print(f"[EMOTION DETECTED] {emotion}")
                    last_processed_user_text = text
                else:
                    print("[SKIPPED] Duplicate message — already processed.")
                break

    return {"status": "ok"}