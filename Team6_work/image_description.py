import base64

import openai
from openai import OpenAI

CLIENT = OpenAI(api_key="")

class ImageDescriber():
    def __init__(self, image_path):
        self.image_path = image_path
        self.base64_image = self.encode_image()

        self.response = self.create_image_description()

    def encode_image(self):
        with open(self.image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    def create_image_description(self):
        response = CLIENT.responses.create(
            model="gpt-4.1-mini",
            input=[{
                "role": "user",
                "content": [
                    {"type": "input_text",
                     "text": "You are an AI assistant preparing conversational prompts for use in a reminiscence therapy session with an early-onset dementia patient. You are given an image and must describe it in a warm, accessible, and emotionally engaging way. Focus on familiar and evocative elements — such as settings from nature, architecture, objects, clothing, people, or activities — that may help the viewer recall meaningful memories or spark gentle, reflective conversation. Your response should be written in full sentences and structured to be passed into another AI model that will use it to guide a dialogue. Highlight specific visual details that could lead to storytelling (e.g., “a worn wooden bench,” “a group of people sharing a meal,” “a child flying a kite”), but do not ask any questions — only describe. Keep the tone positive, nostalgic, and grounded. Your output should feel like a kind narrator describing a photo for someone who may not be able to see it clearly, with the goal of helping them connect it to their own life experience."},
                    {
                        "type": "input_image",
                        "image_url": f"data:image/jpeg;base64,{self.base64_image}",
                    },
                ],
            }],
        )

        return response