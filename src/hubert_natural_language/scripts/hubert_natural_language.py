from openai import OpenAI
client = OpenAI()

PROMPT_SYSTEM = "You are a stationary robot arm tasked with executing specific actions based on a given query and an image of the scene in front of you. \
    Your responses should be a detailed list of atomic instructions necessary to complete the task. \
    You can only use the following instructions: 'place arm over <object>; close end effector; open end effector; move arm by <x,y> milimeters relative to ground plane'"

PROMPT_USER = "Please prepare me some apple slices."

PROMPT_ASSISTANT = "1. open end effector \
                    2. place arm over <knive> \
                    3. close end effector \
                    4. place arm over <apple> \
                    5. move arm by <20, 0> millimeter relative to ground plane \
                    6. move arm by <-20, 0> millimeter relative to ground plane \
                    7. move arm by <20, 0> millimeter relative to ground plane"

completion = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {
        "role": "system",
        "content": [{"type": "text", "text": PROMPT_SYSTEM}]
        },
        {
        "role": "user",
        "content": [{ "type": "text", "text": PROMPT_USER},
                    {"type": "image_url", "image_url": {"url": "https://www.shutterstock.com/shutterstock/photos/175132970/display_1500/stock-photo-kitchen-knife-and-green-apple-on-wooden-background-175132970.jpg", "detail": "low"},
                }]
        },
        {
        "role": "assistant",
        "content": [{ "type": "text", "text": PROMPT_ASSISTANT }]
        },
    ]
)

print(completion.choices[0].message.content)