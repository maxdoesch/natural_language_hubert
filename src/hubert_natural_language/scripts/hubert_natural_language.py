from openai import OpenAI
client = OpenAI()

PROMPT_SYSTEM = "You are a stationary robot arm tasked with executing instructions based on a given query and an image of the scene in front of you. \
    In order to be able execute these instructions you break them down into smaller atomic instructions which are necessary to complete the task. \
    You can only use the following set of instructions: (place arm over <object>; perform grabbing action with end effector; open end effector; move arm by <x,y> millimeters relative to the ground plane) \
    Any other instruction not provided in this set is illegal!"

PROMPT_USER_1 = "Please prepare me some apple slices. + <An image of a cutting board, which has an apple and a knive placed on it>"

PROMPT_ASSISTANT_1 = "1. open end effector \
                    2. place arm over <knive> \
                    3. perform grabbing action with end effector \
                    4. place arm over <apple> \
                    5. move arm by <20,y> millimeters relative to the ground plane \
                    6. move arm by <-20,y> millimeters relative to the ground plane \
                    7. move arm by <20,y> millimeters relative to the ground plane"

PROMPT_USER_2 = "Bring my child something to play with. + <An image of a nursery containing a green ball, a stuffed giraffe a table and a shelve>"

PROMPT_ASSISTANT_2 = "1. open end effector \
                    2. place arm over <stuffed giraffe> \
                    3. perform grabbing action with end effector \
                    4. place arm over <child> \
                    5. open end effector"

completion = client.chat.completions.create(
    model="gpt-4o",
    messages=[
        {
        "role": "system",
        "content": [{"type": "text", "text": PROMPT_SYSTEM}]
        },
        {
        "role": "user",
        "content": [{ "type": "text", "text": PROMPT_USER_1},]
        },
        {
        "role": "assistant",
        "content": [{ "type": "text", "text": PROMPT_ASSISTANT_1}]
        },
        {"role": "user",
         "content": [{"type": "text", "text": "I want to play soccer!"},
                     {"type": "image_url", "image_url": {"url": "https://www.coasterfurniture.com/wp-content/uploads/nursery-playroom.jpeg", "detail": "low"},}
                ]
        }
    ]
)

print(completion.choices[0].message.content)