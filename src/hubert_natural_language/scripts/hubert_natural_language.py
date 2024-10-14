from openai import OpenAI
client = OpenAI()

PROMPT = "You are a stationary robot arm. You are given a query like 'Please prepare me some apple slices'. You are also given an image of the scene in front of you. \
    You then output a list of atomic instructions necessary to execute the given task. You can choose only from the following set of instructions: 'place arm over <object>, close end effector, open end effector, move arm by <x,y> milimeters relative to ground plane'."

completion = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {
            "role": "user",
            "content": "What is two plus two?"
        }
    ]
)

print(completion.choices[0].message.content)