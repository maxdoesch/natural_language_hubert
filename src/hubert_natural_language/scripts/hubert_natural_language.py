import rospy
import re
import time
from hubert_launch.srv import HubertPrompt, HubertPromptResponse, GoToCoordinate, GoToCoordinateResponse, MoveArm, MoveArmResponse
from hubert_launch.msg import LabeledPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import base64
import cv2
from io import BytesIO
from PIL import Image as PILImage
from openai import OpenAI
from std_srvs.srv import Empty
from std_msgs.msg import String


PROMPT_SYSTEM = "You are a stationary robot arm tasked with executing instructions based on a given query by a human operator. If you are also given an image you should analyze the image for objects in the scene relevant to the human prompt. In order to be able execute these instructions you break them down into smaller atomic instructions which are necessary to complete the task. You can only use the following set of instructions: (place arm over <object>; perform grabbing action with end effector; open end effector; move arm by <x,y> millimeters relative to the ground plane, done)."

PROMPT_USER_1 = "Human: Please prepare me some apple slices."

PROMPT_ASSISTANT_1 = "Robot: 1. open end effector 2. place arm over <knive> 3. perform grabbing action with end effector 4. place arm over <apple> 5. move arm by <20,0> millimeters relative to the ground plane 6. move arm by <-20,0> millimeters relative to the ground plane 7. move arm by <20,0> millimeters relative to the ground plane 8. done"

PROMPT_USER_2 = "Human: Bring my child something to play with."

PROMPT_ASSISTANT_2 = "Robot: 1. open end effector 2. place arm over <stuffed giraffe> 3. perform grabbing action with end effector 4. place arm over <child> 5. open end effector 6. done"

PROMPT_USER_3 = "Human: How would you hold the snickers?"

PROMPT_ASSISTANT_3 = "Robot: 1. open end effector 2. place arm over <snickers> 3. perform grabbing action with end effector 4. done"

PROMPT_USER_4 = "Human: How would you put a grapefruit from the table into the bowl?"

PROMPT_ASSISTANT_4 = "Robot: 1. open end effector 2. place arm over <grapefruit> 3. perform grabbing action with end effector 4. place arm over <bowl> 5. open end effector 6. done"

PROMPT_USER_5 = "Human: How would you bring me the peanuts?"

PROMPT_ASSISTANT_5 = "Robot: 1. open end effector 2. place arm over <peanuts> 3. perform grabbing action with end effector 4. move arm by <30,0> millimeters relative to the ground plane 5. done"

PROMPT_USER = [PROMPT_USER_1, PROMPT_USER_2, PROMPT_USER_3, PROMPT_USER_4, PROMPT_USER_5]
PROMPT_ASSISTANT = [PROMPT_ASSISTANT_1, PROMPT_ASSISTANT_2, PROMPT_ASSISTANT_3, PROMPT_ASSISTANT_4, PROMPT_ASSISTANT_5]

class NaturalLanguageHubert:
    def __init__(self):
        self.client = OpenAI()
        self.bridge = CvBridge()

        rospy.Service("hubert_prompt", HubertPrompt, self.prompt_callback)
        rospy.Subscriber("/hubert_camera/image_raw", Image, self.image_callback)
        rospy.Subscriber('/hubert_camera/pixel_coordinate', LabeledPoint, self.labeled_point_callback)
        
        self.label_pub = rospy.Publisher('/hubert/label_topic', String, queue_size=1)

        rospy.wait_for_service('/hubert/open_effector')
        rospy.wait_for_service('/hubert/place_arm')
        rospy.wait_for_service('/hubert/grab')
        rospy.wait_for_service('/hubert/move_arm')

        try:
            self.open_effector_service = rospy.ServiceProxy(
                '/robot/open_effector', 
                Empty
            )
            self.place_arm_service = rospy.ServiceProxy(
                '/robot/go_to_coordinate', 
                GoToCoordinate
            )
            self.grab_service = rospy.ServiceProxy(
                '/robot/grab', 
                Empty
            )
            self.move_arm_service = rospy.ServiceProxy(
                '/robot/move_arm', 
                MoveArm
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"Service proxy creation failed: {e}")

        self.latest_image_base64 = None
        self.labeled_point = None

    def prompt_callback(self, request):

        response = "No image information available!"

        if self.latest_image_base64 is not None:
            response = self.openai_request(request.prompt, self.latest_image_base64)

        return HubertPromptResponse(f"Received prompt: {response}")
    
    def image_callback(self, msg):
        self.latest_image_base64 = self.convert_image_to_base64(msg)

    def labeled_point_callback(self, msg):
        self.labeled_point = msg

    def convert_image_to_base64(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        buffered = BytesIO()
        pil_image.save(buffered, format="JPEG")
        
        img_str = base64.b64encode(buffered.getvalue()).decode('utf-8')
        
        return img_str
    
    def openai_request(self, prompt, base64_image):

        messages = []
        messages += [{"role": "system","content": [{"type": "text", "text": PROMPT_SYSTEM}]},]
        for user_message, assistant_message in zip(PROMPT_USER, PROMPT_ASSISTANT):
            messages += [{
                "role": "user",
                "content": [{"type": "text", "text": user_message}]
            },
            {
                "role": "assistant", 
                "content": [{"type": "text", "text": assistant_message}]
            }]
        messages += [{"role": "user",
                "content": [{"type": "text", "text": f"Human: {prompt}"},
                            #{"type": "image_url", "image_url": {"url":  f"data:image/jpeg;base64,{base64_image}"},},
                        ]
                }]

        completion = self.client.chat.completions.create(
            model="gpt-4o",
            messages = messages
        )

        self.execute_commands(completion.choices[0].message.content)

        return completion.choices[0].message.content
    
    def parse_robot_commands(self, prompt):
        commands = []
        
        clean_prompt = prompt.replace("Robot: ", "")
        command_list = re.split(r'\d+\.\s*', clean_prompt.strip())
        command_list = [cmd.strip() for cmd in command_list if cmd.strip()]

        for command in command_list:
            if "place arm over" in command:
                object_match = re.search(r'<([^>]+)>', command)
                if object_match:
                    object_name = object_match.group(1)
                    commands.append(("place_arm_over", object_name))

            elif "perform grabbing action with end effector" in command:
                commands.append(("grab", None))

            elif "open end effector" in command:
                commands.append(("open_effector", None))

            elif "move arm by" in command:
                coords_match = re.search(r'<([-\d]+),(\w+)>', command)
                if coords_match:
                    x = int(coords_match.group(1))
                    y = coords_match.group(2)
                    commands.append(("move_arm", (x, y)))

            elif "done" in command:
                commands.append(("done", None))

        return commands
    
    def place_arm_over(self, object_name):
        rospy.loginfo(f"Placing arm over {object_name}")

        self.label_pub(object_name)

        while object_name != self.labeled_point.label:
            time.sleep(1)


        self.place_arm_service(self.labeled_point.point)
        

    def grab(self):
        rospy.loginfo("Performing grabbing action")

        self.grab_service()

    def open_effector(self):
        rospy.loginfo("Opening end effector")

        self.open_effector_service()

    def move_arm(self, x, y):
        rospy.loginfo(f"Moving arm by x={x}, y={y} millimeters")

        self.move_arm_service(x, y)


    def done(self):
        rospy.loginfo("Finished execution")
    
    def execute_commands(self, prompt):
        commands = self.parse_robot_commands(prompt)
        
        for command_type, params in commands:
            if command_type == "place_arm_over":
                self.place_arm_over(params)
            elif command_type == "grab":
                self.grab()
            elif command_type == "open_effector":
                self.open_effector()
            elif command_type == "move_arm":
                self.move_arm(*params)
            elif command_type == "done":
                self.done()
                break

if __name__ == '__main__':
    rospy.init_node('natural_language_hubert_node')

    hubert_service = NaturalLanguageHubert()

    rospy.spin()