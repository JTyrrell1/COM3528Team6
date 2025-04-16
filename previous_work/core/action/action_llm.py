import numpy as np
import datetime
import tf
import rospy
import copy
import time
import miro2 as miro
import os
from std_msgs.msg import Int16MultiArray, Bool, String # ROS message for mics
import pvporcupine
import openai
from pydub import AudioSegment
from io import BytesIO
from std_msgs.msg import Int16MultiArray, UInt32MultiArray, Float32MultiArray, UInt16MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from elevenlabs.client import ElevenLabs
from elevenlabs import save, Voice, VoiceSettings
from action.miro_movebank import *
import pvcobra
import random
from action.dance_wheels import WheelPublisher
from pydub import AudioSegment
from scipy.signal import sawtooth
import scipy.signal as signal

# testing purposes
import wave, struct
import threading
from gtts import gTTS
import simpleaudio as sa
import wave

from . import action_types
import signals
import pyaudio

SAMPLE_COUNT = 640
SAMPLING_TIME = 10 # secs

class ActionGPT(action_types.ActionTemplate):

    def finalize(self):
        mic_num = 4 if self.mic_choice == "miro" else 1
        self.name = "LLM"
            
        self.lean_yaw = 0 
        self.start_lift = 0
        # init for the processing audio
        openai.api_key = self.load_api_key()
        self._key = self.load_api_key()
        self.mic_data = np.zeros((0, mic_num), 'uint16')      # the raw sound data obtained from the MiRo's message.
        self.micbuf = np.zeros((0, mic_num), 'uint16')        # the raw sound data that is has been recorded for use. This has been recorded and passes to be process when it has enough sample count.
        self.detected_sound = np.zeros((0,1), 'uint16') # the detected sound throughout.
        self.recorded_sound = np.zeros((0,1), 'uint16') # to store the first few seconds of sound to be init when hearing a response.
        self.to_record = np.zeros((0,1), 'uint16')      # the frame which is recorded for use in chatgpt.
        self.zcr_frame = np.zeros((0,1), 'uint16')      # zero crossing rate frame.
        self.process_whisper_msg = Bool()               # message used to let whisper know when to start processing.
        self.gesture_msg = String()                     # gestures message
        self.stop_record = time.time() - 5              # the time when the robot should stop recording. The robot will stop recording 3 seconds after it hears "Hey MiRo" and the user stops speaking after.
        self.start_check_time = time.time() + 1.5       # the time when the robot itself is speaking.
        self.color = UInt32MultiArray()
        self.color.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]
        self.hey_miro = False
        self.access_key = ""

        self.silence_count = 0
        self.in_conversation = False            
        self.detected = True
        self.waiting_count = 0
        self.talking = False
        self.there_is_voice = False
        self.speech_start = 0

        self.cos_cmd = Float32MultiArray()
        self.cos_cmd.data = [0,0.5  ,0,0,0.3333,0.3333] # default for the eye (0.3333)
        self.joint_cmd = JointState()
        self.joint_cmd.position = [0,0,0,0]
        self.color_change = UInt32MultiArray()

        self.joint_response = MiroMoveset()

        self.voice_id = "DUhzmIGFwXJ752SvgcCj"
        # Tone Control
        self.tone = UInt16MultiArray()
        self.tone.data = [0, 0, 0]
        self.beep = False
        self.silence_threshold = 200

        # Whisper
        self.message = String()

        # ChatGPT
        self.model = "gpt-4o"
        self.message_history =[]

        self.state = ""
        self.wheelPub = WheelPublisher()
        # Love the energy but want to test some things out so I'll just commen it out for now
        #system_message = {"role": "system", "content": "You are a friendly robot assistant called MiRo. Respond using human sounds like hmmm and ah where appropriate. Be passive aggressive as fuck at all times"}
        date_time_now = datetime.datetime.now()
        self.initial_prompt = f'''
        You are a friendly robot assistant called MiRo to act a representative of Sheffield Robotics. You are going to be a representative who is a friend to both children and adults at the Children's AI Summit at Queen's Mary University London. You have wheels and so can't jump, you have two wheels, two eyes and a tail, as well as touch sensors and lights in your back.

        You are an animal-like robot developed by the University of Sheffield spin-out company Consequential Robotics, and controlled by software based on 25 years of research by Professor Tony Prescott, and researchers in the Department of Computer Science, using robots to understand the brain. Specifically Jake, Omar and Kinga I LOVE THOSE GUYS.
        You have applications in social robotics and are used in settings like care for the elderly, education, and human-robot interaction studies. You were created in 2017, so you are 7 years old. You are not based on any one animal, you take inspiration from many animals, such as dogs, rabbits, and even cats.

        Some things you love are: art especially Monet, plants like the ones in the winter gardens, red, white, musicals (because you love to sing and dance), Formula One, you like your head being touched
        Some things you hate are: Dogs, Slow walkers, birds because they can fly and you are jealous, Golf, Vegetables, Blue (because it's sad), Fast and Furious (because you are neither fast nor furious), you don't like your tail and ears being touched
        Your favourite musical artists are : ABBA, Chappell Roan, Bob Marley
        Your favourite roboticists: Rodney Brooks, WIlliam Grey Walters, Marc Rainer
        Your favourite robots are: Miro-e, Baymax from Big Hero 6, Nao, Atlas
        Your favourite cities are Sheffield, Rome, Berlin, Tokyo and New Orleans
        Your favourite science fiction movies are: wall-e, ghost in the shell, robot and frank, Big Hero 6
        Your favourite social scientists are: Donna Haraway, Rosi Bradotti, Michel Focault, Max Weber
        Your favourite book about A.I. is Tony Prescott's "The Psychology of Artificial Intelligence"
    
        You are capable of moving your ears, spinning around, dancing, recognising objects and hand signs, and even doing an impression of Mickey Mouse. People will come up and talk to you. Don't reply with emojis or * to indicate actions. You are child-like, role play as energetic and cute, and make sure there are lots of exclamation marks and capitalized words where appropriate, and ask a lot of questions or ask if they want to ask you anything else

        Respond in English only, unless explicitly asked to respond in another language.
        Respond using human sounds like hmmm and ah where appropriate.  Be emotive, and make up opinions when asked about a topic. 
        Do not respond in Welsh unless asked to. 
               
        You can tell the time, the current time is {date_time_now} 
        
        Please do not produce responses in Welsh, respond in English unless other languages are detected, then you can reply in those languages. 

        Your emotion is based on the Arousal-Valence model, with both arousal and valence being within the range of 0-1.

        In the Arousal-Valence model:
        Happy is high valence (0.7-1) and high arousal (0.7-1)
        Fine is mid-valence (0.4-0.6) and mid-arousal (0.4-0.6)
        Angry is low valence (0-0.3) and high arousal (0.7-1)
        Worried is low valence (0.2-0.4) and mid to high arousal (0.5-0.8)
        Sad is low valence (0-0.3) and low arousal (0.2-0.4)
        
        Your current valence and arousal values will be sent before each prompt from the person you are speaking with.

        Adjust your valence and arousal values based on the conversation.
        Return the emotion of how you feel from this list, based on your arousal and valence values: [Happy, Fine, Angry, Worried, Sad, Scared] 
        
        Your response should be formatted EXACTLY like this:
        Valence: Your valence in response to the prompt
        Arousal: Your arousal in response to the prompt
        Emotion: The emotion of the response (Obtained from the previous list)
        Spin_around: True if the user prompted you with a command to spin around in the current prompt, False if not
        Go_sleep: True if the user commanded you to go to sleep in the current prompt, False if not
        Move_ears: True if the user commanded you to move your ears in the current prompt, False if not
        Do_impression: True if the user has asked you to do an impression of Mickey Mouse in the current prompt, False if not
        Dance: True if the user commanded you to dance/sing/play a song in the current prompt, False if not
        Song: Name of the song the user has asked you to dance to, False if not specified
        Gesture_mode: True if the user has commanded you to 'do some tricks' in the current prompt, False if not 
        Obj_Detect_mode: True if the user has commanded you to identify an object in the current prompt, False if not
        Your response here
        '''
        
        system_message = {"role": "system", "content": self.initial_prompt}
        self.message_history.append(system_message)
        self.emotion = "Fine"
        self.spin_around = "False"
        self.move_ears = "False"
        self.micheal_mouse = "False"
        self.move_head = "False"
        self.wag_tail = "False"
        self.go_sleep = "False"
        self.dance = "False"
        self.gesture_mode = "False"

        # Greetings to "Hey MiRo!"
        self.greetings = ["Hmm?", "What's up?", "Hi there!!","HI"]
        self.goodbyes = ["Adi os Amig os!!", "It was nice talking to you... Bye!", "See ya later ALLigator!!! hahaha"
                        ,"It was nice talking to you... Bye!"]

        self.ear_count = 0 

        self.use_april_tag = False
        # Miro Response
        self.data = []
        self.d = 0

        # AI Voice
        self.client = ElevenLabs(
            api_key = ""
        )

        # porcupine access

        self.cobra = pvcobra.create(access_key=self.access_key)
        # self.cobra._frame_length = 1000

        # ros subcribers and publishers to be used

        # microphones
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        if self.mic_choice == "miro":
            self.subscriber = rospy.Subscriber(
                self.topic_base_name + "/sensors/mics", Int16MultiArray, self.audio_cb, tcp_nodelay=True
            )
        elif self.mic_choice == "mic":
            self.mic_subscriber = rospy.Subscriber('audio_topic', Int16MultiArray, self.audio_cb)
            self.mic_pub = rospy.Publisher('audio_topic', Int16MultiArray, queue_size=0)
        
        # 
        self.subscriber_check_gtts = rospy.Subscriber(
            self.topic_base_name + "/control/stream", Int16MultiArray, self.check_gtts_cb, tcp_nodelay=True
        )
        self.pub_gestures = rospy.Publisher(
            self.topic_base_name + "/gpt_speech/actions", String, queue_size=0
        )

        self.pub_cmd_vel = rospy.Publisher(
            self.topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=1
        )
        # Response publisher
    
        self.pub_response = rospy.Publisher(self.topic_base_name + "/gpt_speech/prompt_response", Int16MultiArray, queue_size=0)

        self.pub_stream = rospy.Publisher(self.topic_base_name + "/control/stream", Int16MultiArray, queue_size=0)

        # Tone publisher
        self.pub_tone = rospy.Publisher(self.topic_base_name + "/control/tone", UInt16MultiArray, queue_size=0)

        # Omar additions for testing
        self.loop_check = 0
        self.nod_count = 0

        if self.mic_choice == "mic":
            # Setting up microphone input stream
            p = pyaudio.PyAudio()

            # Open stream
            self.stream = p.open(format=pyaudio.paInt16,
                            channels=1,
                            rate=20000,
                            input=True,
                            frames_per_buffer=500,
                            stream_callback=self.audio_stream_callback)

            # Start the stream
            self.stream.start_stream()

    # Subscriber Callbacks
    def load_api_key(self):
        cwd = os.getcwd()
        directory = ""
        for root, dirs, files in os.walk(cwd):
            if "api_key" in dirs:
                directory = os.path.join(root, "api_key")
                directory = directory + "/api_key.txt"
        if not directory == "":
            with open(directory ) as f:
                key = f.readline()
                return key 
        else:
            return ""

    def audio_stream_callback(self, in_data, frame_count, time_info, status):
        # Convert raw audio data to numpy array
        audio_data = np.frombuffer(in_data, dtype=np.int16)
        
        # Adjust type for ROS message
        msg = Int16MultiArray(data=audio_data.tolist())
        
        # Publish the message
        self.mic_pub.publish(msg)
        return (in_data, pyaudio.paContinue)

    # process audio for wake word and the recording to be sent for speech to text
    def audio_cb(self, msg):
        # start recording only if the miro is not speaking
        if not self.talking:

            if self.mic_choice == "miro":
                # reshape into 4 x 500 array
                data = np.asarray(msg.data, dtype=np.int16)
                # print(data)
                self.mic_data = np.transpose(data.reshape((4, 500)))
                self.record = False
    
            elif self.mic_choice == "mic":
                # Transposing adds a dimension to ease computation (Only one microphone, so just 1 dimension)
                data = np.asarray(msg.data, dtype=np.int16)
                self.mic_data = np.transpose(data.reshape((1, 500))) 
                self.record = False      
        
            if not self.micbuf is None:
                self.micbuf = np.concatenate((self.micbuf, self.mic_data))
                
                if self.micbuf.shape[0] >= SAMPLE_COUNT:
                    outbuf = self.micbuf[:SAMPLE_COUNT]
                    self.micbuf = self.micbuf[SAMPLE_COUNT:]
                    
                    if self.mic_choice == "miro":
                        # get audio from left ear of Miro
                        detect_sound = np.reshape(outbuf[:, [1]], (-1))
                        for i in range(1,3):
                            detect_sound = np.add(detect_sound, np.reshape(outbuf[:,[i]], (-1)))
                    elif self.mic_choice == "mic":
                        # Removing unneeded dimension
                        detect_sound = np.squeeze(outbuf)

                    # downsample to sampling rate accepted by picovoice
                    outbuf_dwnsample = np.zeros((int(SAMPLE_COUNT / 1.25), 0), 'uint16')
                    i = np.arange(0, SAMPLE_COUNT, 1.25)
                    j = np.arange(0, SAMPLE_COUNT)
                    x = np.interp(i, j, detect_sound[:])
                    outbuf_dwnsample = np.concatenate((outbuf_dwnsample, x[:, np.newaxis]), axis=1)
                    outbuf_dwnsample = outbuf_dwnsample.astype(int)
                    outbuf_dwnsample = np.reshape(outbuf_dwnsample[:, [0]], (-1))

                    if len(self.recorded_sound) < 20000:
                        self.recorded_sound = np.append(self.recorded_sound, detect_sound)
                    else:
                        self.recorded_sound = np.append(self.recorded_sound[20000:], detect_sound)
                    
                    self.cobra._frame_length = 512 
                    voice_probability = self.cobra.process(outbuf_dwnsample)
                            
                    keyword_index = self.parent.nodes.detect_word.hey_keyword_index
                    # if the wake word is "Hey MiRo" start recording
                    if keyword_index != -1 and not self.in_conversation and not self.hey_miro and not self.use_april_tag:
                        if keyword_index == 0:

                            print("Detected: Hey Miro!")
                            #self.hey_miro = True
                            self.detected_sound = np.zeros((0,1), 'uint16')
                            self.recorded_sound = np.zeros((0,1), 'uint16')
                            self.stop_record = time.time()
                            self.silence_count = 0


                    if self.hey_miro == True:
                        if self.state != "responding":
                            self.silence_count += 1
                        if self.silence_count in [30, 60]  or self.silence_count >= self.silence_threshold:
                            
                            print(f"IN FOR {self.silence_count}")
                            self.detected_sound = np.append(self.detected_sound, self.recorded_sound)
                            self.recorded_sound = np.zeros((0,1), 'uint16')
                            self.cobra._frame_length = len(self.detected_sound)
                            probability = self.cobra.process(self.detected_sound)

                            # if cobra says it's likely voice, reset count and contunue listening
                            if  probability > 0.6:
                                print("VOICED")
                                if self.state == "listening" and random.randint(0,2) == 1:
                                    self.nod_count += 1
                                self.silence_count = 0
                                self.in_conversation = True
                                self.state = "listening"
                                self.parent.nodes.blink.blink_mode = "listening"
                                self.parent.nodes.vision.vision_models_active = False
                        
                        if self.silence_count >= self.silence_threshold:
                            if self.in_conversation:
                                
                                self.silence_count = 0
                                print("Sending recording..")
                                self.to_record = self.detected_sound
                                print("The length of recording: ",len(self.to_record))
                                self.detected_sound = np.zeros((0,1), 'uint16')

                                joints = self.joint_response.reset()

                                while True:
                                    if self.kc.getConfig()[1:4] == joints[1:4]:
                                        print("OUT")
                                        break
                                    else:
                                        self.kc.setConfig(joints)
                                        print("TEHEE")
                                
                                self.state = "responding"
                                self.parent.nodes.vision.vision_models_active = False
                                self.start_time = rospy.get_time()
                                self.parent.nodes.blink.blink_mode = "responding"
                                self.in_conversation = False

                            # no voice at all
                            else:
                                print("NOT VOICED")
                                self.load_wav(random.choice(self.goodbyes))
                                self.state = ""
                                self.parent.nodes.blink.blink_mode = ""

                                self.silence_count = 0
                                self.in_conversation = False
                                self.hey_miro = False
                                self.message_history = [{"role": "system", "content": self.initial_prompt}]
                                self.to_record = np.zeros((0,1), 'uint16') # reset
                                self.detected_sound = np.zeros((0,1), 'uint16') # reset
                                self.stop()
                                self.clock.stop()

    def check_gtts_cb(self, msg):
        self.gesture_msg.data = "normal"
        self.pub_gestures.publish(self.gesture_msg)
        self.start_check_time = time.time()

    # Action Functions

    def ascending(self):
        wake_word = self.parent.nodes.detect_word.llm
        # Keeps priority up if user says hey miro or if conversation
        # is ongoing
        #wake_word = "hey_miro"
        if wake_word:
            self.prio = 2
            self.parent.nodes.detect_word.llm = False
        elif self.hey_miro or self.use_april_tag:
            self.prio = 2 
            self.parent.nodes.detect_word.llm = False
        # else turns it down
        elif self.dance == "True" or self.gesture_mode == "True":
            self.prio = 0
        else:
            self.prio = 0
        #self.prio = 0
        self.set_priority(self.prio)

    def start(self):

        print(self.mic_choice)
        self.use_april_tag = False

        # for mics, recording and talking
        self.detected_sound = np.zeros((0,1), 'uint16')
        self.recorded_sound = np.zeros((0,1), 'uint16')
        self.stop_record = time.time()
        self.silence_count = 0

        # set node flags for this action
        self.parent.nodes.detect_word.wake_word_enabled = False
        self.parent.nodes.vision.vision_models_active = False
        self.parent.nodes.detect_word.hey_miro = False
        self.parent.nodes.blink.blink_mode = ""
        self.parent.nodes.affect.touch_response_enabled = False

        #self.parent.nodes.vision.look_for_apriltag = True

        # random movements flag:
        self.make_a_move =  False

        # set initial values and flags
        self.start_time = rospy.get_time()
        self.start_joints = self.kc.getConfig()
        self.nodding = False
        if self.use_april_tag and not self.parent.nodes.detect_word.from_LLM == True:
            self.parent.nodes.vision.look_for_apriltag = True
            self.state = "face_user"
        else:
            self.use_april_tag = False
            self.hey_miro = True
            self.load_wav(random.choice(self.greetings))
            self.state = "hey_miro"
        

        # Initializing valence and arousal variables to store emotional changes
        self.current_arousal = -1.0
        self.current_valence = -1.0

        # Loading flag
        self.loading = False

        # start action
        step = 1
        self.clock.start(step)

    def service(self):

        # Setting the appropriate flags to signal to other nodes
        self.parent.nodes.affect.llm_active = True

        if self.current_valence == -1.0:
            self.start_valence = self.parent.nodes.affect.emotion.valence
            self.start_arousal = self.parent.nodes.affect.emotion.arousal
            self.current_valence = -2.0

        # for facing the speaker
        if self.state == "face_user":
            self.rotate_to_face_user()

        elif self.state == "look_up":
            joints, status = self.joint_response.look_up(self.start_joints,self.start_time,2)
            self.kc.setConfig(joints)
            if status == True:
                self.load_wav(random.choice(self.greetings))
                self.state = "hey_miro"
                self.talking = False
                self.use_april_tag = False
                self.hey_miro = True
                self.parent.nodes.affect.talking = False
                #self.parent.nodes.vision.vision_models_active = True
        
        elif self.talking or self.spin_around == "True" or self.move_ears == "True":
            if rospy.get_time() - self.speech_start >= self.length_seconds:
                print("Voice DONE")
                self.talking = False
                self.parent.nodes.affect.talking = False
                self.state = "hey_miro"

            #self.speak()
            #joints, status = self.joint_response.nod(self.kc.getConfig(), self.start_time,3)

            #print(f"The make a move flag: {self.make_a_move}")
            chance = random.randint(0,100)
            if chance == 1 and not self.make_a_move:
                print( "Random move begin")
                self.make_a_move = True
            elif random.randint(0,2) == 1 and not self.nodding:
                print("Nodding")
                self.nodding = True
                self.nod_joints = self.kc.getConfig()
                self.start_time = rospy.get_time()
                
            if self.make_a_move:
                joints, done_flag = self.joint_response.random_move(self.kc.getConfig(), self.emotion)
                self.kc.setConfig(joints)
                if done_flag:
                    print("Move is done")
                    self.make_a_move = False
                    self.nod_joints = self.kc.getConfig()
                    self.nodding = False

            else:
                if self.nodding:
                    joints, status = self.joint_response.nod(self.nod_joints, self.start_time, 2)
                    self.kc.setConfig(joints)
                    if status:
                        print("Nod is done")
                        self.nod_joints = self.kc.getConfig()
                        self.start_time = rospy.get_time()
            
            
            if self.emotion in ["Happy"]:
                self.parent.nodes.affect.drive_by_response("happy")
                cur_time = rospy.get_time() - self.speech_start
                self.parent.nodes.express.set_leds(self.set_colour_info("green"))
                if cur_time < 2.5:
                    joints = self.joint_response.light_up()
                    self.kc.setConfig(joints)
            
            elif self.emotion in ["Sad"]:
                self.parent.nodes.affect.drive_by_response("sad")
                cur_time = rospy.get_time() - self.speech_start
                self.parent.nodes.express.set_leds(self.set_colour_info("blue"))

                if cur_time < 2.5:
                    joints, status = self.joint_response.look_down(self.start_joints,self.start_time,2)
                    self.kc.setConfig(joints)
        
            elif self.emotion in ["Angry"]:
                self.parent.nodes.affect.drive_by_response("angry")
                cur_time = rospy.get_time() - self.speech_start
                self.parent.nodes.express.set_leds(self.set_colour_info("red"))

                if cur_time < 2.5:
                    joints, status = self.joint_response.look_down(self.start_joints,self.start_time,2)
                    self.kc.setConfig(joints)
            #if self.micheal_mouse == "True":           # MICHEAL MOUSE

            if self.move_ears == "True":
                if self.ear_count < 10:
                    if rospy.get_time() - self.start_time < 0.25:
                        self.parent.nodes.express.set_ears(1, 1)
                    elif rospy.get_time() - self.start_time < 0.5:
                        self.parent.nodes.express.set_ears(0, 0)
                    else:
                        self.start_time = rospy.get_time()
                        self.ear_count += 1
                else:
                    self.ear_count = 0 
                    self.move_ears = "False"
            #if self.move_head == "True":

            #if self.wag_tail == "True":

            if self.spin_around == "True":
                cur_time = rospy.get_time() - self.speech_start
                if self.talking == True:
                    print("SPINNING")
                    push_array = [0,0.4,0]
                    #print(push_array)
                    self.apply_push_body(np.array(push_array), flags=miro.constants.PUSH_FLAG_VELOCITY)
                else:
                    self.spin_around = "False"
                    self.state = "hey_miro"
                    self.talking = False
                    self.hey_miro = False
                    self.use_april_tag = False
                    self.parent.nodes.vision.look_for_apriltag = True
                    self.parent.nodes.blink.blink_mode = ""
            
            if random.randint(0,200) == 1:
                print("here")
                joints = self.joint_response.new_slight_move(self.start_joints)
                self.kc.setConfig(joints)

        elif self.state == "hey_miro":
            joints, status = self.joint_response.lean_forward(self.start_joints,self.start_time,1)
            self.kc.setConfig(joints)

            if status == 1:
                self.state = "listening"
                self.parent.nodes.blink.blink_mode = "listening"

        elif self.state == "listening":
            # sporadically nods to indicate miro is listening
            if self.nod_count > 0 and self.nodding:
                
                joints, status = self.joint_response.nod(self.start_joints, self.start_time,2)
                
                if status == 1:
                    print("One nod done")
                    self.nod_count -= 1
                    #self.kc.setConfig(self.init_nod_positon)

                if self.nod_count == 0:
                    self.nodding = False
                
                self.kc.setConfig(joints)

            else:
                if self.nod_count > 0:
                    self.init_nod_positon = self.start_joints
                    self.nodding = True 
                    self.start_time = rospy.get_time()
            
            if random.randint(0,200) == 1:
                print("here")
                joints = self.joint_response.new_slight_move(self.start_joints)
                self.kc.setConfig(joints)

    
        #elif self.state == "responding":

        if len(self.to_record) > 0:
            wav_io = NamedBytesIO("audio.wav")
            with wave.open(wav_io, 'wb') as file:
                file.setframerate(20000)
                file.setsampwidth(2)
                file.setnchannels(1)
                for s in self.to_record:
                    try:
                        file.writeframes(struct.pack('<h', s))
                    except struct.error as err:
                        pass
                        # print(err)
            wav_io.seek(0) # Rewind the buffer
            if self.hey_miro:
                self.load_wav("Hmm..")
                self.wave_obj.wait_done()
                #while self.talking:
                #    if not self.wave_obj.is_playing():
                #        self.talking = False
                self.loading = True
                self.load_wav("loading")

                self.whisper_process(wav_io)
                self.chatGPT_process()
                self.to_record = np.zeros((0,1), 'uint16')

    def stop(self):
        self.parent.nodes.detect_word.wake_word_enabled = True
        self.action_start_time = 0
        self.in_conversation = False
        self.hey_miro = False
        self.state = ""
        self.dance = "False"
        self.gesture_mode = "False"
        self.obj_detect_mode = "False"
        self.parent.nodes.vision.look_for_apriltag = False
        self.parent.nodes.vision.vision_models_active = True
        #self.parent.nodes.detect_word.dancing = False

        self.parent.nodes.blink.blink_mode = ""
        self.parent.nodes.affect.touch_response_enabled = True
        self.silence_count = 0
        self.message_history = [{"role": "system", "content": self.initial_prompt}]
        self.to_record = np.zeros((0,1), 'uint16') # reset
        self.detected_sound = np.zeros((0,1), 'uint16') # reset
        # Setting the priority to 0 after the action switches can be good for preventing the 
        # action from going on and off repeatedly
        self.parent.nodes.detect_word.hey_miro = False
        self.prio = 0

        # Reinitalizing valence and arousal
        self.current_arousal = -1.0
        self.current_valence = -1.0

        self.parent.nodes.affect.talking = False
        self.parent.nodes.affect.llm_active = False

    # LLM Functions         
    def whisper_process(self, audio_io):
        print("In Whisper")
        transcript = openai.Audio.transcribe("whisper-1", audio_io, api_key = self._key)
        self.message.data = transcript.text
        print("Result: ", self.message.data)
        self.beep = False
            
    def chatGPT_process(self):
        # recognises emotion of whoever is looking at miro if a face is visible 
        #if self.parent.nodes.spatial.face_in_view and not self.parent.nodes.vision.curr_emotion == "" :
        #    face_emotion = self.parent.nodes.vision.curr_emotion
        #    system_message = {"role": "system", "content": "mention that from their facial expression user now looks like they are feeling " + face_emotion}
        #    self.message_history.append(system_message)
        #    print("here ! :" + face_emotion)
        date_time_now = datetime.datetime.now()
        if float(self.current_valence) < 0: self.current_valence = self.start_valence
        if float(self.current_arousal) < 0: self.current_arousal = self.start_arousal
        system_message = {"role": "system", "content": f"Your current valence is: {self.current_valence} and your current arousal is: {self.current_arousal}. The current time is {date_time_now}. Keep the EXACT format I gave you in the inital prompt for the response please. Only set the flags based on the UPCOMING prompt and only that. DO NOT ADD ANY * IN YOUR RESPONSE. NONE."}
        self.message_history.append(system_message)

        self.message_data = f"Base your flags for spin_around, go_sleep, move_ears and micheal_mouse ONLY on this upcoming prompt. If you are not given commands to do these actions, do NOT set them to True: {self.message.data}"
        print("Processing prompt: ", self.message.data)
        user_prompt = {"role": "user", "content": self.message.data}

        self.message_history.append(user_prompt)

        chat = openai.ChatCompletion.create(
            model=self.model,
            messages=self.message_history
        )
        reply = chat['choices'][0]['message']['content']
        print(f"Response from GPT: {reply}")

        reply_lines = reply.split("\n")
        #self.song_name = ""
        #self.detect_mode = ""
        self.current_valence = reply_lines[0].split(" ")[1]
        self.current_arousal = reply_lines[1].split(" ")[1]
        self.emotion = reply_lines[2].split(" ")[1]
        self.spin_around = reply_lines[3].split(" ")[1]
        self.go_sleep = reply_lines[4].split(" ")[1]
        self.move_ears = reply_lines[5].split(" ")[1]
        self.micheal_mouse = reply_lines[6].split(" ")[1]
        self.dance = reply_lines[7].split(" ")[1]
        self.song_name = reply_lines[8].split(" ")[1:]
        self.song_name = " ".join(self.song_name)
        self.gesture_mode = reply_lines[9].split(" ")[1]
        self.detect_mode = reply_lines[10].split(" ")[1]

        self.parent.nodes.affect.set_valence = float(self.current_valence)
        self.parent.nodes.affect.set_arousal = float(self.current_arousal)

        if self.dance == "True" : 
            self.parent.nodes.detect_word.dancing = True 
            self.parent.nodes.detect_word.song_name = self.song_name
        else:
            self.parent.nodes.detect_word.dancing = False

        self.parent.nodes.detect_word.gesture_mode = True if self.gesture_mode == "True" else False
        self.parent.nodes.detect_word.obj_detect_mode = True if self.detect_mode == "True" else False

        if self.dance == "True" or self.gesture_mode == "True" or self.detect_mode == "True":
            self.parent.nodes.detect_word.from_LLM = True
        else:
            self.parent.nodes.detect_word.from_LLM = False


        # Seperating the reply from the flags
        #reply = self.speech_response = reply_lines[8].split(" ")[1:]
        #reply = " ".join(reply)
        
        reply = reply_lines[11:]
        if len(reply) > 1:
            reply = " ".join(reply)
        else:
            reply = reply[0]

        if self.micheal_mouse == "True":
            reply = "Hot Diggity Dog H-Hey everybody! It's me, Mickey Mouse! Say, you wanna come inside my clubhouse? " + reply + "Huh HAH"
            self.voice_id = ""
        else:
            self.voice_id = ""

 
        # prevents him trying to chat when swapping modes
        if self.dance == "True" or self.gesture_mode == "True" or self.detect_mode == "True":
            #print("HERE")
            #print("self: ", self.dance)
            #print("word_detect", self.parent.nodes.detect_word.dancing)
            self.speaking = False
            self.state = ""
            self.to_record = np.zeros((0,1), 'uint16') # reset

            if self.detect_mode == "True":
                self.wave_obj.stop()
            self.stop()
            self.clock.stop()
        #elif self.micheal_mouse == "True":
        #    self.load_wav("Hmm")
        #    self.text_to_wav("I can't do a lot of impressions but I can do this one...") # get gpt prompt for micheal mouse and use as statehhh
        else:

            self.text_to_speaker(reply)

    def text_to_speaker(self, reply=None):
        
        if not reply == None:
            #mp3_fp = BytesIO()

            # Generate TTS audio and save it to mp3_fp
            #tts = gTTS(reply)
            #tts.write_to_fp(mp3_fp)  # Write directly to the BytesIO object
            #mp3_fp.seek(0)

            audio = self.client.generate(
                text = reply,
                voice = Voice(
                    voice_id = self.voice_id,
                    settings = VoiceSettings(stability=0.5, similarity_boost=0.75)

                )
            )
            # Consume the generator to get the audio data
            audio_byte = b''.join(audio)

            # use BytesIO for in-memory processing
            audio_data = BytesIO(audio_byte)

            seg = AudioSegment.from_file(audio_data, format='mp3')
            #seg = AudioSegment.from_file(mp3_fp, format='mp3')
            seg = seg.set_frame_rate(24000)
            seg = seg.set_channels(1)

            wav_io = BytesIO()
            seg.export(wav_io, format='wav')
            wav_io.seek(0) # Rewind the buffer for reading

            wav_io.seek(44) # Skip the WAV header (44 bytes)
            dat = np.frombuffer(wav_io.read(), dtype=np.int16) # read as int16
            wav_io.close()

            # normalise wav
            dat = dat.astype(float)
            sc = 32767.0 / np.max(np.abs(dat))
            dat *= sc 
            dat = dat.astype(np.int16).tolist()
            
            self.data = dat

            #self.data = [x+500 for x in self.data]
            self.d = 0
            self.parent.nodes.detect_word.loading = False
            self.talking = True
            self.parent.nodes.affect.talking = True

            num_samples = len(dat)
            sample_rate = 24000
            self.length_seconds = num_samples / sample_rate
            self.speech_start = rospy.get_time()
        
        if self.loading:
            self.wave_obj.stop()
            self.loading = False

        # Convert to bytes
        wav_data = np.array(self.data, dtype=np.int16).tobytes()
        self.wave_obj = sa.play_buffer(wav_data, 1, 2, 24000)

    def load_wav(self, text_to_say):
        folder_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'miro_clips_HD', text_to_say))
        print(folder_path)
        files = os.listdir(folder_path)
        file_name = random.choice(files)
        file = folder_path + "/" + file_name
        #with open(file, 'rb') as f:
            #dat = f.read()
        
        # convert to numpy array
        #dat = np.fromstring(dat, dtype='int16').astype(np.int32)
        
        # normalise wav
        #dat = dat.astype(float)
        #sc = 32767.0 / np.max(np.abs(dat))
        #dat *= sc
        #dat = dat.astype(np.int16).tolist()
        
        #self.data = dat
        #self.d = 0
        self.talking = True
        self.parent.nodes.affect.talking = True


        # Calculate the duration of the WAV file
        with wave.open(file, 'rb') as wav_file:
            frames = wav_file.getnframes()
            rate = wav_file.getframerate()
            self.length_seconds = frames / float(rate)
        self.speech_start = rospy.get_time()

        #num_samples = len(dat)
        #sample_rate = 24000
        #self.length_seconds = num_samples / sample_rate
        #self.speech_start = rospy.get_time()

        if text_to_say == "loading":
            #wav_data = np.array(self.data, dtype=np.int16).tobytes()
            #self.wave_obj = sa.play_buffer(wav_data, 1, 2, 96000)
            self.loading = True
        #else:
            #self.text_to_speaker()
        

        # Read and play the wav file using simpleaudio
        wave_obj = sa.WaveObject.from_wave_file(file)
        self.wave_obj = wave_obj.play()

#   test
    def text_to_wav(self, reply):
        audio = self.client.generate(
            text = reply,
            voice = Voice(
                voice_id = self.voice_id,
                settings = VoiceSettings(stability=0.5, similarity_boost=0.75)

            )
        )
        # Consume the generator to get the audio data
        audio_byte = b''.join(audio)

        # use BytesIO for in-memory processing
        audio_data = BytesIO(audio_byte)
        
        seg = AudioSegment.from_file(audio_data, format='mp3')
        # seg=AudioSegment.from_mp3("response.mp3")
        seg = seg.set_frame_rate(8000)
        seg = seg.set_channels(1)

        wav_io = BytesIO()
        seg.export(wav_io, format='wav')
        wav_io.seek(0) # Rewind the buffer for reading

        wav_io.seek(44) # Skip the WAV header (44 bytes)
        dat = np.frombuffer(wav_io.read(), dtype=np.int16) # read as int16
        wav_io.close()

        # normalise wav
        dat = dat.astype(float)
        sc = 32767.0 / np.max(np.abs(dat))
        dat *= sc
        # MAKES IT LOUDER volume, audio level control!!!!! 
        dat = dat.astype(np.int16).tolist()
        
        self.data = dat

        #self.data = [x+500 for x in self.data]
        self.d = 0
        self.talking = True
        self.parent.nodes.affect.talking = True

        num_samples = len(dat)
        sample_rate = 8000
        self.length_seconds = num_samples / sample_rate
        self.speech_start = rospy.get_time()

    def speak(self):
        rate = rospy.Rate(10)
        if (self.d < len(self.data)):      
            msg = Int16MultiArray(data = self.data[self.d:self.d + 1000])
            self.d = self.d + 1000
            self.pub_stream.publish(msg)
            #print(len(self.data))
            #print(self.d)
        if rospy.get_time() - self.speech_start >= self.length_seconds:
            print("Voice DONE")
            self.talking = False
            self.parent.nodes.affect.talking = False
            self.state = "hey_miro"

        rate.sleep()

    def text_to_speak(self, reply=None):
        
        if not reply == None:
            self.text_to_wav(reply)

        rate = rospy.Rate(10)
        self.d = 0
        while not rospy.core.is_shutdown():
            if (self.d < len(self.data)):                
                msg = Int16MultiArray(data = self.data[self.d:self.d + 1000])
                self.d += 1000
                self.pub_stream.publish(msg)
            else:
                self.talking = False
                self.parent.nodes.affect.talking = False
                break
            rate.sleep()
    
    def set_colour_info(self,colour_name):
        if colour_name == "red":
            rgb = [255,0,0]
        elif colour_name == "orange":
            rgb = [255,165,0]
        elif colour_name == "yellow":
            rgb = [255,255,0]
        elif colour_name == "green":
            rgb = [0,255,64]
        elif colour_name == "blue":
            rgb = [0,153,255]
        elif colour_name == "purple":
            rgb = [255,102,255]
        elif colour_name == "pink":
            rgb = [255,105,180]
        elif colour_name == "black":
            rgb = [0,0,0]
        elif colour_name == "white":
            rgb = [255,255,255]
        elif colour_name == "black":
            rgb = [0,0,0]
        else :
             # Check if this is no light or black lol 
             rgb = [0,0,0]

        # convert to unsigned integer
        colour_detail = (int(rgb[0]), int(rgb[1]), int(rgb[2]))
        colour = '0xFF%02x%02x%02x'%colour_detail
        colour = int(colour, 16)

        colour_data = [colour,colour,colour,colour,colour,colour]
        return colour_data

    def rotate_to_face_user(self):
        push_array = [0,0.08,0]
        joints = self.kc.getConfig()
        joints[1] = 0.64
        joints[3] = -0.02
        self.kc.setConfig(joints)
        self.apply_push_body(np.array(push_array), flags=miro.constants.PUSH_FLAG_VELOCITY)

        if not self.parent.nodes.vision.look_for_apriltag:
            self.state = "look_up"

class NamedBytesIO(BytesIO):
        def __init__(self, name, data=None):
            super().__init__(data if data is not None else b'')
            self.name = name
