import numpy as np
import tf
import rospy
import copy
import time
import os
import miro2 as miro
from pydub import AudioSegment
from io import BytesIO
from std_msgs.msg import Int16MultiArray, UInt32MultiArray, Float32MultiArray, UInt16MultiArray
from elevenlabs.client import ElevenLabs
from elevenlabs import save, Voice, VoiceSettings
from action.miro_movebank import *
import simpleaudio as sa

from gtts import gTTS

from . import action_types

class ActionGestures(action_types.ActionTemplate):

    def finalize(self):
        
        self.name = "gestures"      # need to define a name for the action
        self.prio = 0.0             # helps track priority when computing

        self.greetings = ["Are you waving at me?","Hello there", "I'm sorry I don't have arms so I can't wave back","HI HI HI"]
        self.victory_speech = ["hell yeah brother, I can do a peace sign too", "yeahhhh peace and loveee","heehee I can do that too"]
        self.iLoveYou_speech = ["ROCK ON YEAHH!!!!","woooo","hell yeahh","uhnts uhnts uhnts"]
        self.spin_intros = ["Here I go!", "Look at me go!", "WOOHOO", "Woahwoahwoah"]

        # flags
        self.talking = False
        self.in_gesture_response = False
        self.rainbow_increment = 0
        self.multiplier = 5
        self.from_llm = False
        self.timer = 0 


        self.joint_response = MiroMoveset()

        # used for timing movements
        self.start_time = 0
        self.start_joints = [0,0,0,0]

        # Cosmetics
        self.cosmetics = [0.0, 0.5, 0.0, 0.0, 0.3333, 0.3333]
        self.ear_count = 0
        self.d = 0 
        self.data = []

        # Response
        self.response = ""

        # AI Voice
        self.client = ElevenLabs(
            api_key = ""
        )

        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        self.pub_stream = rospy.Publisher(self.topic_base_name + "/control/stream", Int16MultiArray, queue_size=1)

    """
    def compute_priority(self):
        curr_gesture = self.parent.nodes.vision.gesture
        self.llm_request = self.parent.nodes.detect_word.gesture_mode
        if curr_gesture != "" or self.in_gesture_response or self.llm_request:
            print("here")
            self.prio = 1.2

        return self.prio
    """

    def ascending(self):
        curr_gesture = self.parent.nodes.vision.gesture
        self.llm_request = self.parent.nodes.detect_word.gesture_mode
        if curr_gesture != "" or self.in_gesture_response or self.llm_request:
            #print("here")
            self.prio = 1.2

        self.set_priority(self.prio)

    def start(self):
        self.parent.nodes.detect_word.wake_word_enabled = False
        self.parent.nodes.vision.vision_models_active = True
        self.parent.nodes.affect.touch_response_enabled = False
        self.timer = 0
        self.parent.nodes.blink.blink_mode = "gestures"
        self.check = False
        
        if self.llm_request:
            joints = [0,0.4,0,0.1]
            self.kc.setConfig(joints)
            self.from_llm = True
            response = "Okay! Do some hand signs and I'll show you some tricks"
            self.text_to_speaker(response)
            self.wave_obj.wait_done()
            self.talking = False

            #while self.talking:
            #    if not self.wave_obj.is_playing():
            #        self.talking = False
            step = 150
        else:
            step = 100    

        print("HERE?!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        self.clock.start(step)

    def service(self):
        # If self.talking, speak regardless of robot action
        if self.talking:
            self.parent.nodes.vision.vision_models_active = False
            
            if not self.wave_obj.is_playing():
                self.text_to_speaker()
            else:
                if rospy.get_time() - self.speech_start  >= self.length_seconds:
                    self.talking = False
        else:
            self.parent.nodes.vision.vision_models_active = True
        
        # if a hand signal has already been seen
        if self.in_gesture_response:
            status = self.response_func()            
            # status indicates whether response is complete
            if status:
                self.in_gesture_response = False
            self.clock.reset()

        # if looking for hand signal (gesture)
        elif self.parent.nodes.vision.vision_models_active:
            curr_gesture = self.parent.nodes.vision.gesture
            #print(curr_gesture)
            if curr_gesture == "Open_Palm":
                # get greeting and convert to pub_streamaudio file to be spoken
                self.response = self.greetings[random.randint(0,len(self.greetings)-1)]
                # prepares speech and current configuration for movements
                self.response_func = self.open_palm

            elif curr_gesture == "Pointing_Up":
                self.response = self.spin_intros[random.randint(0,len(self.spin_intros)-1)]
                self.response_func = self.spin
            
            elif curr_gesture == "Victory":
               # self.response = "Hell yea brother! I can do a peace sign too!"
                self.response = self.victory_speech[random.randint(0,len(self.victory_speech)-1)]
                self.response_func = self.ear_victory

            elif curr_gesture == "ILoveYou":               
                self.response_func = self.party_mode
                self.response = self.iLoveYou_speech[random.randint(0,len(self.iLoveYou_speech)-1)]

                #self.response = "Yeah rock on dude!"

            if curr_gesture in ["Open_Palm", "Pointing_Up", "Victory","ILoveYou"]:
                self.talking = True
                self.text_to_speaker(self.response)
                self.start_joints = self.kc.getConfig()
                self.start_time = rospy.get_time()
                self.parent.nodes.vision.vision_models_active = False
                self.in_gesture_response = True

            #elif curr_gesture == "Closed_Fist":            #########################
            #elif curr_gesture == "Thumb_Up":               These Need Implementing!!
            #elif curr_gesture == "Thumb_Down":             #########################
    
                # party 
                #   rainbow lights 
                #   "dancey gestures"
                #   say ("DID SOMEBODY SAY PARTY!!!!!!!!!!")
            
        self.timer = self.timer + 1

        self.clock.advance(True)
        
        if self.clock.steps_so_far >= self.clock.steps_total:
            print(self.clock.steps_so_far)
            self.stop()

    def stop(self):
        self.prio = 0
        self.parent.nodes.blink.blink_mode = ""
        self.clock.stop()

        # does a voiceline to indicate leaving the gesture mode
        if self.from_llm:
            self.from_llm = False
            self.text_to_speaker("that was fun, do you want to keep talking?")
#            while self.talking:
#                if not self.wave_obj.is_playing():
#                    self.speak()
                
            # swaps modes
            self.parent.nodes.detect_word.hey_miro = True
            self.parent.nodes.detect_word.gesture_mode = False

        self.parent.nodes.affect.touch_response_enabled = True
        self.parent.nodes.vision.vision_models_active = True
        self.parent.nodes.detect_word.wake_word_enabled = True
    
        self.set_priority(self.prio)

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
                    voice_id = "DUhzmIGFwXJ752SvgcCj",
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
        
        # Convert to bytes
        wav_data = np.array(self.data, dtype=np.int16).tobytes()
        
        self.wave_obj = sa.play_buffer(wav_data, 1, 2, 24000)

        self.now_moving = True

    def text_to_speech(self, reply):
        audio = self.client.generate(
            text = reply,
            voice = Voice(
                voice_id = "0m2tDjDewtOfXrhxqgrJ",
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
        dat = dat.astype(np.int16).tolist()
        
        self.data = dat
        self.d = 0
        self.talking = True

    def speak(self):
        rate = rospy.Rate(10)
        if (self.d < len(self.data)):          
            msg = Int16MultiArray(data = self.data[self.d:self.d + 1000])
            self.d = self.d + 1000
            self.pub_stream.publish(msg)
            print(len(self.data))
            print(self.d)
        else:
            self.talking = False
        rate.sleep()
    
    # Gesture Functions ---------------------------------------------------------
    
    def party_mode(self):
        self.rainbow_loop()

        if not self.wave_obj.is_playing():
            self.text_to_speaker()
        joints, status = self.joint_response.party_nod(self.start_joints, self.start_time,4)
        self.kc.setConfig(joints)

        return True if status else False


    def get_rainbow(self,colours):

        color = [0]*6
        #print(colours[0][1])
        count = 0
        for x in colours:
            color_detail = (int(x[0]),int(x[1]),int(x[2]))
            color[count] = '0xFF%02x%02x%02x'%color_detail
            color[count] = int(color[count], 16)
            count = count+1
        # six seperate leds in the miro
        color_change = [
            color[0],
            color[0],
            color[0],
            color[0],
            color[0],
            color[0]
        ]
        
        return color_change

    def rainbow_loop(self):
        purple = [204, 17, 224]
        blue = [0,153,255]
        green = [0,255,0]
        yellow = [255,255,0]
        orange = [255,165,0]
        red = [255,0,0]

        colours = [purple,blue,green,red,orange,yellow]
        if self.rainbow_increment < 1 * self.multiplier:
            colours = [purple]
        elif self.rainbow_increment < 2 * self.multiplier:
            colours = [blue]
        elif self.rainbow_increment < 3 * self.multiplier:
            colours = [green]
        elif self.rainbow_increment < 4 * self.multiplier:
            colours = [yellow]
        elif self.rainbow_increment < 5 * self.multiplier:
            colours = [orange]
        elif self.rainbow_increment < 6 * self.multiplier:
            colours = [red]
        else:
            self.rainbow_increment = 0 
            colours = [red]

        colour_data = self.get_rainbow(colours)
        self.rainbow_increment = self.rainbow_increment + 1
        self.parent.nodes.express.set_leds(colour_data)


    def open_palm(self):
        joints, status = self.joint_response.nod(self.start_joints, self.start_time,20)
        self.kc.setConfig(joints)

        return False if self.talking else True
    
    def spin(self):
        if rospy.get_time() - self.start_time < 1.9:
            self.in_gesture_response = True
            push_array = [0,0.5,0]
            self.apply_push_body(np.array(push_array), flags=miro.constants.PUSH_FLAG_VELOCITY)
            return False
        else:
            return True
    
    def ear_victory(self):
        if self.ear_count < 4:
            if rospy.get_time() - self.start_time < 0.5:
                joints, status = self.joint_response.lean_forward(self.start_joints, self.start_time,3)
                self.kc.setConfig(joints)
                self.parent.nodes.express.set_ears(1, 1)
            elif rospy.get_time() - self.start_time < 1:
                self.parent.nodes.express.set_ears(0, 0)
            else:
                self.start_time = rospy.get_time()
                self.ear_count += 1
        else:
            self.ear_count = 0
            return True

        return False
        

class NamedBytesIO(BytesIO):
    def __init__(self, name, data=None):
        super().__init__(data if data is not None else b'')
        self.name = name
