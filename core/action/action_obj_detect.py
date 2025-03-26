import numpy as np
import tf
import rospy
import copy
import time
import os
import miro2 as miro
from pydub import AudioSegment
import openai
from io import BytesIO
from std_msgs.msg import Int16MultiArray, UInt32MultiArray, Float32MultiArray, UInt16MultiArray, String
from elevenlabs.client import ElevenLabs
from elevenlabs import save, Voice, VoiceSettings
from action.miro_movebank import *
from gtts import gTTS
import simpleaudio as sa
import wave

from . import action_types

class ActionObjectDetect(action_types.ActionTemplate):

    def finalize(self):
        
        self.name = "Object_Game"      # need to define a name for the action
        self.prio = 0.0             # helps track priority when computing
        self.pos_objects = ["apple","banana","orange","cell phone","fork","knife","spoon","car","carrot",""]
        self.object = ""
        self.object_seen = False

        # flags
        self.talking = False
        self.joint_response = MiroMoveset()
        self.gesture_msg = String()                     # gestures message


        # used for timing movements
        self.start_time = 0
        self.start_joints = [0,0,0,0]

        # AI Voice
        self.client = ElevenLabs(
            api_key = ""
        )

        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        self.pub_stream = rospy.Publisher(self.topic_base_name + "/control/stream", Int16MultiArray, queue_size=1)

        # ChatGPT
        openai.api_key = self.load_api_key()

        self.model = "gpt-3.5-turbo"
        self.message_history =[]

        self.initial_prompt = '''
        You are a friendly robot assistant called MiRo. You are child-like, role play as energetic and cute, and make sure there are lots of exclamation marks and capitalized words where appropriate.
        In one sentence and in a child friendy manner, do not use emojis in your response and say something in response to being shown a '''

        system_message = {"role": "system", "content": self.initial_prompt}
        self.message_history.append(system_message)

        self.pub_response = rospy.Publisher(self.topic_base_name + "/gpt_speech/prompt_response", Int16MultiArray, queue_size=0)

        self.subscriber_check_gtts = rospy.Subscriber(
            self.topic_base_name + "/control/stream", Int16MultiArray, self.check_gtts_cb, tcp_nodelay=True
        )

        self.pub_gestures = rospy.Publisher(
            self.topic_base_name + "/gpt_speech/actions", String, queue_size=0
        )

    def check_gtts_cb(self, msg):
            self.gesture_msg.data = "normal"
            self.pub_gestures.publish(self.gesture_msg)
            self.start_check_time = time.time()

    def ascending(self):
        # get command from gpt 
        wake_word = self.parent.nodes.detect_word.word_action
        #wake_word = "obj_detect"
        if wake_word == "obj_detect":
            self.prio = 1.5

        self.set_priority(self.prio)


    def start(self):
        self.parent.nodes.detect_word.wake_word_enabled = False
        self.parent.nodes.vision.object_detect_active = True
        self.timer = 0
        self.object_seen = False
        self.talking = False
        self.parent.nodes.blink.blink_mode = "obj_detection"
        self.parent.nodes.vision.vision_models_active = False
        self.speech_start = 0
        self.length_seconds = 0
        self.loading = False
    
        response = "Hmmm let me see if I can figure out what it is"
        self.text_to_speaker(response)

        self.wave_obj.wait_done()
        self.talking = False

        #while self.talking:
        #    if rospy.get_time() - self.speech_start >= self.length_seconds:
        #        self.talking = False
        
        self.loading = True
        self.load_wav("loading")

        step = 15
        joints = [0,0.4,0,0.1]
        self.kc.setConfig(joints)
        self.clock.start(step)

    def service(self):
        if self.talking:
            self.clock.reset()
            self.parent.nodes.affect.touch_response_enabled = False
            self.parent.nodes.vision.object_detect_active = False

            if not self.wave_obj.is_playing():
                self.talking = False
                self.wave_obj.stop()
        else:
            if not self.loading:
                self.loading = True
                self.load_wav("loading")
                self.parent.nodes.affect.touch_response_enabled = True
                self.parent.nodes.vision.object_detect_active = True

            print(self.clock.steps_so_far)
            self.parent.nodes.vision.object_detect_active = True
            curr_object = self.parent.nodes.vision.objects
            if curr_object != []:
                curr_object = curr_object[0]
            else:
                curr_object = ""
            
            # updates info and says something only when the colour changes
            if self.object != curr_object and curr_object != "":
                #print("self:", self.object , "+ cur:" curr_object)
                self.object = curr_object
                self.chatGPT_process(curr_object)
                self.talking = True
                self.object_seen = True

        self.clock.advance(True)

        if self.clock.steps_so_far >= self.clock.steps_total:
            self.stop()
    
    def stop(self):
        self.clock.stop()
        self.prio = 0
        self.parent.nodes.vision.object_detect_active = False
        self.parent.nodes.vision.vision_models_active = True
        self.parent.nodes.detect_word.obj_detect_mode = False
        self.parent.nodes.affect.touch_response_enabled = True
        
        if self.object_seen == True:
            self.text_to_speaker("That was fun! Do you want to keep talking?")
            self.wave_obj.wait_done()
            self.talking = False

            #while self.talking:
            #    if rospy.get_time() - self.speech_start >= self.length_seconds:
            #        self.talking = False
        else:
            self.text_to_speaker("I'm sorry I couldn't see anything. Remember I can't recognise as much as you humans because I'm still learning, do you want to keep talking instead?")
            self.wave_obj.wait_done()
            self.talking = False

            #while self.talking:
            #    if rospy.get_time() - self.speech_start >= self.length_seconds:
            #        self.talking = False

        self.object_seen = False
        self.parent.nodes.detect_word.wake_word_enabled = True
        self.parent.nodes.detect_word.hey_miro = True
        self.set_priority(self.prio)

    def chatGPT_process(self,curr_object):
        # recognises emotion of whoever is looking at miro if a face is visible 
        #if self.parent.nodes.spatial.face_in_view and not self.parent.nodes.vision.curr_emotion == "" :
        #    face_emotion = self.parent.nodes.vision.curr_emotion
        #    system_message = {"role": "system", "content": "mention that from their facial expression user now looks like they are feeling " + face_emotion}
        #    self.message_history.append(system_message)
        #    print("here ! :" + face_emotion)

        system_message = [{"role": "system", "content": self.initial_prompt + curr_object}]
        #self.message_history.append(system_message)

        chat = openai.ChatCompletion.create(
            model=self.model,
            messages=system_message
        )
        reply = chat['choices'][0]['message']['content']
        print(f"Response from GPT: {reply}")
        
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
                    voice_id = "0m2tDjDewtOfXrhxqgrJ",
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
        self.talking = True


    #def load_wav(self, text_to_say):
#        folder_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'miro_clips_HD', text_to_say))
#        print(folder_path)
#        files = os.listdir(folder_path)
#        file_name = random.choice(files)
#        file = folder_path + "/" + file_name
#        with open(file, 'rb') as f:
#            dat = f.read()
        
        # convert to numpy array
#        dat = np.fromstring(dat, dtype='int16').astype(np.int32)
        
        # normalise wav
#        dat = dat.astype(float)
#        sc = 32767.0 / np.max(np.abs(dat))
#        dat *= sc
#        dat = dat.astype(np.int16).tolist()
        
#        self.data = dat
#        self.d = 0
#        self.talking = True
#        self.parent.nodes.affect.talking = True

#        num_samples = len(dat)
#        sample_rate = 24000
#        self.length_seconds = num_samples / sample_rate
#        self.speech_start = rospy.get_time()

#        if text_to_say == "loading":
#            wav_data = np.array(self.data, dtype=np.int16).tobytes()
#            self.wave_obj = sa.play_buffer(wav_data, 1, 2, 96000)
#            self.loading = True
#        else:
#            self.text_to_speaker()

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
        #self.talking = True
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

    # Functions to ouput audio from MiRo speakers
    def text_to_wav(self, reply):
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
        # MAKES IT LOUDER volume, audio level control!!!!! 
        dat *= 1.25
        dat = dat.astype(np.int16).tolist()
        
        self.data = dat

        #self.data = [x+500 for x in self.data]
        self.d = 0
        self.talking = True

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
            print(len(self.data))
            print(self.d)
        else:
            self.talking = False
        rate.sleep()

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

class NamedBytesIO(BytesIO):
    def __init__(self, name, data=None):
        super().__init__(data if data is not None else b'')
        self.name = name
