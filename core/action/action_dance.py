import numpy as np
import tf
import rospy
import copy
import miro2 as miro
import os
import std_msgs
import time
import wave,struct
import pydub
import asyncio
import base64
import json
import math
import random
import os 
from action.dance_illum import IllumPublisher
from action.dance_wheels import WheelPublisher
from action.dance_body import JointsPublisher

import requests
from requests import post, get
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray, UInt32MultiArray, Float32MultiArray, UInt16MultiArray
from geometry_msgs.msg import Vector3
from shazamio import Shazam

# miro vocalisation
from gtts import gTTS
from pydub import AudioSegment
from io import BytesIO
from elevenlabs.client import ElevenLabs
from elevenlabs import save, Voice, VoiceSettings
import openai
import soundfile as sf
from action.miro_movebank import *
import urllib.request
import simpleaudio as sa

from . import action_types

RECORD_TIME = 5

MIC_SAMPLE_RATE = 20000

SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE

class ActionDance(action_types.ActionTemplate):

    """
        ActionDance can be provoked by either; 
            1- Using a wakeword to stop MiRos current action and listen for music to dance to
        
        If music is detected the dancing will continue for a random length between 10-60 seconds
        before it begins to decay

        If music isn't detected MiRo will perform an sequence (miro_no_match) to indicate it didn't 
        pick up any music to dance along to.
    """
    # Action Functions ---------------------------------------------------------------

    # does same stuff as __init__
    def finalize(self):

        # parameters
        self.name = "dance"
        self.start_listening = False
        self.song_over = False
        self.dance_prio = 0 
        self.autoState = False
        self.start_time = 0
        self.cur_section = 0
        self.joint_moves = MiroMoveset()
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # for spotify
        self.client_id = "e435cd4262be434e8a17be2635b40ef7" #TODO
        self.client_secret = "0bc1bbc09e0240af86c261155821557f" #TODO

        cwd = os.getcwd()
        for root, dirs, files in os.walk(cwd):
            if "dancing_songs" in dirs:
                self.directory = os.path.join(root, "dancing_songs")

        # state flags
        self.state = "wait"
        self.state_start_time = 0
        self.talking = False
        self.preview_available = False

        # song info 
        self.prerecorded_filenames = ["/dance_the_night.mp3","/dancing_queen.mp3","/higher_and_higher.mp3","/in_the_moonlight.mp3","/just_dance.mp3",
                                    "/murder_dancefloor.mp3","/whitney_dance.mp3","/make_you_sweat.mp3"]
        self.prerecorded_songnames = ["dance the night dua lipa","dancing queen abba", "higher and higher jackie wilson", "dancing in the moonlight toploader","just dance lady gaga",
                                    "murder on the dancefloor sophie ellis bextor","I wanna dance with somebody whitney houston", "gonna make you sweat C&C"]

        self.duration = 0 
        self.tempo = 0
        self.sections = []
        self.genre = ""
        self.song_name = ""
        self.now_dancing = False
        # recording params
        self.micbuf = np.zeros((0,4),"uint16")
        self.outbuf = None
        self.x_len = 20000
        self.no_of_mics = 4
        self.input_mics = np.zeros((self.x_len,self.no_of_mics))
        self.start_listening = False

        self.start_dance_phrases = ["YEAH I LOVE THAT ONE!", "YIPEEEE LETS DANCE!!","SURE!!"]
        
        self.wheelPub = WheelPublisher()
        self.jointsPub = JointsPublisher()
        self.illumPub = IllumPublisher()

        # AI Voice
        self.client = ElevenLabs(
            api_key = ""
        )

        self.pub_stream = rospy.Publisher(topic_base_name + "/control/stream", Int16MultiArray, queue_size=0)

        # preprogrammed moves
        self.dance_names = ["head_bounce","head_bang","full_head_spin","head_bop"]
        self.dance_name = "general"

        # subscribers
        topic_name = topic_base_name + "/sensors/mics"
        self.sub_mics = rospy.Subscriber(topic_name,
            std_msgs.msg.Int16MultiArray, self.callback_identify_mics, tcp_nodelay=True)
    
    # used for recording mic audio when triggered by wake word
    def callback_identify_mics(self, msg):
        data = np.asarray(msg.data)
        data = np.transpose(data.reshape((self.no_of_mics,500)))

        if self.start_listening == True:
            if not self.micbuf is None:
                self.micbuf = np.concatenate((self.micbuf, data))

                # Done Recording
                if self.micbuf.shape[0] >= SAMPLE_COUNT:
                    self.outbuf = self.micbuf
                    self.micbuf = None

    # override of inherited function from action_types, to prevent from decaying before ready
    # used to calculate priority
    def ascending(self):
        # wake word + song found in environment
        dance = self.parent.nodes.detect_word.dance
        #dance = "dance"
        # if wake word detected
        if dance == True:
            print("YES")
            # stops listening in environment
            #self.parent.nodes.detect_music.listening = False
            self.dance_prio = 1.5
            if self.parent.nodes.detect_word.from_LLM:
                self.song_name = self.parent.nodes.detect_word.song_name
            else:
                self.song_name = ""
            #print(self.song_name)

        self.set_priority(self.dance_prio)
    
    # called once when the action is first selected
    def start(self):
        self.parent.nodes.detect_word.wake_word_enabled = False
        self.parent.nodes.affect.touch_response_enabled = False
        self.parent.nodes.detect_word.dancing = False
        self.parent.nodes.detect_word.dance = False
        self.parent.nodes.vision.vision_models_active = False
        #self.song_name = "Dance The Night Dua Lipa"
        print(self.song_name)
        # clock : 50~ p/second 
        song_picked = True

        self.speech_start = 0
        self.length_seconds = 0
        self.now_dancing = False
        if self.song_name == "False" or self.song_name == "Not Specified":
            song_picked = False
            self.song_name = ""
        elif self.song_name != "":
            print("we here?")
            self.set_track_data()
            dance_length = random.randint(1000,2500)
            file_name = "/song.mp3"

        if self.song_name == "" or self.preview_available == False:
            print("ewedas")
            if self.parent.nodes.detect_word.from_LLM and song_picked == True:
                self.text_to_speaker("Hmm, I don't know that one, how about this one instead")
            else:
                self.text_to_speaker("YIPPEE Lets GOO")
            self.wave_obj.wait_done()
            self.talking = False
            self.now_dancing = False

            #while self.talking:
            #    if rospy.get_time() - self.speech_start >= self.length_seconds:
            #        self.talking = False
            #        self.now_dancing = False

            dance_length = 500 + random.randint(1500,3000)

            song_to_dance = random.randint(0,7)
            file_name = self.prerecorded_filenames[song_to_dance]
            self.song_name = self.prerecorded_songnames[song_to_dance]

            print(self.song_name)
            self.set_track_data()

        autoState = True if random.randint(0,1) == 1 else False
        self.state = "dance"
        self.state_start_time = rospy.get_time()
        self.start_joints = self.kc.getConfig()
        self.song_over = False
        self.talking = True
        self.wheelPub.set_tempo(self.tempo)
        self.jointsPub.set_tempo(self.tempo)
        self.illumPub.set_tempo(self.tempo)
        self.get_audiofile(file_name)
        self.clock.start(dance_length)
        
        #self.chatGPT_process("Okay! Play me a song or I can play one of MY favourites!")
        #self.chatGPT_process("play song")

    # called repeatedly as the action continues, used to maintain the action
    # here is where the dancing is performed
    # uses a series of states to allow the dancing to be sequential but not use for or while loops
    # as other processes need to be performed between service calls
    def service(self):
        cur_time = rospy.get_time() - self.state_start_time
        if self.talking:
            if self.now_dancing == False:
                print("starting dance")
                self.play_dance_music()
            elif self.now_dancing:
                if not self.wave_obj.is_playing():
                    self.song_over = True
                    self.talking = False

          
        if self.state == "look up":
            joints, status = self.joint_moves.look_up(self.start_joints,self.state_start_time,2)
            self.kc.setConfig(joints) 
            if status:
                self.state = "record and shazam"
                self.state_start_time = rospy.get_time()

        elif self.state == "no match":
            joints, status = self.joint_moves.shake_head(self.state_start_time,3)
            self.kc.setConfig(joints)

            if self.talking == False:
                self.state = "dance"
                self.state_start_time = rospy.get_time()
                self.get_audiofile()
                self.talking = True
                
        elif self.state == "record and shazam":
            # enables the mic callback to record
            self.start_listening = True

            # wait until enough samples
            waiting = True
            while waiting:
                if not self.outbuf is None:
                    break

            # save audio as wav file
            outfilename = self.directory + "/miro_audio.wav"
            file = wave.open(outfilename , "w")
            
            file.setsampwidth(2)
            file.setframerate(MIC_SAMPLE_RATE)
            file.setnchannels(2)
            x = np.reshape(self.outbuf[:,[0,1]],(-1))
            
            for s in x:
                file.writeframes(struct.pack('<h',s))
            file.close()

            # convert wav to ogg for shazamio
            sound = pydub.AudioSegment.from_wav(outfilename)
            ogg_output = self.directory + "/miro_audio.ogg"
            sound.export(ogg_output,format="ogg")

            # find song through shazamio 
            self.start_listening = False
            async def findSong():
                shazam = Shazam()
                out = await shazam.recognize(self.directory + "/miro_audio.ogg")

                # perform sequence if no match found to indicate MiRo's confusion
                if len(out["matches"]) == 0:
                    self.song_name = "i wanna dance with somebody"
                    self.text_to_speaker("Hmm I don't know that one, I can play music instead!!")

                    #print("No song found, check the audio is recording correctly and try again")
                    self.state = "no match"
                    self.state_start_time = rospy.get_time()
                    self.start_joints = self.kc.getConfig()

                    self.talking = True

                    
                else:
                    self.song_name = out["track"]["title"]
                    print(self.song_name)
                    self.state = "dance"


                self.set_track_data()
                self.autoState = False
                self.start_time = rospy.get_time()
                self.wheelPub.set_tempo(self.tempo)
                self.jointsPub.set_tempo(self.tempo)
                self.illumPub.set_tempo(self.tempo)
                self.state_start_time = rospy.get_time()
                    
            loop = asyncio.new_event_loop()
            loop.run_until_complete(findSong())

            self.start_listening = False

        elif self.state == "dance":
            self.parent.nodes.affect.touch_response_enabled = False
            self.parent.nodes.affect.drive_by_dance(self.genre)
            #print(self.sections)
            #print(cur_time)
            for x in self.sections:
                if x < cur_time:
                    new_section = x

            if self.cur_section != new_section:
                # switches between auto and pre-programmed move 
                # for the next section
                self.autoState = not self.autoState
                if self.autoState:
                    self.dance_name = ""
                else:
                    self.change_dance()
                self.cur_section = new_section

                # update publishers with new command
                self.wheelPub.command = self.dance_name
                self.jointsPub.command = self.dance_name

            # wheel controls
            push_array = self.wheelPub.loop(cur_time)
            #self.kc.setPose([3,3,3])
            self.apply_push_body(np.array(push_array), flags=miro.constants.PUSH_FLAG_VELOCITY)
    
            # "body" controls, kinematics = (Yaw, Pitch, Lift), cosmetics = (ears, eyes, tail) 
            cosmetics, kinematics = self.jointsPub.loop(cur_time,0)
            
            # method for sending kinematic commands
            self.kc.setConfig(kinematics.position)

            # connects to express node that is controlling cosmetic output
            self.parent.nodes.express.set_ears(cosmetics.data[4],cosmetics.data[5])
            #self.parent.nodes.express.set_eyes(cosmetics.data[2],cosmetics.data[3])
            self.parent.nodes.blink.blink_mode = "dancing"
            self.parent.nodes.blink.dance_eye_openness = cosmetics.data[2]

            
            # LED controls
            illuminations = self.illumPub.loop(cur_time, 0) 
            self.parent.nodes.express.set_leds(illuminations.data)
            
            # if song ends
        if self.song_over == True or self.parent.nodes.detect_word.stop_miro == True:
            self.stop()
        #self.clock.advance(True)

    def get_audiofile(self, filename):
        outfilename = self.directory + filename

        seg = AudioSegment.from_file(outfilename, format='mp3')
        # seg=AudioSegment.from_mp3("response.mp3")
        seg = seg.set_frame_rate(24000)
        seg = seg.set_channels(1)

        wav_io = BytesIO()
        seg.export(wav_io, format='wav')
        wav_io.seek(0) # Rewind the buffer for reading

        wav_io.seek(44) # Skip the WAV header (44 bytes)
        dat = np.frombuffer(wav_io.read(), dtype=np.int16) # read as int16
        wav_io.close()
        dat = dat.astype(float)
        sc = 32767.0 / np.max(np.abs(dat))
        dat *= sc
        dat = dat.astype(np.int16).tolist()
        print(len(dat))
        self.data = dat
        self.d = 0
    
    def speak(self):
        rate = rospy.Rate(10)
        if (self.d < len(self.data)):      
            msg = Int16MultiArray(data = self.data[self.d:self.d + 1000])
            self.d = self.d + 1000
            self.pub_stream.publish(msg)
        else:
            print("Voice DONE")
            self.talking = False
        
        rate.sleep()
    # called when the action is complete
    def stop(self):
		# standard stop behaviour, reset clock
        self.clock.stop()
        self.wave_obj.stop()
        # reset dancing variables
        #self.parent.nodes.detect_music.listening = True
        print("amiright???")
        # song_name = nothing 
        self.song_name = ""
        self.start_time = 0
        self.state_start_time = 0 
        self.song_over = False


        # set the value to take back to LLM Mode
        if self.parent.nodes.detect_word.from_LLM:
            self.text_to_speaker("that was fun, do you want to keep talking?")
            self.wave_obj.wait_done()
            self.talking = False
            self.now_dancing = False

            #while self.talking:
            #    if rospy.get_time() - self.speech_start >= self.length_seconds:
            #        self.talking = False
            #        self.now_dancing = False
            self.parent.nodes.detect_word.from_LLM = False
            self.parent.nodes.detect_word.hey_miro = True
            self.dancing = False
        else:
            self.text_to_speaker("I love that song !")
            self.wave_obj.wait_done()
            self.talking = False
            self.now_dancing = False

            #while self.talking:
            #    if rospy.get_time() - self.speech_start >= self.length_seconds:
            #        self.talking = False
            #        self.now_dancing = False

        self.parent.nodes.detect_word.wake_word_enabled = True
        self.parent.nodes.affect.touch_response_enabled = True
        self.parent.nodes.vision.vision_models_active = True
        self.dance_prio = 0 
        self.set_priority(self.dance_prio)

    # Spotify and Shazam Functions --------------------------------------------------------------
    def set_track_data(self):
        # Finds Song on Spotify 
        url = "https://api.spotify.com/v1/search"
        token = self.get_token()
        headers = self.get_auth_headers(token)

        query = f"?q={self.song_name}&type=track&limit=1"
        query_url = url + query
        # query_url = "https://api.spotify.com/v1/tracks/2FRnf9qhLbvw8fu4IBXx78"
        result = get(query_url, headers=headers)
        json_result = json.loads(result.content)
        json_result = json_result["tracks"]["items"]

        # If no results were found searching song name, can't continue
        if len(json_result) == 0:
            print("No Track Returned with that name")
            return None
        
        # Song ID
        track = json_result[0]
        track_id = track["id"]
        artist_id = track["artists"][0]["id"]
        preview_url = track["preview_url"]
        self.duration = track["duration_ms"]/1000
        print(f"Duration {self.duration}")
        outfilename = self.directory + "/song.mp3"
        if preview_url is not None:
            urllib.request.urlretrieve(preview_url, outfilename)
            self.preview_available = True
        else:
            self.preview_available = False
        
        # Song Genre 
        url = f"https://api.spotify.com/v1/artists/{artist_id}"
        result = get(url, headers=headers)
        json_result = json.loads(result.content)
        # Returns multiple, need to find way to differentiate as they're all kinda random
        genre_list = json_result["genres"]
        self.decide_genre(genre_list)

        # Audio Analysis (Tempo)
        self.base_url = "https://api.cyanite.ai/graphql"
        self.session = requests.Session()

        access_token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ0eXBlIjoiSW50ZWdyYXRpb25BY2Nlc3NUb2tlbiIsInZlcnNpb24iOiIxLjAiLCJpbnRlZ3JhdGlvbklkIjoxNDAwLCJ1c2VySWQiOjEyOTU2OCwiYWNjZXNzVG9rZW5TZWNyZXQiOiIyYTBmMWI0NDcxOGY0YjQ0MTIyYTZkOTI3NWE5Njc1ZTcxYTQzNmU3YTlmY2NjNjVmMTkxYmI1MjFlMzJjZGVhIiwiaWF0IjoxNzM4Njc4NjI3fQ._k9pcZ9p16ClCy7H2_lsDxe-W2MmQpx1uyjHTAsiTrg" #TODO
        self.headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {access_token}'  # Replace with your actual access token if needed
        }
        misc_mutation = """
            mutation SpotifyTrackEnqueueMutation($input: SpotifyTrackEnqueueInput!) {
                spotifyTrackEnqueue(input: $input) {
                    __typename
                    ... on SpotifyTrackEnqueueSuccess {
                        enqueuedSpotifyTrack {
                            id
                            audioAnalysisV6 {
                                __typename
                                ... on AudioAnalysisV6Finished {
                                    result {
                                        bpmPrediction {
                                        value
                                        confidence
                                        }
                                    }
                                }
                            }
                        }
                    }
                    ... on Error {
                        message
                    }
                }
            }
            """

        variables = {
            "input": {
                "spotifyTrackId": track_id  # Replace with the actual Spotify track ID
            }
        }

        data = {
            "query": misc_mutation,
            "operationName": "SpotifyTrackEnqueueMutation",
            "variables": variables
        }

        self.tempo = self.retrieve_data(data, 'bpmPrediction')['value']
        print(f'BPM: {self.tempo}')
        # url = f"https://api.spotify.com/v1/audio-analysis/{track_id}"
        # result = get(url, headers=headers)
        # audio_analysis_dict = json.loads(result.content)
        # print(audio_analysis_dict)
        # self.duration = audio_analysis_dict["track"]["duration"]
        # self.tempo = int(audio_analysis_dict["track"]["tempo"])
        if self.tempo is None:
            self.tempo = 90
        total = 0 
        self.sections = []
        self.sections.append(total)
        for x in range(0,50):
            # faster song = swap moves more, slower song = swap moves less
            bps = 1 / (self.tempo / 60) 
            random_num = random.randint(8,16) * bps
            total =total + random_num
            self.sections.append(total)

        #for x in range(0,len(audio_analysis_dict["sections"])):
        #    self.sections.append(audio_analysis_dict["sections"][x]["start"])

    # parses the genre list returned by spotify, in order to get a general genre
    # E.g Sam Cooke = ['Classic Soul','Soul','Vocal Jazz'] -->"Soul"
    def decide_genre(self, list_of_genres):
        genre = "none"
        #print(list_of_genres)
        for genre_name in list_of_genres:
            if "soul" in genre_name:
                genre = "soul"
            if "pop" in genre_name:
                genre = "pop"
            if "rock" in genre_name:
                genre = "rock"
            if "metal" in genre_name:
                genre = "metal"
            if "hip-hop" in genre_name:
                genre = "hip-hop"
            if "classical" in genre_name:
                genre = "classical"
            if "electr" in genre_name:
                genre = "electronic"
            # If a genre has been found, doesn't need to iterate through the other genres
            if genre != "none":
                #print(genre)
                break
        print(f"Genre:{self.genre}")
        self.genre = genre
    
    def retrieve_data(self, data, data_type):
        """
            Used in case data does not exist
        """
        try:
            data = self.get_data(data)[data_type]
        except Exception:
            data = self.get_data(data)
        return data
    
    def get_data(self, data):
        """
        Send the POST request to the GraphQL endpoint with error handling
        """
        try:
            response = self.session.post(self.base_url, json=data, headers=self.headers)
            response.raise_for_status()  # Raises an HTTPError for bad responses
            response_json = response.json()
            if 'data' in response_json and 'spotifyTrackEnqueue' in response_json['data']:
                data = response_json['data']['spotifyTrackEnqueue']
                if 'enqueuedSpotifyTrack' in data:
                    if 'result' in data['enqueuedSpotifyTrack']['audioAnalysisV6'].keys():
                        track_data = data['enqueuedSpotifyTrack']['audioAnalysisV6']['result']
                    else:
                        track_data = {}
                    return track_data
                elif 'Error' in data:
                    print(f"Error: {data['Error']['message']}")
            else:
                print("Unexpected response format:", response_json)
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")
            return {}
        return {}
    # needed to access Spotify App
    def get_token(self):
        auth_string = self.client_id + ":" + self.client_secret
        auth_bytes = auth_string.encode("utf-8")
        auth_base64 = str(base64.b64encode(auth_bytes), "utf-8")

        url = "https://accounts.spotify.com/api/token"
        headers = { 
            "Authorization": "Basic " + auth_base64,
            "Content-Type": "application/x-www-form-urlencoded"
        }
        data = {"grant_type": "client_credentials"}
        result = post(url, headers=headers, data=data)
        json_result = json.loads(result.content)
        token = json_result["access_token"]
        return token
    
    # needed to access Spotify App
    def get_auth_headers(self,token):
        return {"Authorization": "Bearer " + token}

    # change dance moves based on spotify genre
    def change_dance(self):
        if self.genre == "pop":
            #print("Genre : Pop")
            dances_for_genre = [0,2]
            genre_index = random.randint(0,len(dances_for_genre)-1)
            move_index = dances_for_genre[genre_index]
            self.dance_name = self.dance_names[move_index]

        elif self.genre == "soul":
            #print("Genre : Soul")
            dances_for_genre = [0,3]
            genre_index = random.randint(0,len(dances_for_genre)-1)
            move_index = dances_for_genre[genre_index]
            self.dance_name = self.dance_names[move_index]

        elif self.genre == "electronic":
            #print("Genre : Electronic")
            dances_for_genre = [1,2]
            genre_index = random.randint(0,len(dances_for_genre)-1)
            move_index = dances_for_genre[genre_index]
            self.dance_name = self.dance_names[move_index]

        elif self.genre == "rock":
            #print("Genre : Rock")
            dances_for_genre = [1,2]
            genre_index = random.randint(0,len(dances_for_genre)-1)
            move_index = dances_for_genre[genre_index]
            self.dance_name = self.dance_names[move_index]

        elif self.genre == "hip-hop":
            #print("Genre : Hip-Hop")
            dances_for_genre = [2,3]
            genre_index = random.randint(0,len(dances_for_genre)-1)
            move_index = dances_for_genre[genre_index]
            self.dance_name = self.dance_names[move_index]
        else:
            print("No Specific Genre")
            #Any move
            index = random.randint(0,3)
            self.dance_name = self.dance_names[index]

    # Non-Dancing Sequences ------------------------------------------------------------------

    def s_curve_generator(self,mx=1, mn=0, freq=1, phase=0 ,t=1.0 ,t0=0):
        if t > (0.5/freq):
            curve = mn
        else:
            curve = ((mx-mn)) * (np.cos(freq*(t-t0)*math.pi*2+phase) / 2) + (mx - ((mx-mn)/2))
        return curve

    def sine_generator(self,mx=1, mn=0, freq=1, phase=0 ,t=1.0 ,t0=0):
        return ((mx-mn)) * (np.sin(freq*(t-t0)*2*math.pi+phase) / 2) + (mx - ((mx-mn)/2))

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

    def play_dance_music(self):
        self.now_dancing = True

        # Convert to bytes
        wav_data = np.array(self.data, dtype=np.int16).tobytes()
        
        self.wave_obj = sa.play_buffer(wav_data, 1, 2, 24000)
    # Text to Wav
    def text_to_wav(self, reply):
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
            #self.pub_stream.publish(msgse)
            self.talking = True
        else:
            self.talking = False
            self.song_over = True

        rate.sleep() 
