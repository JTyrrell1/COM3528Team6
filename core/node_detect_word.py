#!/user/bin/env python3
import numpy as np
import pvporcupine
import pvkoala
import node
import os
import random
from std_msgs.msg import Int16MultiArray
from pydub import AudioSegment
import rospy

SAMPLE_COUNT = 640

class NodeDetectWord(node.Node):

    def __init__(self, sys, mic_choice):
        """
            Update the detected words with the messages being published
        """

        # Microphone choice
        self.mic_choice = mic_choice
        node.Node.__init__(self, sys, "affect")
        mic_num = 4 if mic_choice == "miro" else 1
        self.mic_data = np.zeros((0, mic_num), 'uint16')      # the raw sound data obtained from the MiRo's message.
        self.micbuf = np.zeros((0, mic_num), 'uint16')        # the raw sound data that is has been recorded for use. This has been recorded and passes to be process when it has enough sample count.

        # Mic publisher
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.pub_stream = rospy.Publisher(self.topic_base_name + "/control/stream", Int16MultiArray, queue_size=0)
        
        # porcupine access
        self.access_key_hey_miro = ""
        self.access_key_dance = ""
        self.access_key_stop_miro = ""
        self.handle_dance = pvporcupine.create(access_key=self.access_key_dance,
                            keywords=['dance miro'],
                            keyword_paths=['porcupine_trained_model/Dance-MiRo_en_linux_v3_0_0.ppn'])

        self.handle_hey_miro = pvporcupine.create(access_key=self.access_key_hey_miro,
                            keywords=['hey google'],
                            keyword_paths=['porcupine_trained_model/Hey-Miro_en_linux_v3_0_0.ppn'])
        
        self.handle_stop_miro = pvporcupine.create(access_key=self.access_key_stop_miro,
                            keywords=['stop miro'],
                            keyword_paths=['porcupine_trained_model/Stop-MiRo_en_linux_v3_0_0.ppn'])
        self.word_action = ""
        self.hey_keyword_index = -1
        self.koala = pvkoala.create("")

        self.wake_word_enabled = True 
        self.hey_miro = False
        self.stop_miro = False
        self.dancing = False
        self.song_name = ""
        self.gesture_mode = False
        self.obj_detect_mode = False
        self.ear_count = 0
        self.tail_count = 0

        self.from_LLM = False
        self.llm = False
        self.dance = False 

    def tick(self, msg):
        """
            Process audio for wake word and the recording to be sent for speech to text
        """

        if self.mic_choice == "miro":
            # reshape into 4 x 500 array
            data = np.asarray(msg.data)
            self.mic_data = np.transpose(data.reshape((4, 500)))

        elif self.mic_choice == "mic":
            # Transposing adds a dimension to ease computation (Only one microphone, so just 1 dimension)
            data = np.asarray(msg.data, dtype=np.int16)
            self.mic_data = np.transpose(data.reshape((1, 500)))  

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
                #outbuf_dwnsample = np.append(self.koala.process(outbuf_dwnsample[0:256]), self.koala.process(outbuf_dwnsample[256:512]))
                self.stop_miro = True if self.handle_stop_miro.process(outbuf_dwnsample) != -1 else False
                # check for any wake words
                if self.wake_word_enabled == True:
                    
                    dance_keyword_index = self.handle_dance.process(outbuf_dwnsample)
                    self.hey_keyword_index = self.handle_hey_miro.process(outbuf_dwnsample)
                    if self.hey_keyword_index != -1:
                        print("hey miro heard")
                        self.word_action = "hey_miro"
                        self.llm = True
                    elif self.hey_miro:
                        self.word_action = "hey_miro"
                        self.llm = True
                        self.hey_miro = False
                    elif dance_keyword_index != -1 or self.dancing:
                        print("dance_miro heard :D !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        self.word_action = "dance"
                        self.dance = True
                    elif self.gesture_mode:
                        self.word_action = "gesture"
                    elif self.obj_detect_mode:
                        self.word_action = "obj_detect"
                    else:
                        self.word_action = ""
                #print(self.word_action)