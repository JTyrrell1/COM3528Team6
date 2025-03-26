#!/user/bin/env python3
import numpy as np
import pvporcupine
import pvkoala
import node
import os
import rospy
import pvkoala
import wave,struct
import pydub
import asyncio
from shazamio import Shazam

MIC_SAMPLE_RATE = 20000

SAMPLE_COUNT = 8 * MIC_SAMPLE_RATE

# How many seconds between checking for music in the environment
LENGTH_BETWEEN_CHECKS = 5

class NodeDetectMusic(node.Node):

    def __init__(self, sys):

        self.mic_data = np.zeros((0, 4), 'uint16')      # the raw sound data obtained from the MiRo's message.
        self.micbuf = np.zeros((SAMPLE_COUNT, 4), 'int64')        # the raw sound data that is has been recorded for use. This has been recorded and passes to be process when it has enough sample count.
        #self.micbuf = np.asarray(self.micbuf, 'float32') 
        cwd = os.getcwd()
        for root, dirs, files in os.walk(cwd):
            if "miro_recordings" in dirs:
                self.directory = os.path.join(root, "miro_recordings")

        self.song_name = ""
        self.timer = rospy.get_time()
        self.listening = True

    def tick(self, msg):
        """
            Process audio and the recording to be sent to shazamio
        """

        if self.listening:
            data = np.asarray(msg.data)
            self.mic_data = np.transpose(data.reshape((4, 500)))

            cur_time = rospy.get_time() 
            difference = cur_time - self.timer

            if difference > LENGTH_BETWEEN_CHECKS:
                if not self.micbuf is None:
                    # gives point in recording (0-8 secs)
                    cur_time = difference - LENGTH_BETWEEN_CHECKS

                    sample_loc = int(round(cur_time/0.025))*500

                    if not sample_loc >= SAMPLE_COUNT:
                        self.micbuf[sample_loc:sample_loc+500,:] = self.mic_data

                        if self.micbuf[sample_loc-10].all() == 0 and not sample_loc == 0 :
                            self.micbuf[sample_loc-500:sample_loc] = self.mic_data

                    if sample_loc >= SAMPLE_COUNT:
                        outbuf = self.micbuf[:SAMPLE_COUNT]

                        # get audio from left/right ear of Miro

                        outfilename = self.directory + "/miro_audio.wav"
                        file = wave.open(outfilename , "w")
                        
                        file.setsampwidth(2)
                        file.setframerate(MIC_SAMPLE_RATE)

                        file.setnchannels(2)
                        detect_sound = np.reshape(outbuf[:, [0,1]], (-1))
                        
                        for s in detect_sound:
                            file.writeframes(struct.pack('<h',s))
                        file.close()

                        sound = pydub.AudioSegment.from_wav(outfilename)

                        # convert to ogg for shazamio
                        ogg_output = self.directory + "/miro_audio.ogg"
                        sound.export(ogg_output,format="ogg")
                        
                        # stop listening when recording completed
                        self.start_listening = False

                        # Find song through shazamio ----------------------------
                        async def findSong():
                            shazam = Shazam()
                            out = await shazam.recognize(self.directory + "/miro_audio.ogg")

                            if len(out["matches"]) == 0:
                                self.song_name = ""
                                print("No song found, check the audio is recording correctly and try again")
                                
                            else:
                                self.song_name = out["track"]["title"]
                            
                        self.timer = rospy.get_time()  
                        loop = asyncio.new_event_loop()
                        loop.run_until_complete(findSong())                          
    


        