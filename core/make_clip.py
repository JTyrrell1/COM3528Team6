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
from scipy.io.wavfile import write
import wave
import simpleaudio as sa


class MakeClip():

    def __init__(self):

        # flags
        self.talking = False

        # AI Voice
        self.client = ElevenLabs(
            api_key = ""
        )

        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        self.pub_stream = rospy.Publisher(self.topic_base_name + "/control/stream", Int16MultiArray, queue_size=1)
        rospy.init_node('make_clip')
        

    def text_to_speaker(self, text_to_say):
        # mp3_fp = BytesIO()
        audio = self.client.generate(
            text = text_to_say,
            voice = Voice(
                voice_id = "",
                settings = VoiceSettings(stability=0.5, similarity_boost=0.75)

            )
        )

        # Consume the generator to get the audio data
        audio_byte = b''.join(audio)

        # use BytesIO for in-memory processing
        audio_data = BytesIO(audio_byte)

        seg = AudioSegment.from_file(audio_data, format='mp3')
        seg = seg.set_frame_rate(24000)
        seg = seg.set_channels(1)

        data_to_save = seg

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

        # Convert to bytes
        self.speak()
        wav_data = np.array(self.data, dtype=np.int16).tobytes()
        wave_obj = sa.play_buffer(wav_data, 1, 2, 24000)

        # Saving to file
        print("Do you want to save it? Y or N:")
        response = input()

        if response in ['y', 'Y']:
            folder_path = f'./miro_clips_HD/{text_to_say}'
            if not os.path.isdir(folder_path): os.mkdir(folder_path)
            count = 0

            for file in os.listdir(folder_path):
                count += 1

            file = f"/{text_to_say}" + "_" + str(count) + ".wav"
            file_path = folder_path + file

            data_to_save.export(file_path, format='wav')
            print(f"Saved as {file}\n")
            print(f"data to save: {data_to_save.frame_rate}")
        else:
            print("\n")

    def text_to_speech(self, text_to_say):
        # mp3_fp = BytesIO()
        audio = self.client.generate(
            text = text_to_say,
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
        seg = seg.set_frame_rate(24000)
        seg = seg.set_channels(1)

        data_to_save = seg

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

        self.speak()

        # Saving to file
        print("Do you want to save it? Y or N:")
        response = input()

        if response in ['y', 'Y']:
            folder_path = f'./miro_clips/{text_to_say}'
            if not os.path.isdir(folder_path): os.mkdir(folder_path)
            count = 0

            for file in os.listdir(folder_path):
                count += 1

            file = f"/{text_to_say}" + "_" + str(count) + ".wav"
            file_path = folder_path + file

            data_to_save.export(file_path, format='wav')
            print(f"Saved as {file}\n")
            print(f"data to save: {data_to_save.frame_rate}")
        else:
            print("\n")

    def speak(self):
        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            if (self.d < len(self.data)):                
                msg = Int16MultiArray(data = self.data[self.d:self.d + 1000])
                self.d += 1000
                #self.pub_stream.publish(msg)
            else:
                self.talking = False
                break
            rate.sleep()
    
    def main(self):
        while not rospy.core.is_shutdown():
            if not self.talking:
                print('Enter what MiRo should say:')
                text_to_say = input()
                print("\n")
                self.talking = True
                self.text_to_speaker(text_to_say)

class NamedBytesIO(BytesIO):
    def __init__(self, name, data=None):
        super().__init__(data if data is not None else b'')
        self.name = name
    
if __name__ == "__main__":
    make_clip = MakeClip()
    make_clip.main()
        
