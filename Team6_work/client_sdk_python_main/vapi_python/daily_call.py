import daily
import threading
import pyaudio
import json
import rospy
import numpy as np
from std_msgs.msg import Int16MultiArray

SAMPLE_RATE = 8000
NUM_CHANNELS = 1
CHUNK_SIZE = 640

def is_playable_speaker(participant):
    info = participant.get("info", {})
    mic = participant.get("media", {}).get("microphone", {})
    return (
        info.get("userName") == "Vapi Speaker" and
        mic.get("subscribed") == "subscribed" and
        mic.get("state") == "playable"
    )

class DailyCall(daily.EventHandler):
    def __init__(self):
        daily.Daily.init()
        self.audio = pyaudio.PyAudio()

        # Init ROS
        if not rospy.core.is_initialized():
            rospy.init_node("vapi_to_miro_streamer", anonymous=True, disable_signals=True)
        self.ros_pub = rospy.Publisher("/miro/control/stream", Int16MultiArray, queue_size=1)

        # PyAudio setup
        self.input_stream = self.audio.open(format=pyaudio.paInt16, channels=NUM_CHANNELS,
                                            rate=SAMPLE_RATE, input=True, frames_per_buffer=CHUNK_SIZE)
        self.output_stream = self.audio.open(format=pyaudio.paInt16, channels=NUM_CHANNELS,
                                             rate=SAMPLE_RATE, output=True, frames_per_buffer=CHUNK_SIZE)

        self.mic_dev = daily.Daily.create_microphone_device("my-mic", sample_rate=SAMPLE_RATE, channels=NUM_CHANNELS)
        self.spk_dev = daily.Daily.create_speaker_device("my-spk", sample_rate=SAMPLE_RATE, channels=NUM_CHANNELS)
        daily.Daily.select_speaker_device("my-spk")

        self.client = daily.CallClient(event_handler=self)
        self.client.update_inputs({
            "camera": False,
            "microphone": {
                "isEnabled": True,
                "settings": {
                    "deviceId": "my-mic",
                    "customConstraints": {
                        "autoGainControl": {"exact": True},
                        "noiseSuppression": {"exact": True},
                        "echoCancellation": {"exact": True}
                    }
                }
            }
        })
        self.client.update_subscription_profiles({
            "base": {
                "camera": "unsubscribed",
                "microphone": "subscribed"
            }
        })

        self.participants = {k: v for k, v in self.client.participants().items() if k != "local"}
        self.quit = False
        self.err = None
        self.joined = False
        self.inputs_ready = False
        self.start_event = threading.Event()

        self.recv_thread = threading.Thread(target=self.recv_audio)
        self.send_thread = threading.Thread(target=self.send_audio)
        self.recv_thread.start()
        self.send_thread.start()

    def on_inputs_updated(self, _): self.inputs_ready = True; self._maybe_start()
    def on_joined(self, _, err): self.err = err; self.joined = not err; self._maybe_start()
    def on_participant_joined(self, p): self.participants[p["id"]] = p
    def on_participant_left(self, p, _): self.participants.pop(p["id"], None); self.leave()
    def on_participant_updated(self, p):
        self.participants[p["id"]] = p
        if is_playable_speaker(p): self.client.send_app_message("playable")

    def join(self, url): self.client.join(url, completion=self.on_joined)

    def leave(self):
        self.quit = True
        self.recv_thread.join()
        self.send_thread.join()
        self.client.leave()
        rospy.signal_shutdown("DailyCall terminated cleanly")

    def _maybe_start(self):
        if self.err or (self.inputs_ready and self.joined):
            self.start_event.set()

    def send_audio(self):
        self.start_event.wait()
        if self.err:
            print("Mic error")
            return
        while not self.quit:
            buf = self.input_stream.read(CHUNK_SIZE, exception_on_overflow=False)
            if buf:
                self.mic_dev.write_frames(buf)

    def recv_audio(self):
        self.start_event.wait()
        if self.err:
            print("Speaker error")
            return
        while not self.quit and not rospy.core.is_shutdown():
            buf = self.spk_dev.read_frames(CHUNK_SIZE)
            if not buf:
                continue
            self.output_stream.write(buf, CHUNK_SIZE)

            # Gain boost + ROS publish
            gain = 3
            samples = np.frombuffer(buf, dtype=np.int16).astype(np.float32) * gain
            samples = np.clip(samples, -32768, 32767).astype(np.int16)
            self.ros_pub.publish(Int16MultiArray(data=samples.tolist()))

    def send_app_message(self, msg):
        try:
            self.client.send_app_message(json.dumps(msg))
        except Exception as e:
            print("App message error:", e)
