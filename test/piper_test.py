import os
import wave
from piper.voice import PiperVoice

voicedir = os.path.expanduser('~/Documents/piper/') #Where onnx model files are stored on my machine
print("HERE")
model = voicedir+"en_GB-jenny_dioco-medium.onnx"
voice = PiperVoice.load(model)
wav_file = wave.open('connected.wav', 'w')
text = "Hello, I'm here."
audio = voice.synthesize(text,wav_file)

os.system('aplay connected.wav')