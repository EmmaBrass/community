import os
import wave
from piper.voice import PiperVoice

voicedir = os.path.expanduser('~/Documents/piper/') #Where onnx model files are stored on my machine
print("HERE")
model = voicedir+"en_GB-alba-medium.onnx"
voice = PiperVoice.load(model)
wav_file = wave.open('output.wav', 'w')
text = "This is an example of text-to-speech using Piper TTS."
audio = voice.synthesize(text,wav_file)

os.system('aplay output.wav')