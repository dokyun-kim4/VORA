import pickle
import pyaudio
import keyboard
import wave
import tensorflow as tf
from helpers import preprocess

FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

label_names = ["dexter", "dokyun", "dominic", "mo"]

seconds = 3

model_name = "./voice-detection/working.p"
model_dict = pickle.load(open(model_name, "rb"))
model = model_dict["model"]

pa = pyaudio.PyAudio()

stream = pa.open(
    format=FORMAT,
    channels=CHANNELS,
    rate=RATE,
    input=True,
    frames_per_buffer=FRAMES_PER_BUFFER,
)

printed = False
while True:
    if not printed:
        print("Press q to record!")
        printed = True
    try:
        if keyboard.is_pressed("q"):
            print("Recording")
            break
    except:
        pass

frames = []
second_tracking = 0
second_count = 0
for i in range(0, int(RATE / FRAMES_PER_BUFFER * seconds)):
    data = stream.read(FRAMES_PER_BUFFER)
    frames.append(data)
    second_tracking += 1
    if second_tracking == RATE / FRAMES_PER_BUFFER:
        second_count += 1
        second_tracking = 0
        print(f"Time Left: {seconds - second_count} seconds")

stream.stop_stream()
stream.close()
pa.terminate()

obj = wave.open("live_audio.wav", "wb")
obj.setnchannels(CHANNELS)
obj.setsampwidth(pa.get_sample_size(FORMAT))
obj.setframerate(RATE)
obj.writeframes(b"".join(frames))
obj.close()

audio = preprocess("live_audio.wav")
audio = tf.expand_dims(audio[0], axis=0)

prediction = list(model.predict(audio)[0])
max_idx = prediction.index(max(prediction))

print(prediction)
print(label_names[max_idx])
