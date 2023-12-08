# python -m pip install pyaudio
import pyaudio
import wave
import matplotlib.pyplot as plt
import numpy as np
import os
import keyboard

DATA_DIR = "./audio_helpers/data"
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)

FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

filename = "dominic"

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

on = True
frames = []
second_tracking = 0
second_count = 0

print("Press s to stop recording!")

while on:
    data = stream.read(FRAMES_PER_BUFFER)
    frames.append(data)
    second_tracking += 1
    if second_tracking == RATE / FRAMES_PER_BUFFER:
        second_count += 1
        second_tracking = 0
        print(f"Recording for {second_count} seconds")
    try:
        if keyboard.is_pressed("s"):
            print("Done recording")
            break
    except:
        pass


stream.stop_stream()
stream.close()
pa.terminate()

obj = wave.open(os.path.join(DATA_DIR, f"{filename}.wav"), "wb")
obj.setnchannels(CHANNELS)
obj.setsampwidth(pa.get_sample_size(FORMAT))
obj.setframerate(RATE)
obj.writeframes(b"".join(frames))
obj.close()
