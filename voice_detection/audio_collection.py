# python -m pip install pyaudio
import pyaudio
import wave
import matplotlib.pyplot as plt
import numpy as np
import os
import keyboard

DATA_DIR = "./data"
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)

FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

number_of_classes = 3
dataset_size = 20
seconds = 5

for class_num in range(number_of_classes):
    if not os.path.exists(os.path.join(DATA_DIR, str(class_num))):
        os.makedirs(os.path.join(DATA_DIR, str(class_num)))

    for record_num in range(dataset_size):
        pa = pyaudio.PyAudio()

        stream = pa.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=FRAMES_PER_BUFFER,
        )

        print(f"Recording for class {class_num} record {record_num}")

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

        obj = wave.open(
            os.path.join(DATA_DIR, str(class_num), f"{record_num}.wav"), "wb"
        )
        obj.setnchannels(CHANNELS)
        obj.setsampwidth(pa.get_sample_size(FORMAT))
        obj.setframerate(RATE)
        obj.writeframes(b"".join(frames))
        obj.close()
