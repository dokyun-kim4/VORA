import speech_recognition as sr
import pyttsx3
import pickle
import pyaudio
import tty
import sys
import termios
import wave
import tensorflow as tf
from ctypes import *
from helpers import preprocess
from contextlib import contextmanager

# Filter out unecessary errors
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)

# Function to convert text to
# speech
def SpeakText(command):
    # Initialize the engine
    engine = pyttsx3.init()
    engine.say(command)
    engine.runAndWait()


FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
seconds = 3

# for key inputs
orig_settings = termios.tcgetattr(sys.stdin) 
tty.setcbreak(sys.stdin)
x = 0

label_names = ["dexter", "dokyun", "dominic", "mo"]

model_name = "./voice_detection/new3s.p"
model_dict = pickle.load(open(model_name, "rb"))
model = model_dict["model"]

running = True

while running: 

    printed = False

    while True:
        if not printed:
            print("Press q to record or s to stop!")
            printed = True
        try:
            x=sys.stdin.read(1)[0]
            if x == "q":
                break
            if x == "s":
                print("Turning off")
                running = False
                break
        except:
            pass

    if running:

        with noalsaerr():
            pa = pyaudio.PyAudio()

        stream = pa.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=FRAMES_PER_BUFFER,
        )

        frames = []
        second_tracking = 0
        second_count = 0


        print('Recording')
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

        # Initialize recognizer class
        r = sr.Recognizer()

        # audio object
        audio = sr.AudioFile("live_audio.wav")

        # read audio object and transcribe

        try:
            with audio as source:
                audio = r.record(source)
                result = r.recognize_google(audio)
                result = result.split()

            print(result)

            for word in result:
                match word:
                    case "forward":
                        SpeakText(f"Hi {label_names[max_idx]}, moving forward")
                    case "backward":
                        SpeakText(f"Hi {label_names[max_idx]}, moving backward")
                    case "right":
                        SpeakText(f"Hi {label_names[max_idx]}, turning right")
                    case "left":
                        SpeakText(f"Hi {label_names[max_idx]}, turning left")
                    case "stop":
                        SpeakText(f"Hi {label_names[max_idx]}, stopping")

        except sr.RequestError as e:
            print("Could not request results; {0}".format(e))

        except sr.UnknownValueError:
            print("unknown error occurred")
