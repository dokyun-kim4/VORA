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


# Recorded audio parameters
FRAMES_PER_BUFFER = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
seconds = 3

# Classes
label_names = ["dexter", "dokyun", "dominic", "mo"]

# Import the classifier
model_name = "./voice_detection/new3s.p"
model_dict = pickle.load(open(model_name, "rb"))
model = model_dict["model"]

# Key words
trigger = "hey"
ender = "off"

continuous = True  # Loop until user says stop

while continuous:
    # Initialize the recognizer
    r = sr.Recognizer()
    # Exception handling to handle
    # exceptions at the runtime
    try:
        # use the microphone as source for input.
        with noalsaerr():
            with sr.Microphone() as source2:

                # wait for a second to let the recognizer
                # adjust the energy threshold based on
                # the surrounding noise level
                r.adjust_for_ambient_noise(source2, duration=0.5)

                r.pause_threshold = 1
                print("listening...")

                # listens for the user's input
                audio2 = r.listen(source2)

                # Using google to recognize audio
                MyText = r.recognize_google(audio2)
                MyText = MyText.lower()

                print("Did you say ", MyText)

                if trigger in MyText.split():
                    SpeakText("Listening")
                    # with noalsaerr():
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

                    # Initialize recorded audio recognizer class
                    r2 = sr.Recognizer()

                    # audio object
                    audio = sr.AudioFile("live_audio.wav")

                    # read audio object and transcribe

                    try:
                        with audio as source:
                            audio = r2.record(source)
                            result = r2.recognize_google(audio)
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

                        if "off" in MyText.split():
                            print(MyText.split())
                            SpeakText("Turning off")
                            continuous = False

        if ender in MyText.split():
            SpeakText("Turning off")
            continuous = False

    except sr.RequestError as e:
        print("Could not request results; {0}".format(e))

    except sr.UnknownValueError:
        print("unknown error occurred")
