import tensorflow as tf
import tensorflow_io as tfio
import os


def load_wav_16k_mono(filename):
    # Load encoded wav file
    file_contents = tf.io.read_file(filename)
    # Decode wav (tensors by channels)
    wav, sample_rate = tf.audio.decode_wav(file_contents, desired_channels=1)
    # Removes trailing axis
    wav = tf.squeeze(wav, axis=-1)
    sample_rate = tf.cast(sample_rate, dtype=tf.int64)
    # Goes from 44100Hz to 16000hz - amplitude of the audio signal
    wav = tfio.audio.resample(wav, rate_in=sample_rate, rate_out=16000)
    return wav


def preprocess(file_path, *args):
    wav = load_wav_16k_mono(file_path)
    spectrogram = tf.signal.stft(wav, frame_length=500, frame_step=250)
    spectrogram = tf.abs(spectrogram)
    spectrogram = tf.expand_dims(spectrogram, axis=2)
    return spectrogram, args


def train_preprocess(file_path, label):
    wav = load_wav_16k_mono(file_path)
    spectrogram = tf.signal.stft(wav, frame_length=500, frame_step=250)
    spectrogram = tf.abs(spectrogram)
    spectrogram = tf.expand_dims(spectrogram, axis=2)
    return spectrogram, label


def make_data(data_folder: str, classes: list):
    data = []
    for idx, class_name in enumerate(classes):
        path = os.path.join(data_folder, class_name)
        pathset = tf.data.Dataset.list_files(path + "/*.wav")

        if data == []:
            data = tf.data.Dataset.zip(
                (pathset, tf.data.Dataset.from_tensor_slices(tf.zeros(len(pathset))))
            )
        else:
            dataset = tf.data.Dataset.zip(
                (
                    pathset,
                    tf.data.Dataset.from_tensor_slices(idx * tf.ones(len(pathset))),
                )
            )
            data = data.concatenate(dataset)
    return data
