import scipy.io.wavfile as wav
import numpy as np

def splitWav(read_path: str, split_time: float, write_path: str):
    """
    Splits a .wav file into multiple .wav files of the same length

    Parameters:
        read_path (str): The path to the .wav file to split
        split_time (float): The length of each split in seconds
        write_path (str): The path to write the split files to. Format string with one argument for the split number.
    
    Returns:
        None
    """

    # set types for the rate and data since wav.read doesn't have type hints
    rate: int
    data: np.ndarray

    # read the wav file
    rate, data = wav.read(read_path)

    # calculate the number of samples in each split
    samples: int = round(rate*split_time)

    # save splits until there are no more samples
    i: int = 0
    while data.shape[0] > 0:

        # if there are more samples than the split length,
        if data.shape[0] > samples:
            # save the split length
            wav.write(write_path.format(i), rate, data[:samples])
            # remove the split length from the data
            data = data[samples:]

        # otherwise,
        else:
            # save the remaining samples
            wav.write(write_path.format(i), rate, data)
            # remove the remaining samples from the data
            data = data[0:0]

        # increment the split number
        i += 1

# test the function if this file is not imported
if __name__ == "__main__":
    splitWav('audio_helpers/CantinaBand60.wav', 10, 'audio_helpers/wav_split/CantinaBand60_{}.wav')
