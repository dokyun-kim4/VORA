import os
from matplotlib import pyplot as plt
import tensorflow as tf
import tensorflow_io as tfio
import pickle
from helpers import preprocess, train_preprocess
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import (
    Conv2D,
    Dense,
    Flatten,
    Resizing,
    Input,
    Normalization,
    Dropout,
    MaxPooling2D,
)


# Make filepaths to extract training data
N = os.path.join("old_train_data", "0")
T = os.path.join("old_train_data", "1")
M = os.path.join("old_train_data", "2")

n = tf.data.Dataset.list_files(N + "\*.wav")
t = tf.data.Dataset.list_files(T + "\*.wav")
m = tf.data.Dataset.list_files(M + "\*.wav")

nans = tf.data.Dataset.zip((n, tf.data.Dataset.from_tensor_slices(tf.zeros(len(n)))))
tats = tf.data.Dataset.zip((t, tf.data.Dataset.from_tensor_slices(tf.ones(len(t)))))
mos = tf.data.Dataset.zip((m, tf.data.Dataset.from_tensor_slices(2 * tf.ones(len(m)))))
data = nans.concatenate(tats).concatenate(mos)

# Preprocess training data
data = data.map(train_preprocess)
# data = data.cache()
data = data.shuffle(buffer_size=1000)
data = data.batch(1)
data = data.prefetch(8)

# Train test split
train = data.take(40)
test = data.skip(40).take(20)

# Define model
model = Sequential(
    [
        Input(shape=(2491, 257, 1)),
        # Downsample the input.
        Resizing(32, 32),
        # Normalize.
        Conv2D(32, 3, activation="relu"),
        Conv2D(64, 3, activation="relu"),
        MaxPooling2D(),
        Dropout(0.25),
        Flatten(),
        Dense(128, activation="relu"),
        Dropout(0.5),
        Dense(3, activation="softmax"),
    ]
)

# Compile and train model
model.compile(
    optimizer="adam",
    loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
    metrics=["accuracy"],
)

hist = model.fit(train, epochs=4, validation_data=test, verbose=False)

# Export model
f = open("new_classifier.p", "wb")
pickle.dump({"model": model}, f)
f.close()
