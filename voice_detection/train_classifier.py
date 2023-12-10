import os
from matplotlib import pyplot as plt
import tensorflow as tf
import tensorflow_io as tfio
import pickle
from helpers import preprocess, train_preprocess, make_data, make_data_old
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
folder = "./audio_helpers/data"
classes = ["dexter", "dokyun", "dominic", "mo"]

# dataset = make_data(folder, classes, 80)
dataset = make_data_old(folder, classes)
data_size = len(list(dataset))
print(data_size)

# Preprocess training data
data = dataset.map(train_preprocess)
# data = data.cache()
data = data.shuffle(buffer_size=1000)
data = data.batch(1)
data = data.prefetch(8)

# Train test split
train_split = 0.8
test_split = 0.2

train_size = int(data_size * train_split)
test_size = int(data_size * test_split)

train = data.take(train_size)
test = data.skip(train_size).take(test_size)

data_shape = tf.squeeze(train.as_numpy_iterator().next()[0], axis=0).shape
# data_shape = train.as_numpy_iterator().next()[0].shape
classes_size = len(classes)

# Define model
model = Sequential(
    [
        Input(shape=data_shape),
        # Downsample the input.
        Resizing(128, 128),
        # Normalize.
        Conv2D(32, 3, activation="relu"),
        MaxPooling2D(),
        Conv2D(64, 3, activation="relu"),
        MaxPooling2D(),
        Conv2D(32, 3, activation="relu"),
        MaxPooling2D(),
        Conv2D(64, 3, activation="relu"),
        MaxPooling2D(),
        Dropout(0.25),
        Flatten(),
        Dense(128, activation="relu"),
        Dropout(0.5),
        Dense(classes_size, activation="softmax"),
    ]
)

# Compile and train model
model.compile(
    optimizer="adam",
    loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
    metrics=["accuracy"],
)

history = model.fit(train, epochs=15, validation_data=test, verbose=True)

plt.plot(history.history["accuracy"])
plt.plot(history.history["val_accuracy"])
plt.title("model accuracy")
plt.ylabel("accuracy")
plt.xlabel("epoch")
plt.legend(["train", "val"], loc="upper left")
plt.show()

plt.plot(history.history["loss"])
plt.plot(history.history["val_loss"])
plt.title("model loss")
plt.ylabel("loss")
plt.xlabel("epoch")
plt.legend(["train", "val"], loc="upper left")
plt.show()

# Export model
f = open("voice-detection/new3s.p", "wb")
pickle.dump({"model": model}, f)
f.close()
