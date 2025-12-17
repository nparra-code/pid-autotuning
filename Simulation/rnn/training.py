from functions import *

import numpy as np
import tensorflow as tf
import keras
from keras import layers
from sklearn.preprocessing import MinMaxScaler

# 2. Load and prepare sequences
X, y = load_experimental_dataset("../../pid-autotuning-firmware/to_train/", seq_len=20)

model = keras.Sequential()
# Add an Embedding layer expecting input vocab of size 1000, and
# output embedding dimension of size 64.
# model.add(layers.Embedding(input_dim=1000, output_dim=64)) # Embedding layer is not needed for this task

model_name = "3wheel_model_lstm_128_tanh_experimental"
model = keras.Sequential([
    layers.LSTM(128, input_shape=(None, 15), activation='tanh', return_sequences=False),
    layers.Dense(64, activation='relu'),
    layers.Dense(9)  # [Kp, Ki, Kd]
])

model.summary()

# Compile the model
model.compile(optimizer='adam',
              loss='mse',  # Mean Squared Error is a common loss function for regression tasks
              metrics=['mae']) # Mean Absolute Error is another useful metric for regression



print("X shape:", X.shape)
print("y shape:", y.shape)
print(model(X[:1]).shape)

# Train the model
# You can adjust the number of epochs and batch size as needed
history = model.fit(X, y, epochs=20, batch_size=32, validation_split=0.2)
model.save(f"models/{model_name}.h5")

print("Training complete.")