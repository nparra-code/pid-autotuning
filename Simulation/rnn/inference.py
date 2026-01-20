"""  
@file inference.py
@brief RNN model inference for PID prediction
@details Loads trained LSTM model and predicts PID gains from test data
"""

from functions import *

X, _ = load_dataset("/content/drive/MyDrive/pid_autotuning/data/to_predict", seq_len=20)
y_pred = model.predict(X, verbose=1)
mean_gains = np.mean(y_pred, axis=0)
print(np.round(mean_gains, 2))

print(f'Kp = np.array([{mean_gains[0]}, {mean_gains[1]}, {mean_gains[2]}])')
print(f'Ki = np.array([{mean_gains[3]}, {mean_gains[4]}, {mean_gains[5]}])')
print(f'Kd = np.array([{mean_gains[6]}, {mean_gains[7]}, {mean_gains[8]}])')