"""  
@file training.py
@brief RNN model training for PID autotuning
@details Implements LSTM neural network training for learning optimal PID gains
         from experimental telemetry data. Includes comprehensive evaluation and
         visualization of training results.
"""

from functions import *

import numpy as np
import tensorflow as tf
import keras
from keras import layers
from sklearn.preprocessing import MinMaxScaler
import matplotlib.pyplot as plt

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

# ===== VISUALIZATION AND EVALUATION =====
print("\n" + "="*50)
print("TRAINING METRICS AND EVALUATION")
print("="*50 + "\n")

# 1. Plot Training History - Loss
plt.figure(figsize=(14, 5))

plt.subplot(1, 2, 1)
plt.plot(history.history['loss'], label='Training Loss', linewidth=2)
plt.plot(history.history['val_loss'], label='Validation Loss', linewidth=2)
plt.title('Model Loss Over Epochs', fontsize=14, fontweight='bold')
plt.xlabel('Epoch', fontsize=12)
plt.ylabel('Loss (MSE)', fontsize=12)
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)

# 2. Plot Training History - MAE
plt.subplot(1, 2, 2)
plt.plot(history.history['mae'], label='Training MAE', linewidth=2)
plt.plot(history.history['val_mae'], label='Validation MAE', linewidth=2)
plt.title('Model Mean Absolute Error Over Epochs', fontsize=14, fontweight='bold')
plt.xlabel('Epoch', fontsize=12)
plt.ylabel('MAE', fontsize=12)
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(f'models/{model_name}_training_history.png', dpi=300, bbox_inches='tight')
print(f"✓ Training history plot saved to: models/{model_name}_training_history.png")
plt.show()

# 3. Final Training Metrics Summary
print("\n" + "-"*50)
print("FINAL TRAINING METRICS")
print("-"*50)
print(f"Final Training Loss:      {history.history['loss'][-1]:.6f}")
print(f"Final Validation Loss:    {history.history['val_loss'][-1]:.6f}")
print(f"Final Training MAE:       {history.history['mae'][-1]:.6f}")
print(f"Final Validation MAE:     {history.history['val_mae'][-1]:.6f}")

# 4. Make predictions on validation data
split_idx = int(len(X) * 0.8)
X_train, X_val = X[:split_idx], X[split_idx:]
y_train, y_val = y[:split_idx], y[split_idx:]

y_pred = model.predict(X_val)

# 5. Calculate per-parameter metrics
print("\n" + "-"*50)
print("PER-PARAMETER EVALUATION (on validation set)")
print("-"*50)
param_names = ['Kp_motor0', 'Ki_motor0', 'Kd_motor0',
               'Kp_motor1', 'Ki_motor1', 'Kd_motor1',
               'Kp_motor2', 'Ki_motor2', 'Kd_motor2']

for i, param_name in enumerate(param_names):
    mae = np.mean(np.abs(y_val[:, i] - y_pred[:, i]))
    mse = np.mean((y_val[:, i] - y_pred[:, i])**2)
    rmse = np.sqrt(mse)
    print(f"{param_name:12s} - MAE: {mae:.6f}, RMSE: {rmse:.6f}")

# 6. Plot Predictions vs Actual for each parameter
fig, axes = plt.subplots(3, 3, figsize=(15, 12))
fig.suptitle('Predicted vs Actual PID Gains (Validation Set)', fontsize=16, fontweight='bold')

for i, (ax, param_name) in enumerate(zip(axes.flat, param_names)):
    ax.scatter(y_val[:, i], y_pred[:, i], alpha=0.5, s=20)
    
    # Plot perfect prediction line
    min_val = min(y_val[:, i].min(), y_pred[:, i].min())
    max_val = max(y_val[:, i].max(), y_pred[:, i].max())
    ax.plot([min_val, max_val], [min_val, max_val], 'r--', linewidth=2, label='Perfect Prediction')
    
    ax.set_xlabel('Actual Value', fontsize=10)
    ax.set_ylabel('Predicted Value', fontsize=10)
    ax.set_title(param_name, fontsize=12, fontweight='bold')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(f'models/{model_name}_predictions_vs_actual.png', dpi=300, bbox_inches='tight')
print(f"\n✓ Predictions vs Actual plot saved to: models/{model_name}_predictions_vs_actual.png")
plt.show()

# 7. Plot Error Distribution
plt.figure(figsize=(15, 10))
for i, param_name in enumerate(param_names):
    plt.subplot(3, 3, i+1)
    errors = y_val[:, i] - y_pred[:, i]
    plt.hist(errors, bins=30, alpha=0.7, color='blue', edgecolor='black')
    plt.axvline(x=0, color='red', linestyle='--', linewidth=2)
    plt.xlabel('Prediction Error', fontsize=10)
    plt.ylabel('Frequency', fontsize=10)
    plt.title(f'{param_name} Error Distribution', fontsize=11, fontweight='bold')
    plt.grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig(f'models/{model_name}_error_distribution.png', dpi=300, bbox_inches='tight')
print(f"✓ Error distribution plot saved to: models/{model_name}_error_distribution.png")
plt.show()

# 8. Overall Statistics
print("\n" + "-"*50)
print("OVERALL STATISTICS")
print("-"*50)
overall_mae = np.mean(np.abs(y_val - y_pred))
overall_mse = np.mean((y_val - y_pred)**2)
overall_rmse = np.sqrt(overall_mse)
r2_score = 1 - (np.sum((y_val - y_pred)**2) / np.sum((y_val - np.mean(y_val))**2))

print(f"Overall MAE:              {overall_mae:.6f}")
print(f"Overall MSE:              {overall_mse:.6f}")
print(f"Overall RMSE:             {overall_rmse:.6f}")
print(f"R² Score:                 {r2_score:.6f}")

print("\n" + "="*50)
print("EVALUATION COMPLETE")
print("="*50)