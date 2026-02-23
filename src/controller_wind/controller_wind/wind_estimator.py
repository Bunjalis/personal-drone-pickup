import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from wind import Wind
from tqdm import tqdm
import matplotlib.pyplot as plt
import statistics
import numpy as np
from torch.utils.data import TensorDataset, DataLoader
import torch.multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import time
from torch.nn.utils.rnn import pad_packed_sequence
import os
import math
import gc
 
'''
Optimized Hyperparameter settings for better GPU utilization
'''
WINDOW_SIZE = 180
INPUT_SIZE = 3
HIDDEN_DIM = 32
LR = 0.0001
BATCH_SIZE = 512  # Increased for better GPU utilization
NUM_WORKERS = 32  # For DataLoader parallelization
PREFETCH_FACTOR = 8  # For DataLoader optimization
HANDOFF_ITERATIONS = 30
 
# Device configuration
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Using device: {device}")
if device.type == 'cuda':
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
 
class LSTM_wind_estimator(nn.Module):
    def __init__(self, hidden_dim=HIDDEN_DIM, input_size=INPUT_SIZE, num_layers=7, dropout=0.1):
        super(LSTM_wind_estimator, self).__init__()
        self.hidden_dim = hidden_dim
        self.input_size = input_size
        self.num_layers = num_layers
        
        # Enhanced LSTM with multiple layers and dropout
        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0
        )
        self.dropout = nn.Dropout(dropout)
        self.linear = nn.Linear(in_features=hidden_dim, out_features=hidden_dim)
        self.linear2 = nn.Linear(in_features=hidden_dim, out_features=3)
        self.relu = nn.ReLU()
 
    def forward(self, window):
        # window shape: (batch_size, sequence_length, input_size)
        lstm_out, _ = self.lstm(window)
        # Apply dropout before final layer
        lstm_out = self.dropout(lstm_out)
        # Only take the last timestep output
        lin_out = self.linear(lstm_out[:, -1, :])
        lin_out = self.relu(lin_out)
        lin_out = self.dropout(lin_out)
        
        lin_out = self.linear(lin_out)
        lin_out = self.relu(lin_out)
        lin_out = self.dropout(lin_out)
        
        lin_out = self.linear(lin_out)
        lin_out = self.relu(lin_out)
        lin_out = self.dropout(lin_out)
        
        velocity_space = self.linear2(lin_out)
        return velocity_space
 
def generate_training_batch(wind_displacements, wind, device):
    
    all_window = []
    all_output = []
    for i, wind_displacement in enumerate(wind_displacements):
        wind_displacement = np.array(wind_displacement)
        windows = np.lib.stride_tricks.sliding_window_view(wind_displacement, window_shape=(100, wind_displacement.shape[1]))
        windows = windows.squeeze(axis=1)  # (num_windows, 100, 3)
        num_windows = windows.shape[0]
        targets = np.array(wind[i])[99:99+num_windows, :]  # (num_windows, 3)
 
        all_window.append(windows)
        all_output.append(targets)
 
 
    all_window = torch.tensor(np.concatenate(all_window, axis=0), device=device, dtype=torch.float32)
    all_output = torch.tensor(np.concatenate(all_output, axis=0), device=device, dtype=torch.float32)
 
    return all_window, all_output
 
def train_optimized(wind_displacements, wind, model, device, num_epoch=150):
    """
    Optimized training function with vectorization and parallelization
    """
    loss_function = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=LR, weight_decay=1e-5)  # Adam often works better than SGD
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'min', patience=10, factor=0.5)
    
    history = []
    best_loss = float('inf')
 
    # Enable mixed precision training for faster computation on modern GPUs
    # scaler = torch.amp.GradScaler('cuda') if device.type == 'cuda' else None
    scaler = torch.amp.GradScaler(device=device)
    
    print("Starting wind_estimator training...")
    start_time = time.time()
    
    # Generate batch of episodes - use fewer episodes but larger batches for efficiency
    batch_windows, batch_targets = generate_training_batch(wind_displacements, wind, device=device)
    dataset = TensorDataset(batch_windows, batch_targets)
    loader = DataLoader(dataset, batch_size=32, shuffle=True)
    
    patience = 15
    for epoch in tqdm(range(num_epoch), desc="Training Progress"):
        model.train()
        epoch_start = time.time()
        epoch_loss = 0
        num_batches = 0
        
        N = batch_windows.size(0)
        indices = torch.randperm(N, device=device)
 
        for batch_x, batch_y in loader:
            optimizer.zero_grad()
            
            if scaler is not None:
                # Mixed precision training
                with torch.amp.autocast('cuda'):
                    output = model(batch_x)
                    loss = loss_function(output.squeeze(), batch_y)
                
                scaler.scale(loss).backward()
                scaler.step(optimizer)
                scaler.update()
            else:
                # Standard training
                output = model(batch_x)
                loss = loss_function(output.squeeze(), batch_y)
                loss.backward()
                optimizer.step()
            
            epoch_loss += loss.item()
            num_batches += 1
        
        avg_epoch_loss = (epoch_loss / max(num_batches, 1))
        history.append(avg_epoch_loss)
        
        # Learning rate scheduling
        scheduler.step(avg_epoch_loss)
        
        # Save best model
        if avg_epoch_loss < best_loss:
            best_loss = avg_epoch_loss
            torch.save(model.state_dict(), './wind_estimator_best.mdl')
        
        epoch_time = time.time() - epoch_start
        # if epoch % 10 == 0:
        #     print(f"Epoch {epoch}: Loss = {avg_epoch_loss:.6f}, Time = {epoch_time:.2f}s")
    
    total_time = time.time() - start_time
    print(f"Training completed in {total_time:.2f} seconds")
    
    # Plot training history
    plt.figure(figsize=(10, 6))
    plt.plot(range(len(history)), history)
    plt.title("Training Loss Over Epochs (Optimized)")
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.yscale('log')
    plt.grid(True)
    i= 0
    while True:
        if os.path.exists(f"./lstm_training_{i}.png"):
            i += 1
        else:
            plt.savefig(f"./lstm_training_{i}.png")
            break
    plt.close()
    
    del batch_windows
    del batch_targets
    del optimizer
    del scheduler
    del scaler
 
    gc.collect()
    return model
 
if __name__ == "__main__":
    print("=== Optimized Wind Estimator ===")