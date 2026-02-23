import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from simulator import simulator
from wind_estimator import train_optimized, LSTM_wind_estimator
import time
import numpy as np
import matplotlib.pyplot as plt
import os

device = torch.device(
    "cuda" if torch.cuda.is_available()
    else "mps" if torch.backends.mps.is_available()
    else "cpu"
)
torch.cuda.empty_cache()  

wind_lstm = LSTM_wind_estimator().to(device=device)

NUM_EPOCH = 10
for epoch in np.arange(NUM_EPOCH):

    num_env = 5
    training_input = []
    expected_output = []

    # First epoch: use perfect wind knowledge (p_truth=1.0 means always use [0,0,0])
    # Subsequent epochs: use the LSTM estimator
    if epoch == 0:
        p_truth = 1.0
        estimator = None
    else:
        p_truth = 1.0 - (epoch / (0.5*NUM_EPOCH))
        estimator = wind_lstm

    for env in np.arange(num_env):
        print(f"Epoch {epoch}, Environment {env}/{num_env}")
        sim = simulator(p_truth=p_truth, wind_estimator=estimator)
        while sim.control_loop() is True:
            continue

        runtime = len(sim.observed_state_history)
        training_input.append(sim.wind_estimation_history)
        expected_output.append(np.array(sim.wind_history))

    model = train_optimized(training_input, expected_output, model=wind_lstm, device=device)
    wind_lstm = model

    ## Evaluation: run one simulation with the trained LSTM and plot results
    print(f"--- Evaluating LSTM after epoch {epoch} ---")
    eval_sim = simulator(p_truth=0.0, wind_estimator=wind_lstm)
    while eval_sim.control_loop() is True:
        continue

    eval_runtime = len(eval_sim.observed_state_history)
    eval_timesteps = (1.0 / 30.0) * np.arange(eval_runtime)
    wind_est = np.array(eval_sim.wind_estimation_history)
    wind_true = np.array(eval_sim.wind_history)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    labels = ["across", "along", "vertical"]
    for i in range(3):
        axes[i].plot(eval_timesteps, wind_true[:eval_runtime, i], label=f"true_{labels[i]}", linestyle='--')
        axes[i].plot(eval_timesteps, wind_est[:eval_runtime, i], label=f"est_{labels[i]}")
        axes[i].legend()
        axes[i].set_title(f"Wind {labels[i]} - Epoch {epoch}")
        axes[i].set_xlabel("Time (s)")
        axes[i].set_ylabel("Wind speed (m/s)")
        axes[i].grid(alpha=0.3)
    fig.suptitle(f"LSTM Wind Estimator Evaluation - Epoch {epoch}")
    plt.tight_layout()
    plt.savefig(f"./lstm_eval_epoch_{epoch}.png")
    plt.show()
    print(f"Saved evaluation plot: lstm_eval_epoch_{epoch}.png")

torch.save(wind_lstm.state_dict(), f"./{time.time()}_wind_estimator_pth")