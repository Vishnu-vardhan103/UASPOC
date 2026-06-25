import numpy as np
import matplotlib.pyplot as plt
import random
import os

# 1. Create a special folder on your desktop so it doesn't get messy
os.makedirs("drone_dataset", exist_ok=True)
print("Starting the Dataset Factory...")

# 2. THE LOOP: Generate 5 different drone signals automatically
for i in range(5):
    # Pick a random frequency between 40Hz and 80Hz
    random_freq = random.randint(40, 80)
    
    time = np.linspace(0, 0.5, 500)
    drone_signal = np.sin(2 * np.pi * random_freq * time) 
    noise = np.random.normal(0, 0.5, 500)
    messy_signal = drone_signal + noise

    # Generate and save the spectrogram
    plt.clf() 
    plt.specgram(messy_signal, Fs=1000, cmap='inferno')
    plt.title(f"Drone at {random_freq}Hz")
    
    filename = f"drone_dataset/spectrogram_drone_{random_freq}Hz.png"
    plt.savefig(filename)
    print(f"Created: {filename}")

# 3. Create one "Empty Sky" (Just noise, NO drone)
plt.clf()
pure_noise = np.random.normal(0, 0.5, 500)
plt.specgram(pure_noise, Fs=1000, cmap='inferno')
plt.title("Empty Sky (Pure Noise)")
plt.savefig("drone_dataset/spectrogram_empty_sky.png")
print("Created: drone_dataset/spectrogram_empty_sky.png")

print("Factory finished! Check your new folder.")