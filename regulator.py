# -*- coding: utf-8 -*-
"""
Created on Wed Jan 28 14:04:11 2026

@author: maksm
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 1. WCZYTANIE DANYCH
try:
    df = pd.read_csv("pomiar_grzania.csv")
    t_measured = df["Czas [s]"].values
    temp_measured = df["Temperatura [C]"].values
except FileNotFoundError:
    print("Błąd: Nie znaleziono pliku pomiar_grzania.csv!")
    exit()

# 2. GENEROWANIE ODPOWIEDZI MODELOWEJ (TEORETYCZNEJ)
# Założenia: start 22°C, cel 60°C, stała czasowa tau=30s
T_env = temp_measured[0]
T_target = 47.0
tau = 315.0
temp_model = T_env + (T_target - T_env) * (1 - np.exp(-t_measured / tau))

# 3. OBLICZENIE BŁĘDU (Różnica między pomiarem a modelem)
error = temp_model - temp_measured

# 4. REGULATOR PID (Symulacja)
# Parametry, które będziesz musiał dobrać (tuning):
Kp = 2.0  # Wzmocnienie proporcjonalne
Ki = 0.5  # Wzmocnienie całkujące
Kd = 0.1  # Wzmocnienie różniczkujące

pid_output = []
integral = 0
last_error = 0
dt = np.mean(np.diff(t_measured)) if len(t_measured) > 1 else 0.1

for e in error:
    integral += e * dt
    derivative = (e - last_error) / dt
    output = Kp * e + Ki * integral + Kd * derivative
    pid_output.append(output)
    last_error = e

# --- WYKRESY ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Wykres porównawczy
ax1.plot(t_measured, temp_measured, 'r.', label="Pomiar")
ax1.plot(t_measured, temp_model, 'b--', label="Model")
ax1.set_ylabel("Temperatura [°C]")
ax1.set_title("Porównanie: Pomiar vs Model")
ax1.legend()
ax1.grid(True)

# Wykres błędu i sygnału PID
ax2.plot(t_measured, error, 'g', label="Błąd (Model - Pomiar)")
#ax2.plot(t_measured, pid_output, 'm', alpha=0.5, label="Sygnał sterujący PID [0-100%]")
ax2.set_xlabel("Czas [s]")
ax2.set_ylabel("Temperatura [°C]")
#ax2.set_title("Błąd i wyjście regulatora")
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()