import numpy as np
import matplotlib.pyplot as plt

# PARAMETRY MODELU
T_env = 24.85
T_max_power = 47.0
tau = 315.0
K = T_max_power - T_env

# USTAWIENIA SYMULACJI
dt = 1.0
t_sim = 1200
setpoint = 38.0

# NASTAWY PID
Kp = 5
Ki = 0.1
Kd = 2

# Inicjalizacja
t_axis = np.arange(0, t_sim, dt)
temp = np.zeros_like(t_axis)
duty_cycle_out = np.zeros_like(t_axis)
temp[0] = T_env

integral = 0
last_error = 0

# PĘTLA SYMULACJI
for i in range(1, len(t_axis)):
    error = setpoint - temp[i - 1]

    integral += error * dt

    derivative = (error - last_error) / dt

    # OBLICZENIE WYPEŁNIENIA PWM (0.0 do 1.0)
    u = (Kp * error + Ki * integral + Kd * derivative) / 100.0
    u = np.clip(u, 0, 1)

    duty_cycle_out[i] = u
    last_error = error

    # Wentylator reaguje na WYPEŁNIENIE PWM
    dT_dt = (K * u - (temp[i - 1] - T_env)) / tau
    temp[i] = temp[i - 1] + dT_dt * dt

# --- WYKRESY ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Wykres Temperatury
ax1.plot(t_axis, [setpoint] * len(t_axis), 'r--', alpha=0.6, label="Wartość zadana")
ax1.plot(t_axis, temp, 'b', linewidth=2, label="Temperatura rzeczywista")
ax1.set_ylabel("Temperatura [°C]")
ax1.set_title("Regulacja wypełnieniem PWM przez PID")
ax1.legend();
ax1.grid(True)

# Wykres Wypełnienia PWM (Duty Cycle)
ax2.fill_between(t_axis, duty_cycle_out * 100, color='m', alpha=0.3, label="Obszar wypełnienia PWM")
ax2.plot(t_axis, duty_cycle_out * 100, 'm', linewidth=1.5, label="Wypełnienie PWM [%]")
ax2.set_ylabel("Duty Cycle [%]")
ax2.set_xlabel("Czas [s]")
ax2.set_ylim(-5, 105)
ax2.legend();
ax2.grid(True)

plt.tight_layout()
plt.show()