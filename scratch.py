import serial
import matplotlib.pyplot as plt
import csv
import time
import struct  # Biblioteka do obsługi danych binarnych

# --- KONFIGURACJA ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
FILE_NAME = "pomiar_grzania_proba.csv"

timestamps = []
temperatures = []

with open(FILE_NAME, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Czas [s]", "Temperatura [C]"])

plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot([], [], 'r-', label='Odpowiedź skokowa obiektu')
ax.set_xlabel("Czas od startu [s]")
ax.set_ylabel("Temperatura [°C]")
ax.set_title("Charakterystyka nagrzewania rezystora")
ax.legend()
ax.grid(True)

start_time = time.time()

try:
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) as ser:
        print(f"Rozpoczęto rejestrację binarną do pliku {FILE_NAME}...")

        while True:
            # Czekamy, aż w buforze będą co najmniej 4 bajty (tyle zajmuje float)
            if ser.in_waiting >= 4:
                raw_bytes = ser.read(4)

                try:
                    # Dekodujemy 4 bajty na liczbę float
                    # '<f' oznacza: Little Endian (standard STM32), typ float
                    temp = struct.unpack('<f', raw_bytes)[0]

                    current_time = time.time() - start_time
                    timestamps.append(current_time)
                    temperatures.append(temp)

                    # Zapis do CSV
                    with open(FILE_NAME, mode='a', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow([round(current_time, 2), temp])

                    line.set_data(timestamps, temperatures)
                    ax.relim()
                    ax.autoscale_view()

                    plt.draw()
                    plt.pause(0.1)
                    fig.canvas.flush_events()

                except Exception as e:
                    print(f"Błąd dekodowania: {e}")

except KeyboardInterrupt:
    print(f"\nKoniec pomiaru. Dane zapisane.")
    plt.ioff()
    plt.show()