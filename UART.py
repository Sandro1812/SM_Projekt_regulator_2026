import serial
import struct
import sys

# --- KONFIGURACJA ---
PORT = 'COM3'
BAUD_RATE = 115200


def main():
    try:
        # otwarcie portu UART
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print(f"Połączono z {PORT}. Wpisz wartość setpoint (np. 36.5) i naciśnij Enter.")
        print("Wpisz 'exit', aby zakończyć.")

        while True:
            # pobieranie danych z terminala
            user_input = input("Podaj setpoint > ")

            if user_input.lower() == 'exit':
                break

            try:
                # konwersja tekstu na liczbę zmiennoprzecinkową
                value = float(user_input)

                # pakowanie floata do 4 bajtów
                data = struct.pack('<f', value)

                # wysyłka surowych bajtów
                ser.write(data)

                print(f"Wysłano: {value} | Bajty (hex): {data.hex().upper()}")

            except ValueError:
                print("Błąd: To nie jest poprawna liczba!")

        ser.close()

    except serial.SerialException as e:
        print(f"Błąd portu szeregowego: {e}")
    except KeyboardInterrupt:
        print("\nPrzerwano pracę.")


if __name__ == "__main__":
    main()