import serial
import csv
import time

# Serial port configuration
PORT = '/dev/ttyACM1'
BAUDRATE = 115200
TIMEOUT = 2

# Output CSV file
CSV_FILENAME = 'motor_thrust_data.csv'

def main():
    try:
        # Open serial connection
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        time.sleep(2)  # Give time for Arduino to reset

        # Send start command
        ser.write(b'start\n')
        print("Sent 'start' to Arduino. Logging data...")

        with open(CSV_FILENAME, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['speed', 'thrust'])  # CSV header

            while True:
                try:
                    line = ser.readline().decode().strip()
                    if not line:
                        continue
                    
                    if '|' in line:
                        speed_str, thrust_str = line.split('|')
                        speed = int(speed_str)
                        thrust = float(thrust_str)
                        writer.writerow([speed/100, thrust])
                        print(f"{speed/100} | {thrust:.5f}")
                    else:
                        print(f"Ignored malformed line: {line}")
                
                except KeyboardInterrupt:
                    print("\nLogging stopped by user.")
                    break
                except Exception as e:
                    print(f"Error parsing line: {e}")
    
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print(f"Data saved to {CSV_FILENAME}")

if __name__ == "__main__":
    main()
