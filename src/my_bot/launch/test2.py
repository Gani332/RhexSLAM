import serial
import time

# CHANGE THIS to match your Arduino Giga's port
PORT = '/dev/ttyACM0'  # Example for Linux/Mac. On Windows: 'COM3'
BAUD = 115200

def send_command(ser, cmd):
    print(f">>> Sending: {cmd}")
    ser.write((cmd + '\n').encode())
    time.sleep(0.1)
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        if line:
            print(f"<<< {line}")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # Allow Arduino to reset

        print("Waiting for encoder startup...")
        while True:
            line = ser.readline().decode().strip()
            if line.startswith("L"):
                print(f"<<< {line}")
                break

        # Configure Motoron
        send_command(ser, "SET_BRAKE 0,0,0,0,0,0")
        send_command(ser, "SET_ACCEL 2000")
        send_command(ser, "SET_DECEL 2000")
        send_command(ser, "SET_CURRENT_LIMIT 3000")
        send_command(ser, "GET_CURRENT_LIMIT")

        print("\n--- Driving motors for 3 seconds ---")
        start = time.time()
        while time.time() - start < 3:
            send_command(ser, "SET_SPEEDS 100,200,300,-100,-200,-300")
            time.sleep(0.1)

        print("\n--- Done. Watching encoder output for 2 more seconds ---")
        start = time.time()
        while time.time() - start < 2:
            line = ser.readline().decode().strip()
            if line.startswith("L") or line.startswith("R"):
                print(f"<<< {line}")

    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
