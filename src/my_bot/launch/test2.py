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
        time.sleep(2)  # Allow time for Arduino to reset

        # 1. Set motor speeds
        send_command(ser, "SET_SPEEDS 100,200,300,-100,-200,-300")

        # 2. Set braking (1 = brake, 0 = coast)
        send_command(ser, "SET_BRAKE 1,1,1,0,0,0")

        # 3. Set acceleration (units/s^2)
        send_command(ser, "SET_ACCEL 2000")

        # 4. Set deceleration (units/s^2)
        send_command(ser, "SET_DECEL 2000")

        # 5. Set current limit (milliamps)
        send_command(ser, "SET_CURRENT_LIMIT 3000")

        # 6. Get current limits
        send_command(ser, "GET_CURRENT_LIMIT")

        # 7. Wait and print a few encoder lines
        print("\n--- Reading encoder outputs for 2 seconds ---")
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
