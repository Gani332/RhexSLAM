import serial
import time

# --- CONFIG ---
PORT = '/dev/ttyACM0'  # Update if needed
BAUD = 115200

def send_command(ser, cmd, wait_response=True):
    print(f">>> Sending: {cmd}")
    ser.write((cmd + '\n').encode())
    time.sleep(0.1)
    if wait_response:
        timeout = time.time() + 1.0  # wait up to 1 sec for response
        while time.time() < timeout:
            line = ser.readline().decode().strip()
            if line:
                print(f"<<< {line}")
            elif ser.in_waiting == 0:
                break

def wait_for_encoder_data(ser):
    print("Waiting for encoder output from Arduino...")
    while True:
        line = ser.readline().decode().strip()
        if line.startswith("L1:"):
            print(f"<<< {line}")
            break

def read_encoder_output(ser, duration_sec=2):
    print(f"\n--- Reading encoder output for {duration_sec} seconds ---")
    start = time.time()
    while time.time() - start < duration_sec:
        line = ser.readline().decode().strip()
        if line.startswith("L1:"):
            parts = line.split(",")
            angles = []
            for part in parts:
                if ":" in part:
                    _, val = part.split(":")
                    angles.append(float(val))
            print("Angles (rad):", angles)
        elif line:
            print(f"<<< {line}")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # Allow Arduino to reset

        wait_for_encoder_data(ser)

        # Configure Motoron
        send_command(ser, "SET_BRAKE 0,0,0,0,0,0")
        send_command(ser, "SET_ACCEL 2000")
        send_command(ser, "SET_DECEL 2000")
        send_command(ser, "SET_CURRENT_LIMIT 3000")
        send_command(ser, "GET_CURRENT_LIMIT")

        print("\n--- Driving motors for 3 seconds ---")
        start = time.time()
        while time.time() - start < 3:
            send_command(ser, "SET_SPEEDS 100,200,300,-100,-200,-300", wait_response=False)
            time.sleep(0.1)

        print("\n--- Stopping motors ---")
        send_command(ser, "SET_SPEEDS 0,0,0,0,0,0")

        read_encoder_output(ser, duration_sec=2)

    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
