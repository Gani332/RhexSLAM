import serial
import time

# --- CONFIG ---
PORT = '/dev/ttyACM0'
BAUD = 115200

def send_command(ser, cmd, wait_response=True):
    ser.write((cmd + '\n').encode())
    time.sleep(0.05)
    if wait_response:
        timeout = time.time() + 0.5
        while ser.in_waiting or time.time() < timeout:
            line = ser.readline().decode().strip()
            if line:
                print(f"<<< {line}")

def wait_for_encoder_data(ser):
    print("Waiting for encoder output from Arduino...")
    while True:
        line = ser.readline().decode().strip()
        if line.startswith("L1:"):
            print(f"<<< {line}")
            break

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # Let Arduino reset

        wait_for_encoder_data(ser)

        # Configure Motoron
        send_command(ser, "SET_BRAKE 0,0,1,0,0,0")  # Motor 3 is braked
        send_command(ser, "SET_ACCEL 2000")
        send_command(ser, "SET_DECEL 2000")
#        send_command(ser, "SET_CURRENT_LIMIT 3000")
        send_command(ser, "GET_CURRENT_LIMIT")

        print("\n--- Driving motors for 3 seconds (M3 braked) ---")
        start = time.time()
        last_encoder_print = 0
        last_current_check = 0

        while time.time() - start < 3:
            # Send speed command (motor 3 is ignored due to brake)
            send_command(ser, "SET_SPEEDS 100,100,100,100,100,100", wait_response=False)

            now = time.time()

            # Print encoder data every 0.2s
            if now - last_encoder_print > 0.2:
                line = ser.readline().decode().strip()
                if line.startswith("L1:") or line.startswith("R1:"):
                    print(f"ENC: {line}")
                last_encoder_print = now

            # Read current limits every 1s
            if now - last_current_check > 1.0:
                send_command(ser, "GET_CURRENT_LIMIT")
                last_current_check = now

            time.sleep(0.05)

        print("\n--- Stopping motors ---")
        send_command(ser, "SET_SPEEDS 0,0,0,0,0,0")

        print("\n--- Final encoder readings ---")
        end_time = time.time() + 2
        while time.time() < end_time:
            line = ser.readline().decode().strip()
            if line.startswith("L1:") or line.startswith("R1:"):
                print(f"ENC: {line}")
            time.sleep(0.05)

    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
