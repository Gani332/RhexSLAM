import serial
import time

# --- CONFIG ---
PORT = '/dev/ttyACM0'
BAUD = 115200
MOTOR_SPEEDS = [0, 0, 0, 0, 0, 0]  # example speeds

# --- SETUP ---
ser = serial.Serial(PORT, BAUD)
time.sleep(2)  # Allow time for Arduino to reset

# --- SEND SPEED COMMAND ---
command = ','.join(str(int(s)) for s in MOTOR_SPEEDS) + '\n'
ser.write(command.encode())
print(f"Command sent: {command.strip()}")

# --- READ ENCODER VALUES ---
print("Reading encoder values:")
try:
    while True:
        line = ser.readline().decode().strip()
        if line.startswith("L1:"):
            # Example: L1:0.0000,L2:0.0038,L3:0.0000,R1:0.0000,R2:0.0000,R3:0.0000
            parts = line.split(",")
            angles = []
            for part in parts:
                if ":" in part:
                    _, val = part.split(":")
                    angles.append(float(val))
            print("Angles (rad):", angles)
        else:
            print("Unrecognized line:", line)
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    ser.close()
