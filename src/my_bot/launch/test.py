import serial
import time

PORT = '/dev/ttyACM0'
BAUD = 115200
MOTOR_SPEEDS = [-800, -800, 0, 800, 800, 800]

joint_names = [
    'front_left_leg_joint',
    'centre_left_leg_joint',
    'back_left_leg_joint',
    'front_right_leg_joint',
    'centre_right_leg_joint',
    'back_right_leg_joint'
]

ser = serial.Serial(PORT, BAUD)
time.sleep(2)  # wait for Arduino

# Send motor speed command (CSV)
command = ','.join(str(int(s)) for s in MOTOR_SPEEDS) + '\n'
ser.write(command.encode())
print(f"Command sent: {command.strip()}")

print("Reading encoder values...")
try:
    while True:
        line = ser.readline().decode().strip()
        if line.startswith("L1:"):
            parts = line.split(",")
            angles = []
            for part in parts:
                if ":" in part:
                    _, val = part.split(":")
                    angles.append(float(val))

            if len(angles) == 6:
                joint_angles = dict(zip(joint_names, angles))
                print("Joint Angles:", joint_angles)
            else:
                print("Incomplete data:", parts)
        else:
            print("Unrecognized line:", line)
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    ser.close()
