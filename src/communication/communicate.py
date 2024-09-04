import math
import serial
import time
import torch

def angle_rad_to_pwm(angle_rad, mirrored=False):
    """
    Convert an angle in radians (from -1.5 to 1.5 radians) to PWM pulse width for the MG90S servo, with optional mirroring.
    This version maps -1.5 radians to 1000 µs (1.0 ms) and 1.5 radians to 2000 µs (2.0 ms).
    
    Parameters:
        angle_rad (float): Angle in radians (from -1.5 to 1.5 radians).
        mirrored (bool): If True, mirror the angle mapping (0 radians becomes ±1.5 radians, etc.).

    Returns:
        float: Pulse width in milliseconds (ms) corresponding to the angle.
    """
    # Ensure angle is within valid range
    if angle_rad < -1.5 or angle_rad > 1.5:
        raise ValueError("Angle must be between -1.5 and 1.5 radians")

    # If mirrored, reverse the angle mapping
    if mirrored:
        angle_rad = -angle_rad

    # Define pulse width range
    min_pulse_width = 1.0  # Pulse width in ms for -1.5 radians
    max_pulse_width = 2.0  # Pulse width in ms for 1.5 radians
    
    # Calculate pulse width based on angle
    # Linear interpolation between min_pulse_width and max_pulse_width
    pulse_width = min_pulse_width + ((max_pulse_width - min_pulse_width) * (angle_rad + 1.5) / 3.0)

    return pulse_width


model = torch.load('model.pth')

# Set the model to evaluation mode
model.eval()

# ssc32 = serial.Serial('/dev/ttyUSB0, 9600, timeout=1.0')
# ssc32 = serial.Serial(
#   port='COM6',
#   baudrate=9600,
#   parity=serial.PARITY_ODD,
#   stopbits=serial.STOPBITS_ONE,
#   bytesize=serial.EIGHTBITS
# )

# print("STARTING")
# ssc32.write("#1 P1500 <cr>")
# time.sleep(10)
# print("END")
# ssc32.close()

print(angle_rad_to_pwm((-math.pi / 2)+ 0.1))