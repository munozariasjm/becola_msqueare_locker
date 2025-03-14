# config.py
# Basic configuration for the Laser Locking system

# Laser connection parameters
IP_ADDRESS = "192.168.1.222"
# PORT = 39933
PORT = 8888

# EPICS Process Variable (PV) names
WAVENUMBER_PV = "LaserLab:wavenumber_1"
DEBUG_MODE = True  # Set to False for production

# PID controller parameters
KP = 40.0
KI = 0.8
KD = 0.0

# Reading frequency in seconds
READ_FREQ = 0.1
