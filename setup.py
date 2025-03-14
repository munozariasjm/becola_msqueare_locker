from cx_Freeze import setup, Executable
import sys

# Dependencies
build_exe_options = {
    "packages": ["sys", "time", "threading", "PyQt5", "epics"],
    "includes": ["config"],
    "excludes": ["tkinter"],
    "include_files": ["./src/config.py", "./src/_all.py"],
}

# Base configuration for Windows (use 'None' for console apps)
base = None
if sys.platform == "win32":
    base = "Win32GUI"  # Use this for GUI applications (no terminal window)

# Executable configuration
executables = [
    Executable("src/_all.py", base=base, target_name="LaserControl.exe")
]

# Setup
setup(
    name="Laser Control GUI",
    version="1.0",
    description="Minimal GUI for laser etalon control",
    options={"build_exe": build_exe_options},
    executables=executables
)

#Run python setup.py build
