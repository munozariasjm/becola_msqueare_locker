# Emergency M2 controller

This is a minimalistic laser wavelength locking system using **PyQt5** for the GUI, **EPICS** for reading the wavenumber, and a **PID controller** to stabilize the wavelength by tuning the etalon.

This software is designed for **physicists** who need a **simple yet effective laser stabilization** tool with minimal setup.

## Features
-  **Simple PyQt5 GUI** with:
     - Lock/Unlock Etalon buttons
     - Target Wavenumber input
     - Enable/Disable PID buttons
-  **PID Controller** for automatic stabilization
-  **Reads live wavenumber from an EPICS PV**
-  **Configuration in `config.py`** for easy customization
-  **Minimal background threads for efficient operation**

---

## Installation

### **1. Install Dependencies**
Ensure you have Python installed (preferably **Python 3.7+**). Install the required libraries:

```sh
pip install pyqt5 pylablib pyepics
```

## Usage

Edit config.py to set your laser and EPICS parameters.
Run the software:

```
python laser_lock_gui.py
```


## Troubleshooting
**GUI doesn't start?**


Ensure Python 3 and dependencies are installed.
Try running with python3 laser_lock_gui.py.

**Wavenumber not updating?**

Verify WAVENUMBER_PV in config.py.
Ensure EPICS is running and accessible.

**PID not working?**

Adjust KP, KI, KD in config.py.
Restart the software after changes.

**Etalon lock issues?**

Check IP and PORT in config.py.

## Support
For issues or feature requests please get in touch. I will answer ASAP.