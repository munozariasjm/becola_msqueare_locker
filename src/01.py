import sys
import time
import threading

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QMessageBox, QDialog
)
from PyQt5.QtCore import Qt

from epics import PV
from pylablib.devices import M2

# Laser connection parameters
IP_ADDRESS = "192.168.1.222"
PORT = 39933

# EPICS Process Variable (PV) names
WAVENUMBER_PV = "DBEL_RISE:MTER_N0002:FR_RD"
DEBUG_MODE = True  # Set to False for production

# PID controller parameters
KP = 40.0
KI = 0.8
KD = 0.0

# Reading frequency in seconds
READ_FREQ = 0.1


###############################################################################
#                  Minimal Classes for Reading, PID, and Laser Control        #
###############################################################################

class EMAServerReader:
    """
    Minimal reader for the frequency (in THz) from an EPICS PV in a background thread.
    """
    def __init__(self, pv_name, read_freq=READ_FREQ):
        """
        Args:
            pv_name (str): The EPICS PV name that provides the frequency.
            read_freq (float): Frequency (seconds) between successive reads.
        """
        self._pv = PV(pv_name)
        self._read_freq = read_freq
        self._thread = None
        self._running = False
        self._current_value = 0.0

    def start(self):
        """Start continuously reading the frequency in a background thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._reading_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the background reading thread."""
        self._running = False
        if self._thread:
            self._thread.join()

    def _reading_loop(self):
        while self._running:
            val = self._pv.get()
            if val is not None:
                try:
                    self._current_value = round(float(val), 5)
                except ValueError:
                    self._current_value = 0.0
            time.sleep(self._read_freq)

    def get_current_value(self):
        """Return the latest frequency value read from EPICS."""
        return self._current_value


class PIDController:
    """
    A simple PID controller.
    """
    def __init__(self, kp, ki, kd, setpoint=0.0):
        """
        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            setpoint (float): Desired target frequency.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.time()
        self._first_update = True

    def update(self, current_value):
        """
        Compute the correction based on the difference between setpoint and current_value.
        """
        now = time.time()
        error = self.setpoint - current_value
        dt = now - self._prev_time if not self._first_update else 0.0
        if dt <= 0:
            dt = 1e-6

        self._integral += error * dt
        derivative = 0.0 if self._first_update else (error - self._prev_error) / dt

        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)

        self._prev_time = now
        self._prev_error = error
        self._first_update = False

        return output

    def set_setpoint(self, new_setpoint):
        self.setpoint = new_setpoint
        self.reset()

    def reset(self):
        """Reset PID internal states."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.time()
        self._first_update = True


class MinimalLaserController:
    """
    A minimal controller to show:
      1) Reading the frequency from an EPICS PV,
      2) Locking/Unlocking the etalon, and
      3) A PID control loop for stabilizing the frequency.
    """
    def __init__(self, ip_address, port, pv_name, kp, ki, kd):
        """
        Args:
            ip_address (str): IP address for the M2 SolsTiS Laser.
            port (int): Port for the M2 SolsTiS Laser.
            pv_name (str): EPICS PV name for the frequency.
            kp, ki, kd (float): PID gains.
        """
        # Connect to the M2 laser
        self.laser = M2.Solstis(ip_address, port)

        # Use a single etalon lock (initially unlocked)
        self.etalon_lock_status = "off"

        # Create the reader for the frequency from the EPICS PV
        self.reader = EMAServerReader(pv_name, read_freq=READ_FREQ)

        # Create the PID controller for controlling the frequency
        self.pid = PIDController(kp, ki, kd, setpoint=0.0)

        self._control_thread = None
        self._running = False
        self._pid_enabled = False

    def start(self):
        """Start reading and the PID control loop."""
        self.reader.start()
        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

    def stop(self):
        """Stop background threads."""
        self._running = False
        self.reader.stop()
        if self._control_thread:
            self._control_thread.join()

    # --- Laser / Etalon Lock Methods ---
    def lock_etalon(self):
        """Lock the etalon and update status."""
        self.laser.lock_etalon()
        self.etalon_lock_status = "on"

    def unlock_etalon(self):
        """Unlock the etalon and update status."""
        self.laser.unlock_etalon()
        self.etalon_lock_status = "off"

    def get_etalon_lock_status(self):
        return self.etalon_lock_status

    # --- Laser Tuning Method ---
    def tune_etalon(self, new_value):
        """Tune the etalon to a new value."""
        self.laser.tune_etalon(new_value, sync=True)

    # --- PID Control Methods ---
    def set_target_frequency(self, freq):
        """Set a new target frequency for the PID controller."""
        self.pid.set_setpoint(freq)

    def enable_pid(self):
        """Enable the PID control loop."""
        self._pid_enabled = True
        self.pid.reset()

    def disable_pid(self):
        """Disable the PID control loop."""
        self._pid_enabled = False

    def update_pid_parameters(self, kp, ki, kd):
        """Update the PID parameters."""
        self.pid.kp = kp
        self.pid.ki = ki
        self.pid.kd = kd
        self.pid.reset()

    def get_current_tuner_value(self):
        """
        Retrieve the current etalon tuning value from the laser.
        Returns:
            float or None: Current tuner value, or None if not available.
        """
        try:
            return float(self.laser.get_full_web_status()["etalon_tune"])
        except Exception:
            return None

    def _control_loop(self):
        """
        Background loop for applying PID corrections to the etalon
        when PID is enabled and the etalon is locked.
        """
        while self._running:
            if self._pid_enabled and (self.etalon_lock_status == "on"):
                current_value = self.reader.get_current_value()
                correction = self.pid.update(current_value)

                # Retrieve current etalon tuner value and apply the correction
                current_etalon = self.get_current_tuner_value()
                if current_etalon is not None:
                    # Scale the correction as needed (here, multiplying by 0.1)
                    new_val = current_etalon - correction * 0.1
                    self.tune_etalon(new_val)
            time.sleep(0.05)


###############################################################################
#                          PID Parameter Dialog                               #
###############################################################################

class PIDParameterDialog(QDialog):
    """
    A dialog to update the PID parameters.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Update PID Parameters")
        layout = QVBoxLayout(self)

        # Input fields for kp, ki, kd
        self.kp_input = QLineEdit(self)
        self.ki_input = QLineEdit(self)
        self.kd_input = QLineEdit(self)

        layout.addWidget(QLabel("Proportional Gain (kp):", self))
        layout.addWidget(self.kp_input)
        layout.addWidget(QLabel("Integral Gain (ki):", self))
        layout.addWidget(self.ki_input)
        layout.addWidget(QLabel("Derivative Gain (kd):", self))
        layout.addWidget(self.kd_input)

        # Buttons for update and cancel
        button_layout = QHBoxLayout()
        self.ok_button = QPushButton("Update", self)
        self.cancel_button = QPushButton("Cancel", self)
        button_layout.addWidget(self.ok_button)
        button_layout.addWidget(self.cancel_button)
        layout.addLayout(button_layout)

        self.ok_button.clicked.connect(self.accept)
        self.cancel_button.clicked.connect(self.reject)

    def get_parameters(self):
        """
        Returns:
            tuple: (kp, ki, kd) as floats if valid, otherwise None.
        """
        try:
            kp = float(self.kp_input.text())
            ki = float(self.ki_input.text())
            kd = float(self.kd_input.text())
            return kp, ki, kd
        except ValueError:
            return None


###############################################################################
#                          Minimal PyQt5 GUI                                  #
###############################################################################

class MinimalGUI(QMainWindow):
    """
    A simple PyQt5 GUI for locking the etalon, updating PID parameters,
    and stabilizing the frequency.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minimal Laser/Etalon Lock GUI")
        self.setGeometry(100, 100, 400, 350)

        # Central widget and layout
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        vbox = QVBoxLayout(central_widget)

        # Status label for etalon lock
        self.status_label = QLabel("Etalon Lock Status: off", self)
        vbox.addWidget(self.status_label, alignment=Qt.AlignCenter)

        # Label to display current etalon tuning value
        self.tune_label = QLabel("Current Etalon Tune Value: N/A", self)
        vbox.addWidget(self.tune_label, alignment=Qt.AlignCenter)

        # Target frequency input (in THz)
        hbox_target = QHBoxLayout()
        hbox_target.addWidget(QLabel("Target Frequency (THz):", self))
        self.target_input = QLineEdit(self)
        hbox_target.addWidget(self.target_input)
        vbox.addLayout(hbox_target)

        # Buttons for lock/unlock, PID control, and updating PID parameters
        hbox_buttons = QHBoxLayout()
        self.btn_lock = QPushButton("Lock Etalon", self)
        self.btn_unlock = QPushButton("Unlock Etalon", self)
        hbox_buttons.addWidget(self.btn_lock)
        hbox_buttons.addWidget(self.btn_unlock)
        vbox.addLayout(hbox_buttons)

        hbox_pid = QHBoxLayout()
        self.btn_enable_pid = QPushButton("Enable PID", self)
        self.btn_disable_pid = QPushButton("Disable PID", self)
        hbox_pid.addWidget(self.btn_enable_pid)
        hbox_pid.addWidget(self.btn_disable_pid)
        vbox.addLayout(hbox_pid)

        # Button to update PID parameters
        self.btn_update_pid = QPushButton("Update PID Params", self)
        vbox.addWidget(self.btn_update_pid, alignment=Qt.AlignCenter)

        # Connect buttons to event handlers
        self.btn_lock.clicked.connect(self.on_lock_etalon)
        self.btn_unlock.clicked.connect(self.on_unlock_etalon)
        self.btn_enable_pid.clicked.connect(self.on_enable_pid)
        self.btn_disable_pid.clicked.connect(self.on_disable_pid)
        self.btn_update_pid.clicked.connect(self.on_update_pid_params)

        # Create the minimal laser controller using configuration parameters
        self.controller = self._create_laser_controller()
        self.controller.start()

        # QTimer to update the etalon tuning value display
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_tune_label)
        self.timer.start(100)  # update every 100 ms

    def _create_laser_controller(self):
        """Instantiate and return a MinimalLaserController using config values."""
        return MinimalLaserController(
            IP_ADDRESS,
            PORT,
            WAVENUMBER_PV,  # Assuming the EPICS PV now returns frequency in THz
            kp=KP,
            ki=KI,
            kd=KD
        )

    def update_tune_label(self):
        """Poll and update the current etalon tuning value in the GUI."""
        current_tune = self.controller.get_current_tuner_value()
        if current_tune is not None:
            self.tune_label.setText(f"Current Etalon Tune Value: {current_tune:.5f}")
        else:
            self.tune_label.setText("Current Etalon Tune Value: N/A")

    def on_lock_etalon(self):
        self.controller.lock_etalon()
        self.status_label.setText(f"Etalon Lock Status: {self.controller.get_etalon_lock_status()}")

    def on_unlock_etalon(self):
        self.controller.unlock_etalon()
        self.status_label.setText(f"Etalon Lock Status: {self.controller.get_etalon_lock_status()}")

    def on_enable_pid(self):
        val_str = self.target_input.text()
        try:
            target = float(val_str)
        except ValueError:
            QMessageBox.critical(self, "Error", "Please enter a valid number for the target frequency.")
            return
        self.controller.set_target_frequency(target)
        self.controller.enable_pid()

    def on_disable_pid(self):
        self.controller.disable_pid()

    def on_update_pid_params(self):
        dialog = PIDParameterDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            params = dialog.get_parameters()
            if params is None:
                QMessageBox.critical(self, "Error", "Please enter valid numeric values for PID parameters.")
                return
            kp, ki, kd = params
            self.controller.update_pid_parameters(kp, ki, kd)
            QMessageBox.information(self, "PID Update", "PID parameters have been updated.")

    def closeEvent(self, event):
        """Ensure background threads are stopped before closing."""
        self.controller.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MinimalGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
