import sys
import time
import threading

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QMessageBox
)
from PyQt5.QtCore import Qt

from epics import PV
from pylablib.devices import M2

# Import configuration parameters
import .config


###############################################################################
#                  Minimal Classes for Reading, PID, and Laser Control        #
###############################################################################

class EMAServerReader:
    """
    Minimal reader for the wavenumber from an EPICS PV in a background thread.
    """
    def __init__(self, pv_name, read_freq=config.READ_FREQ):
        """
        Args:
            pv_name (str): The EPICS PV name that provides the wavenumber.
            read_freq (float): Frequency (seconds) between successive reads.
        """
        self._pv = PV(pv_name)
        self._read_freq = read_freq
        self._thread = None
        self._running = False
        self._current_wnum = 0.0

    def start(self):
        """Start continuously reading the wavenumber in a background thread."""
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
                    self._current_wnum = round(float(val), 5)
                except ValueError:
                    self._current_wnum = 0.0
            time.sleep(self._read_freq)

    def get_current_wavenumber(self):
        """Return the latest wavenumber read from EPICS."""
        return self._current_wnum


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
            setpoint (float): Desired wavenumber target.
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
        Compute the correction based on the difference
        between setpoint and current_value.
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
      1) Reading the wavenumber from an EPICS PV,
      2) Locking/Unlocking the etalon, and
      3) A PID control loop for stabilizing the wavenumber.
    """
    def __init__(self, ip_address, port, wavenumber_pv, kp, ki, kd):
        """
        Args:
            ip_address (str): IP address for the M2 SolsTiS Laser.
            port (int): Port for the M2 SolsTiS Laser.
            wavenumber_pv (str): EPICS PV name for the wavenumber.
            kp, ki, kd (float): PID gains.
        """
        # Connect to the M2 laser
        self.laser = M2.Solstis(ip_address, port)

        # Use a single etalon lock (initially unlocked)
        self.etalon_lock_status = "off"

        # Create the reader for the wavenumber from the EPICS PV
        self.reader = EMAServerReader(wavenumber_pv, read_freq=config.READ_FREQ)

        # Create the PID controller for controlling the wavenumber
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
    def set_target_wavenumber(self, wn):
        """Set a new target wavenumber for the PID controller."""
        self.pid.set_setpoint(wn)

    def enable_pid(self):
        """Enable the PID control loop."""
        self._pid_enabled = True
        self.pid.reset()

    def disable_pid(self):
        """Disable the PID control loop."""
        self._pid_enabled = False

    def _control_loop(self):
        """
        Background loop for applying PID corrections to the etalon
        when PID is enabled and the etalon is locked.
        """
        while self._running:
            if self._pid_enabled and (self.etalon_lock_status == "on"):
                current_wnum = self.reader.get_current_wavenumber()
                correction = self.pid.update(current_wnum)

                # Retrieve current etalon tuner value and apply the correction
                current_etalon = float(self.laser.get_full_web_status()["etalon_tune"])
                # Scale the correction as needed (here, multiplying by 0.1)
                new_val = current_etalon - correction * 0.1

                self.tune_etalon(new_val)
            time.sleep(0.05)


###############################################################################
#                          Minimal PyQt5 GUI                                  #
###############################################################################

class MinimalGUI(QMainWindow):
    """
    A simple PyQt5 GUI for locking the etalon and stabilizing the wavenumber.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minimal Laser/Etalon Lock GUI")
        self.setGeometry(100, 100, 400, 300)

        # Central widget and layout
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        vbox = QVBoxLayout(central_widget)

        # Status label
        self.status_label = QLabel("Etalon Lock Status: off", self)
        vbox.addWidget(self.status_label, alignment=Qt.AlignCenter)

        # Target wavenumber input
        hbox_target = QHBoxLayout()
        hbox_target.addWidget(QLabel("Target Wavenumber (cm^-1):", self))
        self.target_input = QLineEdit(self)
        hbox_target.addWidget(self.target_input)
        vbox.addLayout(hbox_target)

        # Buttons for lock/unlock and PID control
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

        # Connect buttons to event handlers
        self.btn_lock.clicked.connect(self.on_lock_etalon)
        self.btn_unlock.clicked.connect(self.on_unlock_etalon)
        self.btn_enable_pid.clicked.connect(self.on_enable_pid)
        self.btn_disable_pid.clicked.connect(self.on_disable_pid)

        # Create the minimal laser controller using configuration parameters
        self.controller = self._create_laser_controller()
        self.controller.start()

    def _create_laser_controller(self):
        """Instantiate and return a MinimalLaserController using config values."""
        return MinimalLaserController(
            config.IP_ADDRESS,
            config.PORT,
            config.WAVENUMBER_PV,
            kp=config.KP,
            ki=config.KI,
            kd=config.KD
        )

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
            QMessageBox.critical(self, "Error", "Please enter a valid number for the target wavenumber.")
            return
        self.controller.set_target_wavenumber(target)
        self.controller.enable_pid()

    def on_disable_pid(self):
        self.controller.disable_pid()

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
