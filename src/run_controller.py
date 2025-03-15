#!/usr/bin/env python3
import sys
import time
import threading

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QCheckBox, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer

from epics import PV
from pylablib.devices import M2

###############################################################################
#                           Configuration Dictionary                          #
###############################################################################
CONFIG = {
    "ip_address": "192.168.1.222",
    "port": 39933,
    "frequency_pv": "DBEL_RISE:MTER_N0002:FR_RD",  # EPICS PV for frequency (THz)
    "reading_frequency": 0.5,  # seconds between successive PV reads
    "pid": {
        "kp": 40.0,
        "ki": 0.8,
        "kd": 0.0,
        "setpoint": 0.0  # Default setpoint (THz)
    },
    "update_delay": 0.2,  # Delay (in seconds) after each tuning command
    "ema_alpha": 0.9      # Smoothing factor for exponential moving average (0 < alpha <= 1)
}

###############################################################################
#                          1. EPICS Frequency Reader                          #
###############################################################################

class EPICSFrequencyReader:
    """
    Continuously read the frequency (in THz) from an EPICS PV in a background thread.
    """
    def __init__(self, pv_name, read_freq):
        """
        Args:
            pv_name (str): The EPICS PV name that provides the frequency in THz.
            read_freq (float): Frequency (in seconds) between successive reads.
        """
        self._pv = PV(pv_name)
        self._read_freq = read_freq
        self._thread = None
        self._running = False
        self._current_freq = 0.0

    def start(self):
        """Start continuously reading frequency in a background thread."""
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
        """Background loop to read the EPICS PV."""
        while self._running:
            val = self._pv.get()
            if val is not None:
                try:
                    self._current_freq = float(val)
                except ValueError:
                    pass
            time.sleep(self._read_freq)

    def get_current_frequency(self):
        """Return the latest frequency in THz from the EPICS PV."""
        return self._current_freq

###############################################################################
#                    2. Simple PID Controller for Laser Frequency            #
###############################################################################

class PIDController:
    """
    A simple PID controller for locking the frequency to a setpoint (in THz).
    """
    def __init__(self, kp, ki, kd, setpoint=0.0):
        """
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain
            kd (float): Derivative gain
            setpoint (float): Desired frequency in THz
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
        Compute the correction based on the difference between setpoint and
        current_value. Returns a float that the caller will use to tune the laser.
        """
        now = time.time()
        error = self.setpoint - current_value  # in THz
        dt = now - self._prev_time if not self._first_update else 0.0
        if dt <= 0:
            dt = 1e-6

        # Integrate error
        self._integral += error * dt

        # Derivative
        derivative = 0.0 if self._first_update else (error - self._prev_error) / dt

        # PID output
        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)

        # Update state
        self._prev_time = now
        self._prev_error = error
        self._first_update = False

        return output

    def set_setpoint(self, new_setpoint):
        """Set a new frequency setpoint and reset internal states."""
        self.setpoint = new_setpoint
        self.reset()

    def reset(self):
        """Reset the integral and error terms."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.time()
        self._first_update = True

###############################################################################
#                  3. Minimal Laser Controller using M2.Solstis              #
###############################################################################

class MinimalLaserController:
    """
    Uses:
      - EPICSFrequencyReader to get the current frequency
      - M2.Solstis to lock/unlock the etalon
      - tune_laser_resonator() to adjust the resonator (coarse or fine)
      - PID logic for closed-loop control
      - A time delay after each tuning command to allow hardware to settle
      - An exponential moving average (EMA) filter for noisy frequency readings.
    """

    def __init__(self, ip_address, port, freq_pv, pid_params, read_freq, update_delay, ema_alpha):
        """
        Args:
            ip_address (str): IP address for the M2 Solstis laser.
            port (int): Port for the M2 Solstis laser.
            freq_pv (str): EPICS PV that reads out the laser frequency in THz.
            pid_params (dict): Dictionary containing PID parameters (kp, ki, kd, setpoint).
            read_freq (float): Seconds between EPICS PV reads.
            update_delay (float): Delay (in seconds) after each tuning command.
            ema_alpha (float): Smoothing factor for exponential moving average.
        """
        # Connect to M2 laser
        self.laser = M2.Solstis(ip_address, port)

        # Single lock: "etalon" (only allow tuning if locked)
        self.etalon_lock_status = "off"

        # Reader from EPICS
        self.reader = EPICSFrequencyReader(freq_pv, read_freq)

        # PID controller for frequency locking
        self.pid = PIDController(pid_params["kp"], pid_params["ki"], pid_params["kd"],
                                 setpoint=pid_params["setpoint"])

        self._control_thread = None
        self._running = False
        self._pid_enabled = False

        # Additional delay time for hardware settling after tuning commands
        self._update_delay = update_delay

        # Flag for fine tuning (if True, call tune_laser_resonator with fine=True)
        self.fine_tuning = False

        # EMA parameters: smoothing factor and current EMA value
        self.ema_alpha = ema_alpha
        self._ema = None

    ############# Start / Stop #############
    def start(self):
        """Start reading from EPICS and start the control loop thread."""
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

    ############# Lock / Unlock #############
    def lock_etalon(self):
        """Lock the etalon in hardware and update local status."""
        self.laser.lock_etalon()   # M2 method
        self.etalon_lock_status = "on"

    def unlock_etalon(self):
        """Unlock the etalon in hardware and update local status."""
        self.laser.unlock_etalon() # M2 method
        self.etalon_lock_status = "off"

    def get_etalon_lock_status(self):
        return self.etalon_lock_status

    ############# Tuning the Resonator #############
    def tune_resonator(self, fraction):
        """
        Tune the laser resonator to a given value (as a percentage).
        If self.fine_tuning is True, perform a fine tune.
        """
        self.laser.tune_laser_resonator(value=fraction, fine=self.fine_tuning, sync=True)

    ############# PID Control #############
    def set_target_frequency(self, freq_thz):
        """Set the new target frequency (in THz) for the PID loop."""
        self.pid.set_setpoint(freq_thz)

    def enable_pid(self):
        """Enable the PID control loop (only if etalon lock is on)."""
        if self.etalon_lock_status != "on":
            print("Warning: Cannot enable PID since etalon is not locked.")
            return
        self._pid_enabled = True
        self.pid.reset()

    def disable_pid(self):
        self._pid_enabled = False

    def _control_loop(self):
        """
        Background loop that:
          1. Reads the current frequency.
          2. Applies an exponential moving average (EMA) filter.
          3. If PID is enabled and etalon is locked, calculates correction.
          4. Adjusts the resonator using tune_laser_resonator.
          5. Waits for _update_delay seconds before the next update.
        """
        while self._running:
            raw_freq = self.reader.get_current_frequency()
            if self._ema is None:
                self._ema = raw_freq
            else:
                self._ema = self.ema_alpha * raw_freq + (1 - self.ema_alpha) * self._ema

            if self._pid_enabled and self.etalon_lock_status == "on":
                correction = self.pid.update(self._ema)

                # Retrieve current resonator fraction from full_web_status.
                current_fraction = float(self.laser.get_full_web_status()["resonator_tune"])

                # Scale correction to a fraction step (tweak as needed for your hardware)
                fraction_step = correction * 5.0
                new_fraction = current_fraction - fraction_step

                # Bound within [0, 100]
                new_fraction = max(0, min(100, new_fraction))

                self.tune_resonator(new_fraction)

                # Wait for hardware to settle
                time.sleep(self._update_delay)
            else:
                time.sleep(0.05)

###############################################################################
#                           4. PyQt5 GUI Implementation                        #
###############################################################################

class MinimalLockGUI(QMainWindow):
    """
    A simple PyQt5 GUI to:
      - Display current frequency (THz)
      - Lock/Unlock the etalon
      - Enable/Disable PID
      - Provide a target frequency input
      - Toggle fine/coarse tuning
      - Display status information in real time
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minimal Laser Resonator Lock with PID")
        self.setGeometry(100, 100, 420, 300)

        # Main layout
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 1) Current frequency display
        freq_hbox = QHBoxLayout()
        freq_hbox.addWidget(QLabel("Current Frequency (THz):"))
        self.lbl_current_freq = QLabel("0.00000")
        freq_hbox.addWidget(self.lbl_current_freq)
        layout.addLayout(freq_hbox)

        # 2) Etalon lock status display
        self.lbl_lock_status = QLabel("Etalon Lock Status: off")
        layout.addWidget(self.lbl_lock_status)

        # 3) Target frequency input
        target_hbox = QHBoxLayout()
        target_hbox.addWidget(QLabel("Target Frequency (THz):"))
        self.edit_target_freq = QLineEdit("0.00000")
        target_hbox.addWidget(self.edit_target_freq)
        layout.addLayout(target_hbox)

        # 4) Fine Tuning checkbox
        self.chk_fine_tuning = QCheckBox("Fine Tuning")
        layout.addWidget(self.chk_fine_tuning)

        # 5) Buttons: Lock/Unlock, Enable/Disable PID
        button_hbox = QHBoxLayout()
        self.btn_lock = QPushButton("Lock Etalon")
        self.btn_unlock = QPushButton("Unlock Etalon")
        button_hbox.addWidget(self.btn_lock)
        button_hbox.addWidget(self.btn_unlock)
        self.btn_enable_pid = QPushButton("Enable PID")
        self.btn_disable_pid = QPushButton("Disable PID")
        button_hbox.addWidget(self.btn_enable_pid)
        button_hbox.addWidget(self.btn_disable_pid)
        layout.addLayout(button_hbox)

        # Connect signals to handlers
        self.btn_lock.clicked.connect(self.on_lock_etalon)
        self.btn_unlock.clicked.connect(self.on_unlock_etalon)
        self.btn_enable_pid.clicked.connect(self.on_enable_pid)
        self.btn_disable_pid.clicked.connect(self.on_disable_pid)
        self.chk_fine_tuning.stateChanged.connect(self.on_fine_tuning_changed)

        # Create the laser controller using parameters from CONFIG
        self.controller = self._create_laser_controller()
        self.controller.start()

        # Timer to update GUI periodically
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.on_update_gui)
        self.update_timer.start(500)  # update every 0.5 seconds

    def _create_laser_controller(self):
        """Instantiate the MinimalLaserController using values from CONFIG."""
        return MinimalLaserController(
            ip_address=CONFIG["ip_address"],
            port=CONFIG["port"],
            freq_pv=CONFIG["frequency_pv"],
            pid_params=CONFIG["pid"],
            read_freq=CONFIG["reading_frequency"],
            update_delay=CONFIG["update_delay"],
            ema_alpha=CONFIG["ema_alpha"]
        )

    ############################################################################
    #                           GUI Event Handlers                             #
    ############################################################################

    def on_lock_etalon(self):
        self.controller.lock_etalon()
        self.lbl_lock_status.setText(f"Etalon Lock Status: {self.controller.get_etalon_lock_status()}")

    def on_unlock_etalon(self):
        self.controller.unlock_etalon()
        self.lbl_lock_status.setText(f"Etalon Lock Status: {self.controller.get_etalon_lock_status()}")

    def on_enable_pid(self):
        try:
            target_freq = float(self.edit_target_freq.text())
        except ValueError:
            QMessageBox.critical(self, "Error", "Enter a valid numeric value for target frequency (THz).")
            return
        self.controller.set_target_frequency(target_freq)
        self.controller.enable_pid()

    def on_disable_pid(self):
        self.controller.disable_pid()

    def on_fine_tuning_changed(self, state):
        self.controller.fine_tuning = (state == Qt.Checked)

    def on_update_gui(self):
        """Periodically update current frequency and lock status."""
        current_freq = self.controller.reader.get_current_frequency()
        self.lbl_current_freq.setText(f"{current_freq:.5f}")
        lock_status = self.controller.get_etalon_lock_status()
        self.lbl_lock_status.setText(f"Etalon Lock Status: {lock_status}")

    def closeEvent(self, event):
        """Ensure background threads are stopped before closing."""
        self.controller.stop()
        event.accept()

###############################################################################
#                                Main Application                             #
###############################################################################

def main():
    app = QApplication(sys.argv)
    window = MinimalLockGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
