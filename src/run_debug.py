import sys
import time
import threading
import random

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer

# Debug Mode Flag
DEBUG_MODE = True  # Set to False to use real hardware

# Default Configuration Values
IP_ADDRESS = "192.168.1.222"
PORT = 39933
WAVENUMBER_PV = "DBEL_RISE:MTER_N0002:FR_RD"
KP = 40.0
KI = 0.8
KD = 0.0
DEAD_TIME = 0.2
EMA_ALPHA = 0.1
READ_FREQ = 0.1

if not DEBUG_MODE:
    from epics import PV
    from pylablib.devices import M2


###############################################################################
#                         Debugging Helpers (Mocking)                         #
###############################################################################

class MockPV:
    """Simulated EPICS PV that generates a random frequency in debug mode."""
    def __init__(self, pv_name):
        self.pv_name = pv_name
        self.value = 193.5  # Default simulated frequency (THz)

    def get(self):
        """Simulate random frequency variations."""
        self.value += random.uniform(-0.005, 0.005)  # Small random fluctuation
        return self.value


class MockLaser:
    """Simulated laser controller for debug mode."""
    def __init__(self, ip_address, port):
        self.ip_address = ip_address
        self.port = port
        self.etalon_tune_value = 10.0  # Arbitrary starting value

    def lock_etalon(self):
        print("[DEBUG] Etalon locked.")

    def unlock_etalon(self):
        print("[DEBUG] Etalon unlocked.")

    def tune_etalon(self, new_value, sync=True):
        """Simulate tuning the etalon."""
        self.etalon_tune_value = new_value
        print(f"[DEBUG] Etalon tuned to: {new_value:.5f}")

    def get_full_web_status(self):
        """Simulate getting status."""
        return {"etalon_tune": self.etalon_tune_value}


###############################################################################
#                         Reader and PID Controller                           #
###############################################################################

class EMAServerReader:
    """Reads frequency from an EPICS PV in a background thread (or simulates it in debug mode)."""
    def __init__(self, pv_name, read_freq=READ_FREQ):
        self._pv = MockPV(pv_name) if DEBUG_MODE else PV(pv_name)
        self._read_freq = read_freq
        self._thread = None
        self._running = False
        self._current_value = 0.0

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._reading_loop, daemon=True)
        self._thread.start()

    def stop(self):
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
        return self._current_value


class MinimalLaserController:
    """
    Controls the etalon lock and stabilizes the frequency using a PID controller.
    Uses a mocked laser if in debug mode.
    """
    def __init__(self, ip_address, port, pv_name, kp, ki, kd):
        self.laser = MockLaser(ip_address, port) if DEBUG_MODE else M2.Solstis(ip_address, port)
        self.reader = EMAServerReader(pv_name, read_freq=READ_FREQ)
        self._control_thread = None
        self._running = False
        self._pid_enabled = False
        self._ema_value = None  # For filtering
        self.etalon_lock_status = "off"

    def start(self):
        self.reader.start()
        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

    def stop(self):
        self._running = False
        self.reader.stop()
        if self._control_thread:
            self._control_thread.join()

    def lock_etalon(self):
        self.laser.lock_etalon()
        self.etalon_lock_status = "on"

    def unlock_etalon(self):
        self.laser.unlock_etalon()
        self.etalon_lock_status = "off"

    def get_etalon_lock_status(self):
        return self.etalon_lock_status

    def tune_etalon(self, new_value):
        self.laser.tune_etalon(new_value, sync=True)

    def get_current_tuner_value(self):
        return self.laser.etalon_tune_value

    def _control_loop(self):
        """Background loop for applying simulated PID corrections."""
        while self._running:
            if self._pid_enabled and (self.etalon_lock_status == "on"):
                raw_value = self.reader.get_current_value()
                if self._ema_value is None:
                    self._ema_value = raw_value
                else:
                    self._ema_value = EMA_ALPHA * raw_value + (1 - EMA_ALPHA) * self._ema_value

                correction = -0.1 * (self._ema_value - 193.5)  # Simulated correction

                current_etalon = self.get_current_tuner_value()
                new_val = current_etalon + correction
                self.tune_etalon(new_val)

            time.sleep(0.1)


###############################################################################
#                              PyQt5 GUI                                      #
###############################################################################

class MinimalGUI(QMainWindow):
    """PyQt5 GUI for debugging the laser control system."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minimal Laser Debug GUI")
        self.setGeometry(100, 100, 400, 300)

        # Layout
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        vbox = QVBoxLayout(central_widget)

        self.status_label = QLabel("Etalon Lock Status: off", self)
        vbox.addWidget(self.status_label, alignment=Qt.AlignCenter)

        self.tune_label = QLabel("Current Etalon Tune Value: N/A", self)
        vbox.addWidget(self.tune_label, alignment=Qt.AlignCenter)

        self.btn_lock = QPushButton("Lock Etalon", self)
        self.btn_unlock = QPushButton("Unlock Etalon", self)
        vbox.addWidget(self.btn_lock)
        vbox.addWidget(self.btn_unlock)

        self.btn_lock.clicked.connect(self.on_lock_etalon)
        self.btn_unlock.clicked.connect(self.on_unlock_etalon)

        self.controller = MinimalLaserController(IP_ADDRESS, PORT, WAVENUMBER_PV, kp=KP, ki=KI, kd=KD)
        self.controller.start()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_tune_label)
        self.timer.start(100)

    def update_tune_label(self):
        current_tune = self.controller.get_current_tuner_value()
        self.tune_label.setText(f"Current Etalon Tune Value: {current_tune:.5f}")

    def on_lock_etalon(self):
        self.controller.lock_etalon()
        self.status_label.setText(f"Etalon Lock Status: {self.controller.get_etalon_lock_status()}")

    def on_unlock_etalon(self):
        self.controller.unlock_etalon()
        self.status_label.setText(f"Etalon Lock Status: {self.controller.get_etalon_lock_status()}")

    def closeEvent(self, event):
        self.controller.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MinimalGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
