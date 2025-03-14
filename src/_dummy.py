import sys
import time
import threading
import random

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QMessageBox
)
from PyQt5.QtCore import Qt

try:
    import config
except ImportError:
    class config:
        DEBUG_MODE = True
        IP_ADDRESS = "127.0.0.1"
        PORT = 1234
        WAVENUMBER_PV = "SIM:WAVENUMBER"
        KP, KI, KD = 1.0, 0.1, 0.01
        READ_FREQ = 1.0

if not config.DEBUG_MODE:
    from epics import PV
    from pylablib.devices import M2

###############################################################################
#                            Dummy Classes for Debugging                      #
###############################################################################

class DummyEMAServerReader:
    """
    Simulates reading a wavenumber from a PV channel.
    """
    def __init__(self, pv_name, read_freq=config.READ_FREQ):
        self._read_freq = read_freq
        self._thread = None
        self._running = False
        self._current_wnum = 500.0  # Default simulated wavenumber

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
            self._current_wnum += random.uniform(-0.05, 0.05)  # Simulate fluctuation
            time.sleep(self._read_freq)

    def get_current_wavenumber(self):
        return self._current_wnum

class DummyLaserController:
    """
    Simulates laser and etalon control.
    """
    def __init__(self, ip_address, port, wavenumber_pv, kp, ki, kd):
        self.etalon_lock_status = "off"
        self.reader = DummyEMAServerReader(wavenumber_pv, read_freq=config.READ_FREQ)
        self._running = False
        self._pid_enabled = False

    def start(self):
        self.reader.start()
        self._running = True

    def stop(self):
        self._running = False
        self.reader.stop()

    def lock_etalon(self):
        self.etalon_lock_status = "on"

    def unlock_etalon(self):
        self.etalon_lock_status = "off"

    def get_etalon_lock_status(self):
        return self.etalon_lock_status

    def tune_etalon(self, new_value):
        pass  # No real tuning needed for dummy

    def set_target_wavenumber(self, wn):
        pass  # No real target setting in dummy mode

    def enable_pid(self):
        self._pid_enabled = True

    def disable_pid(self):
        self._pid_enabled = False

###############################################################################
#                          Minimal PyQt5 GUI                                  #
###############################################################################

class MinimalGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minimal Laser/Etalon Lock GUI [DEBUG MODE]" if config.DEBUG_MODE else "Minimal Laser/Etalon Lock GUI")
        self.setGeometry(100, 100, 400, 300)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        vbox = QVBoxLayout(central_widget)

        self.status_label = QLabel("Etalon Lock Status: off", self)
        vbox.addWidget(self.status_label, alignment=Qt.AlignCenter)

        hbox_target = QHBoxLayout()
        hbox_target.addWidget(QLabel("Target Wavenumber (cm^-1):", self))
        self.target_input = QLineEdit(self)
        hbox_target.addWidget(self.target_input)
        vbox.addLayout(hbox_target)

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

        self.btn_lock.clicked.connect(self.on_lock_etalon)
        self.btn_unlock.clicked.connect(self.on_unlock_etalon)
        self.btn_enable_pid.clicked.connect(self.on_enable_pid)
        self.btn_disable_pid.clicked.connect(self.on_disable_pid)

        self.controller = self._create_laser_controller()
        self.controller.start()

    def _create_laser_controller(self):
        if config.DEBUG_MODE:
            return DummyLaserController(
                config.IP_ADDRESS,
                config.PORT,
                config.WAVENUMBER_PV,
                kp=config.KP,
                ki=config.KI,
                kd=config.KD
            )
        else:
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
        try:
            target = float(self.target_input.text())
        except ValueError:
            QMessageBox.critical(self, "Error", "Enter a valid number for the target wavenumber.")
            return
        self.controller.set_target_wavenumber(target)
        self.controller.enable_pid()

    def on_disable_pid(self):
        self.controller.disable_pid()

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
