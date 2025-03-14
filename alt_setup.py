# For py2exe

from distutils.core import setup
import py2exe

setup(
    windows=[{"script": "./src/_all.py"}],  # Replace with your script name
    options={"py2exe": {"includes": ["PyQt5", "epics", "pylablib.devices", "config"]}},
)
