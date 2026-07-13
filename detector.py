"""Compatibility entrypoint for the original detector command.

Use `python3 main.py` for new work. This file intentionally delegates to it so
Pi deployments that still run `python3 detector.py` keep working.
"""
from main import run


if __name__ == "__main__":
    run()
