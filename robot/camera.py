import os
import platform
import traceback

import cv2

from robot import config


class Picamera2Camera:
    def __init__(self, width, height):
        from picamera2 import Picamera2

        self.camera = Picamera2()
        self.camera.configure(self.camera.create_preview_configuration(main={"size": (width, height), "format": "BGR888"}))
        self.camera.start()
        self.frame_count = 0
        print("Picamera2 started: " + str(width) + "x" + str(height) + " BGR888")

    def read(self):
        try:
            frame = self.camera.capture_array()
        except Exception:
            traceback.print_exc()
            return False, None
        if frame is None or len(frame.shape) != 3 or frame.shape[2] < 3:
            print("Picamera2 returned invalid frame: " + str(getattr(frame, "shape", None)))
            return False, None
        self.frame_count += 1
        frame = frame[:, :, :3]
        return True, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if config.PICAMERA2_SWAP_RED_BLUE else frame

    def release(self):
        self.camera.stop()


class OpenCVCamera:
    def __init__(self, width, height):
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.camera.isOpened():
            raise RuntimeError("Could not access camera through OpenCV")
        print("OpenCV camera started: " + str(width) + "x" + str(height))

    def read(self):
        return self.camera.read()

    def release(self):
        self.camera.release()


def open_camera(width, height):
    if config.CAMERA_BACKEND not in ("auto", "picamera2", "opencv"):
        raise ValueError("CAMERA_BACKEND must be auto, picamera2, or opencv")
    print("Camera backend=" + config.CAMERA_BACKEND + " python=" + platform.python_version() + " opencv=" + cv2.__version__)
    if config.CAMERA_BACKEND in ("auto", "picamera2"):
        try:
            return Picamera2Camera(width, height)
        except Exception:
            if config.CAMERA_BACKEND == "picamera2" or os.name != "nt":
                print("Picamera2 startup failed:")
                traceback.print_exc()
                raise
            print("Picamera2 unavailable, falling back to OpenCV.")
    return OpenCVCamera(width, height)
