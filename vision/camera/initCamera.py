from picamera2 import Picamera2

def start_camera() -> None:
    picam: Picamera2 = Picamera2()
    picam.start()