from picamera2 import Picamera2

def start_camera() -> Picamera2:
    picam: Picamera2 = Picamera2()
    picam.start()
    return picam