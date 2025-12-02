from picamera2 import Picamera2

def take_image(cam: Picamera2, path: str):
    cam.capture_file(path)
    return None