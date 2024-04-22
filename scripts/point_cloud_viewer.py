import logging
from vzense.camera import VzenseCamera
import vzense.helper_functions

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    cam = VzenseCamera()
    cam.connect()
    latest_frame = cam.update()
    vzense.helper_functions.view_frame(latest_frame)