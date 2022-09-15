import sys
from datetime import datetime

sys.path.append('/home/pi/cs437-picar')
import cv2
from road_sign_detector import Detector
import time
import collections


class FPS:
    def __init__(self, average_of=50):
        self.frame_timestamps = collections.deque(maxlen=average_of)

    def __call__(self):
        self.frame_timestamps.append(time.time())
        if len(self.frame_timestamps) > 1:
            return len(self.frame_timestamps) / (self.frame_timestamps[-1] - self.frame_timestamps[0])
        else:
            return 0.0


# if __name__ == "__main__":
#     sign_detector = Detector()
#     fps = FPS()
#     im_res = (320*2, 240*2)
#     with PiCamera() as camera:
#         camera.resolution = im_res
#         camera.framerate = 24
#         time.sleep(2)
#         frame = np.empty((im_res[1], im_res[0], 3), dtype=np.uint8)
#         camera.capture(frame, 'bgr')
#         print("Camera OK")
#         print(f"Image resolution: {frame.shape[:2]} pixels")
#
#         for _ in range(100):
#             camera.capture(frame, 'bgr')
#             bounding_boxes = sign_detector.detected(frame)
#             print(fps())
#             for road_sign, bb in bounding_boxes.items():
#                 if len(bb) > 0:
#                     print(road_sign.name)

im_res = (320, 240)
save_movie = True
sign_detector = Detector()
fps = FPS()
stream = cv2.VideoCapture(0)
stream.set(3, im_res[0])
stream.set(4, im_res[1])
out = cv2.VideoWriter(f'objdet_{datetime.now().strftime("%H:%M:%S")}.avi', cv2.VideoWriter_fourcc(*'XVID'), 20, im_res)


(grabbed, frame) = stream.read()
if grabbed:
    print("Camera OK")
    print(f"Image resolution: {frame.shape[:2]} pixels")
for _ in range(20):
    (grabbed, frame) = stream.read()
    if not grabbed:
        break
    bounding_boxes = sign_detector.detected(frame)
    for road_sign, bb in bounding_boxes.items():
        if len(bb) > 0:
            if save_movie:
                for (x, y, w, h) in bb:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), road_sign.value, 2)
                    cv2.putText(frame, road_sign.name, (x, y), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1.0,
                                color=road_sign.value)
            else:
                print(road_sign.name)
    if save_movie:
        fps_num = fps()
        cv2.putText(frame, f"fps: {fps_num:.2f}", (10, 28), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1.0,
                    color=(128, 255, 128))
        out.write(frame)
    else:
        print(fps())

out.release()
stream.release()
