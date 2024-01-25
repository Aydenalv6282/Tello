from djitellopy.tello import Tello
import time
import cv2
import numpy as np
from matplotlib import pyplot as plt
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
model_path = '/absolute/path/to/gesture_recognizer.task'

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

# Create a hand landmarker instance with the image mode:
options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path='models/hand_landmarker.task'),
    running_mode=VisionRunningMode.VIDEO, min_hand_detection_confidence=.1)

t = 0
with HandLandmarker.create_from_options(options) as landmarker:
    # The landmarker is initialized. Use it here.
    # ...
    # Load the input image from an image file.
    # mp_image = mp.Image.create_from_file('/path/to/image')

    import winwifi
    winwifi.WinWiFi.connect('TELLO-99A570')

    drone = Tello()

    drone.connect()

    drone.streamon()
    drone.takeoff()
    time.sleep(.5)
    drone.send_rc_control(forward_backward_velocity=0, up_down_velocity=0, left_right_velocity=0,
                          yaw_velocity=0)
    time.sleep(.5)
    drone.move_up(33)

    # @TODO: Call feed.py start_feed() from new thread. #

    while True:
        f = drone.get_frame_read(with_queue=False, max_queue_len=30)
        # cap = cv2.VideoCapture(0)
        # Read a frame from the camera
        # ret, frame = cap.read()
        # cv2.waitKey()
        time.sleep(.01)
        t += 10
        # @TODO: Call feed.py update_feed(f) with f.frame using the thread from earlier. #
        frame = cv2.cvtColor(f.frame, cv2.COLOR_RGB2BGR)
        #print("Checking...")

        # Load the input image from a numpy array.
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        # Perform hand landmarks detection on the provided single image.
        # The hand landmarker must be created with the image mode.
        hand_landmarker_result = landmarker.detect_for_video(mp_image, timestamp_ms=t)
        if hand_landmarker_result.handedness:
            hl = hand_landmarker_result.hand_landmarks
            hand_center_x = sum(hl[0][i].x for i in range(21)) / 21
            hand_center_y = 1-sum(hl[0][i].y for i in range(21)) / 21
            if hand_center_x > .6:
                #drone.rotate_clockwise(20)
                yaw_vel = 50
            elif hand_center_x < .4:
                #drone.rotate_counter_clockwise(20)
                yaw_vel = -50
            else:
                yaw_vel = 0
            if hand_center_y > .6:
                # drone.rotate_clockwise(20)
                up_vel = 30
            elif hand_center_y < .4:
                # drone.rotate_counter_clockwise(20)
                up_vel = -30
            else:
                up_vel = 0
            drone.send_rc_control(forward_backward_velocity=0, up_down_velocity=up_vel, left_right_velocity=0,
                                  yaw_velocity=yaw_vel)
            print(hand_center_x, hand_center_y)
        else:
            drone.send_rc_control(forward_backward_velocity=0, up_down_velocity=0, left_right_velocity=0,
                                  yaw_velocity=0)
        #cv2.imshow("gizmo", frame)
        #cv2.waitKey()




'''
import winwifi
winwifi.WinWiFi.connect('TELLO-99A570')

drone = Tello()

drone.connect()

drone.streamon()

f = drone.get_frame_read(with_queue=False, max_queue_len=30)
time.sleep(.1)
fram = cv2.resize(f.frame, (100, 100))

print(fram)
'''
'''

print(len(np.asarray(fram).flatten()))

cv2.imshow("gizmo", cv2.cvtColor(fram, cv2.COLOR_RGB2BGR))
cv2.waitKey()

count = 0'''
