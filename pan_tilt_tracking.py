# USAGE
# python pan_tilt_tracking.py --cascade haarcascade_frontalface_default.xml
# import necessary packages
from multiprocessing import Manager, process
from multiprocessing import Process
from objcenter import ObjCenter
from pid import PID
import argparse
import signal
import time
import sys
import cv2
import math
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo, AngularServo
# from picamera.array import PiRGBArray  # Generates a 3D RGB array
# from picamera import PiCamera  # Provides a Python interface for the RPi Camera Module
import time
import csv
import pandas as pd
import pyfakewebcam


servo_range = (-45, 45)

# define the range for the motors
pigpio_factory = PiGPIOFactory()
pan_servo = AngularServo(17, min_angle=-45, max_angle=45, pin_factory=pigpio_factory)
tilt_servo = AngularServo(27, min_angle=-45, max_angle=45, pin_factory=pigpio_factory)
pan_servo.angle = 0
tilt_servo.angle = 0

fake_camera = pyfakewebcam.FakeWebcam("/dev/video1", 640, 480)

# function to handle keyboard interrupt
def signal_handler(sig, frame):
    # print a status message
    print("[INFO] You pressed `ctrl + c`! Exiting...")
    # exit
    sys.exit()


def obj_center(args, objX, objY, centerX, centerY):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    # start the video stream and wait for the camera to warm up
    # vs = VideoStream().start()
    device = cv2.CAP_V4L2
    cam = cv2.VideoCapture(device)
    # cam.set(3,1296)#width
    # cam.set(4,972)#height
    # cap.set(10,100)#brightness
    time.sleep(0.6)
    # if capture failed to open, try again
    if not cam.isOpened():
        cam.open(device)

    # initialize the object center finder
    obj = ObjCenter()
    

    # # Initialize the camera
    # camera = PiCamera()
    # # Set the camera resolution
    # camera.resolution = (640, 480)
    # # Set the number of frames per second
    # camera.framerate = 32
    # # Generates a 3D RGB array and stores it in rawCapture
    # raw_capture = PiRGBArray(camera, size=(640, 480))
    # # Wait a certain number of seconds to allow the camera time to warmup
    # time.sleep(0.1)

    if cam.isOpened():
        # loop indefinitely
        width  = cam.get(cv2.CAP_PROP_FRAME_WIDTH)   # float `width`
        height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float `height`
        # or
        width  = cam.get(3)  # float `width`
        height = cam.get(4)  # float `height`

        print('width, height:', width, height)
        fps = cam.get(cv2.CAP_PROP_FPS)
        # or
        fps = cam.get(5)
        print('fps:', fps)  # float `fps`
        frame_count = cam.get(cv2.CAP_PROP_FRAME_COUNT)
        # or
        frame_count = cam.get(7)
        print('frames count:', frame_count)  # float `frame_count`

        # Print supported resolutions
        # url = "https://en.wikipedia.org/wiki/List_of_common_resolutions"
        # table = pd.read_html(url)[0]
        # table.columns = table.columns.droplevel()
        # resolutions = {}
        # for index, row in table[["W", "H"]].iterrows():
        #     cam.set(cv2.CAP_PROP_FRAME_WIDTH, row["W"])
        #     cam.set(cv2.CAP_PROP_FRAME_HEIGHT, row["H"])
        #     width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        #     height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        #     resolutions[str(width)+"x"+str(height)] = "OK"
        # print(resolutions)

        # cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        # cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024)

        width  = cam.get(3)  # float `width`
        height = cam.get(4)  # float `height`

        print('width, height:', width, height)
        fps = cam.get(cv2.CAP_PROP_FPS)
        count = 0;
        while True:
            # grab the frame from the threaded video stream and flip it
            # vertically (since our camera was upside down)
            # Grab the raw NumPy array representing the image
            ret, frame = cam.read()
            count = count + 1
            initial = True
            if(count == 5000): 
                count = 0
            if ret:
                cv2.imshow("Pan-Tilt Face Tracking", frame)
                # frame = cv2.flip(frame, 0)
                frame = cv2.flip(frame, 1)
                fake_camera_frame = frame.copy()
                # calculate the center of the frame as this is where we will
                # try to keep the object
                (H, W) = frame.shape[:2]
                centerX.value = W // 2
                centerY.value = H // 2

                # find the object's location
                
                if count % 5 == 0 or initial:
                    objectLoc = obj.update(frame, (centerX.value, centerY.value))
                    intial = False
                ((objX.value, objY.value), rect) = objectLoc

                # extract the bounding box and draw it
                if rect is not None:
                    (x, y, w, h) = rect
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # display the frame to the screen
                # frame = cv2.flip(frame, 0)
                # frame = cv2.flip(frame, 1)
                cv2.imshow("Pan-Tilt Face Tracking", frame)
                fake_camera.schedule_frame(fake_camera_frame[...,  ::-1])
                cv2.waitKey(1)
            else:
                print("Error reading capturing device")
    else:
        print("Failed to open capture device")



def pid_process(output, p, i, d, objCoord, centerCoord):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # create a PID and initialize it
    p = PID(p.value, i.value, d.value)
    p.initialize()
    time.sleep(1)

    # loop indefinitely
    while True:
        # calculate the error
        error = objCoord.value - centerCoord.value 

        # update the value
        output.value = p.update(error)


def in_range(val, start, end):
    # determine the input vale is in the supplied range
    return (val >= start and val <= end)


def pan_the_angle(angle):
    if in_range(angle, servo_range[0], servo_range[1]):
        pan_servo.angle = angle
    elif angle < servo_range[0]:
        pan_servo.angle = servo_range[0]
    elif angle > servo_range[1]:
        pan_servo.angle = servo_range[1]


def tilt_the_angle(angle):
    if in_range(angle, servo_range[0], servo_range[1]):
        tilt_servo.angle = angle
    elif angle < servo_range[0]:
        tilt_servo.angle = servo_range[0]
    elif angle > servo_range[1]:
        tilt_servo.angle = servo_range[1]


def set_servos(pan_target, tilt_target):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    speed = 1
    # loop indefinitely
    time.sleep(1)
    while True:
        # the pan and tilt angles are reversed
        panAngle = -1 * pan.value
        tltAngle = -1 * tlt.value
        # if the pan angle is within the range, pan
        if in_range(pan_target.value, servo_range[0], servo_range[1]):
            # pan_the_angle(pan_target.value)
           
            if (abs(pan_servo.angle - pan_target.value) < speed):
                pass
            elif (pan_servo.angle < pan_target.value):
                pan_servo.angle += speed
            elif (pan_servo.angle > pan_target.value):
                pan_servo.angle -= speed
        # if the tilt angle is within the range, tilt
        if in_range(tilt_target.value, servo_range[0], servo_range[1]):
            # tilt_the_angle(tilt_target.value)
           
            if (abs(tilt_servo.angle - tilt_target.value) < speed):
                pass
            elif (tilt_servo.angle < tilt_target.value):
                tilt_servo.angle += speed
            elif (tilt_servo.angle > tilt_target.value):
                tilt_servo.angle -= speed
        time.sleep(0.1)



def log(head_x, head_y, frame_x, frame_y, pan, tlt):
    # writing to csv file
    with open("logs.csv", 'a') as csvfile:
        # creating a csv writer object
        csvwriter = csv.writer(csvfile)
        while True:
            # writing the fields
            csvwriter.writerow([time.time(), head_x.value, head_y.value, frame_x.value, frame_y.value, pan.value, tlt.value])

# check to see if this is the main body of execution
if __name__ == "__main__":
    # start a manager for managing process-safe variables
    with Manager() as manager:
        # set integer values for the object center (x, y)-coordinates
        centerX = manager.Value("i", 0)
        centerY = manager.Value("i", 0)

        # set integer values for the object's (x, y)-coordinates
        objX = manager.Value("i", 0)
        objY = manager.Value("i", 0)

        # pan and tilt values will be managed by independed PIDs
        pan = manager.Value("i", 0)
        tlt = manager.Value("i", 0)

        # set PID values for panning
        panP = manager.Value("f", 0.045)
        panI = manager.Value("f", 0.03)
        panD = manager.Value("f", 0.01)

        # set PID values for tilting
        tiltP = manager.Value("f", 0.045)
        tiltI = manager.Value("f", 0.03)
        tiltD = manager.Value("f", 0.01)

        # we have 4 independent processes
        # 1. objectCenter  - finds/localizes the object
        # 2. panning       - PID control loop determines panning angle
        # 3. tilting       - PID control loop determines tilting angle
        # 4. setServos     - drives the servos to proper angles based
        #                    on PID feedback to keep object in center
        processObj = Process(target=obj_center, args=("haarcascade_frontalface_default.xml", objX, objY, centerX, centerY))
        processPan = Process(target=pid_process, args=(pan, panP, panI, panD, objX, centerX))
        processTilt = Process(target=pid_process, args=(tlt, tiltP, tiltI, tiltD, objY, centerY))
        processSetServos = Process(target=set_servos, args=(pan, tlt))
        processLog = Process(target=log,args = (objX, objY, centerX, centerY, pan, tlt))

        # start processes
        processObj.start()
        processPan.start()
        processTilt.start()
        processSetServos.start()
        # processLog.start()

        processObj.join()
        processPan.join()
        processTilt.join()
        processSetServos.join()
        # processLog.join()