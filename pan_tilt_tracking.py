# USAGE
# python pan_tilt_tracking.py --cascade haarcascade_frontalface_default.xml
# import necessary packages
from multiprocessing import Manager
from multiprocessing import Process
from imutils.video import VideoStream
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
from picamera.array import PiRGBArray  # Generates a 3D RGB array
from picamera import PiCamera  # Provides a Python interface for the RPi Camera Module
import time
import csv

servo_range = (-45, 45)

# define the range for the motors
pigpio_factory = PiGPIOFactory()
pan_servo = AngularServo(17, min_angle=-45, max_angle=45, pin_factory=pigpio_factory)
tilt_servo = AngularServo(27, min_angle=-45, max_angle=45, pin_factory=pigpio_factory)
pan_servo.angle = 0
tilt_servo.angle = 0


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
    vs = VideoStream(usePiCamera=True).start()
    time.sleep(0.6)

    # initialize the object center finder
    obj = ObjCenter("haarcascade_frontalface_default.xml")

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

    # loop indefinitely
    while True:
        # grab the frame from the threaded video stream and flip it
        # vertically (since our camera was upside down)
        # Grab the raw NumPy array representing the image
        frame = vs.read()
        cv2.imshow("Pan-Tilt Face Tracking", frame)
        frame = cv2.flip(frame, 1)

        # calculate the center of the frame as this is where we will
        # try to keep the object
        (H, W) = frame.shape[:2]
        centerX.value = W // 2
        centerY.value = H // 2

        # find the object's location
        objectLoc = obj.update(frame, (centerX.value, centerY.value))
        ((objX.value, objY.value), rect) = objectLoc

        # extract the bounding box and draw it
        if rect is not None:
            (x, y, w, h) = rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # display the frame to the screen
        frame = cv2.flip(frame, 1)
        cv2.imshow("Pan-Tilt Face Tracking", frame)
        cv2.waitKey(1)


def pid_process(output, p, i, d, objCoord, centerCoord):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # create a PID and initialize it
    p = PID(p.value, i.value, d.value)
    p.initialize()

    # loop indefinitely
    while True:
        # calculate the error
        error = centerCoord.value - objCoord.value

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


def set_servos(pan, tlt):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # loop indefinitely
    while True:
        # the pan and tilt angles are reversed
        panAngle = -1 * pan.value
        tltAngle = -1 * tlt.value
        # if the pan angle is within the range, pan
        if in_range(panAngle, servo_range[0], servo_range[1]):
            pan_the_angle(panAngle)
        # if the tilt angle is within the range, tilt
        if in_range(tltAngle, servo_range[0], servo_range[1]):
            tilt_the_angle(tltAngle)


def log(head_x, head_y, frame_x, frame_y, pan, tlt):
    # writing to csv file
    with open("logs.csv", 'a') as csvfile:
        # creating a csv writer object
        csvwriter = csv.writer(csvfile)
        # writing the fields
        csvwriter.writerow([head_x, head_y, frame_x, frame_y, pan, tlt])


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
        panP = manager.Value("f", 0.06)
        panI = manager.Value("f", 0.07)
        panD = manager.Value("f", 0.003)

        # set PID values for tilting
        tiltP = manager.Value("f", 0.06)
        tiltI = manager.Value("f", 0.07)
        tiltD = manager.Value("f", 0.003)

        # we have 4 independent processes
        # 1. objectCenter  - finds/localizes the object
        # 2. panning       - PID control loop determines panning angle
        # 3. tilting       - PID control loop determines tilting angle
        # 4. setServos     - drives the servos to proper angles based
        #                    on PID feedback to keep object in center
        processObjectCenter = Process(target=obj_center,
                                      args=("haarcascade_frontalface_default.xml", objX, objY, centerX, centerY))
        processPanning = Process(target=pid_process,
                                 args=(pan, panP, panI, panD, objX, centerX))
        processTilting = Process(target=pid_process,
                                 args=(tlt, tiltP, tiltI, tiltD, objY, centerY))
        processSetServos = Process(target=set_servos, args=(pan, tlt))
        processLog = Process(target=log(objX, objY, centerX, centerY, pan, tlt))

        # start processes
        processObjectCenter.start()
        processPanning.start()
        processTilting.start()
        processSetServos.start()

        processObjectCenter.join()
        processPanning.join()
        processTilting.join()
        processSetServos.join()
