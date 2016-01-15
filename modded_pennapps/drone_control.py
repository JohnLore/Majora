import pygame

from libardrone import libardrone

from cv2 import *
import numpy as np

from kalman2d import Kalman2D

import urllib2
import sys
import time
from __builtin__ import max,min
from functools import partial

def report_parameter_change(parameter, val):
  print "%s changed to %d" % (parameter, val)

def addControl(name, default, maxval):
  createTrackbar(name, "control", default, maxval,
    partial(report_parameter_change, name)
  )

def main():
    HUE_LOWERB = 40
    HUE_UPPERB = 100
    SAT_LOWERB = 50
    SAT_UPPERB = 170
    BRI_LOWERB = 50
    BRI_UPPERB = 255
    EROSION = 3
    DILATION = 4
    FOLLOW_WIDTH = 50
    DRONE_SPEED = 10/100.
    CIRCLE_SPEED = 10/100.
    TURN_SPEED = 50/100.
    HFUZZ = 100
    VFUZZ = 100
    testing=False

    if len(sys.argv) > 1:
        if sys.argv[1] == 'testing':
            testing = True
        elif sys.argv[1] == 'control':
            control = True
    pygame.init()
    print "initialized"

    screen = pygame.display.set_mode((320, 240))
    navdata_0 = None
    if not testing:
        drone = libardrone.ARDrone(True)
        drone.reset()
        navdata_0 = drone.navdata.copy()

    tracking_center = False
    k2d = Kalman2D(1, 0.0001, 0.1)
    estimated = False

    namedWindow("output", 1)

    namedWindow("threshold", 1)
    moveWindow("threshold", 1000, 0)

    namedWindow("control", WINDOW_NORMAL)
    resizeWindow("control", 1024, 300)
    moveWindow("control", 0, 1000)

    addControl("hue lower", HUE_LOWERB, 255)
    addControl("hue upper", HUE_UPPERB, 255)
    addControl("sat lower", SAT_LOWERB, 255)
    addControl("sat upper", SAT_UPPERB, 255)
    addControl("bri lower", BRI_LOWERB, 255)
    addControl("bri upper", BRI_UPPERB, 255,)
    addControl("erosion", EROSION, 10)
    addControl("dilation", DILATION, 10)
    addControl("dilation", DILATION, 10)
    addControl("follow width", FOLLOW_WIDTH, 100)
    addControl("drone speed", int(DRONE_SPEED*100), 100)
    addControl("turn speed", int(TURN_SPEED*100), 100)
    addControl("circle speed", int(CIRCLE_SPEED*100), 100)

    if testing:
        capture = VideoCapture(0)
        while (not capture.isOpened()):
            print "not opened"

    running = True
    while running:
        HUE_LOWERB = getTrackbarPos("hue lower", "control")
        HUE_UPPERB = getTrackbarPos("hue upper", "control")
        SAT_LOWERB = getTrackbarPos("sat lower", "control")
        SAT_UPPERB = getTrackbarPos("sat upper", "control")
        BRI_LOWERB = getTrackbarPos("bri lower", "control")
        BRI_UPPERB = getTrackbarPos("bri upper", "control")
        EROSION = getTrackbarPos("erosion", "control")
        DILATION = getTrackbarPos("dilation", "control")
        FOLLOW_WIDTH = getTrackbarPos("follow width", "control")
        DRONE_SPEED = getTrackbarPos("drone speed", "control")/100.
        CIRCLE_SPEED = getTrackbarPos("circle speed", "control")/100.
        TURN_SPEED = getTrackbarPos("turn speed", "control")/100.

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYUP:
                drone.hover()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    drone.reset()
                    running = False
                # takeoff / land
                elif event.key == pygame.K_RETURN:
                    print("take off")
                    drone.takeoff()
                elif event.key == pygame.K_SPACE:
                    print("land")
                    drone.land()
                # emergency
                elif event.key == pygame.K_BACKSPACE:
                    drone.reset()
                # forward / backward
                elif event.key == pygame.K_w:
                    drone.move_forward()
                elif event.key == pygame.K_s:
                    drone.move_backward()
                # left / right
                elif event.key == pygame.K_a:
                    drone.move_left()
                elif event.key == pygame.K_d:
                    drone.move_right()
                # up / down
                elif event.key == pygame.K_UP:
                    drone.move_up()
                elif event.key == pygame.K_DOWN:
                    drone.move_down()
                # turn left / turn right
                elif event.key == pygame.K_LEFT:
                    drone.turn_left()
                elif event.key == pygame.K_RIGHT:
                    drone.turn_right()
                elif event.key == pygame.K_p:
                    psi_i = drone.navdata['psi']
                    while (drone.navdata['psi']-psi_i) % 360 < 90:
                        drone.turn_right()

        if testing:
            ret, frame = capture.read()
            frame = flip(frame, 1)
        else:
            frame = drone.get_image()
            frame = cvtColor(frame, COLOR_RGB2BGR)
        if frame != None:
            processed_frame = cvtColor(frame, COLOR_BGR2HSV)
            processed_frame = inRange(
              processed_frame,
              (HUE_LOWERB, SAT_LOWERB, BRI_LOWERB),
              (HUE_UPPERB, SAT_UPPERB, BRI_UPPERB)
            )
            kernel = np.ones((5, 5))
            processed_frame = GaussianBlur(processed_frame, (5, 5), 0, 0)
            processed_frame = erode(processed_frame, kernel, iterations=EROSION)
            processed_frame = dilate(processed_frame, kernel, iterations=DILATION)
            imshow("threshold", processed_frame)
            contours = findContours(
              processed_frame,
              RETR_EXTERNAL,
              CHAIN_APPROX_SIMPLE,
              (0, 0)
            )[0]

            center = (frame.shape[1]/2, frame.shape[0]/2)
            hat = (center[0]-1, center[1]-1, 2, 2)
            hat_distance = 90000
            for contour in contours:
                (x, y, w, h) = boundingRect(contour)
                distance = np.linalg.norm(np.array((x+w/2,y+h/2))-estimated)
                if distance < hat_distance:
                  hat = (x, y, w, h)
                  hat_distance = distance
            (x1, y1, w, h) = hat
            (x2, y2) = (x1 + w, y1 + h)

            k2d.update(x1+w/2, y1+h/2)
            estimated = [int(c) for c in k2d.getEstimate()]
            tracking_center = estimated

            circle(frame, (estimated[0], estimated[1]), 4, (0, 255, 0))
            rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0))

            action = ""
            if not (testing or control):
                drone.speed = DRONE_SPEED
                if (center[1] + VFUZZ < tracking_center[1]):
                    action += "move down "
                    drone.move_down()
                elif (center[1] - VFUZZ > tracking_center[1]):
                    action += "move up "
                    drone.move_up()
                elif (center[0] + HFUZZ < tracking_center[0]):
                    action += "turn right "
                    drone.speed = TURN_SPEED
                    drone.turn_right()
                    drone.speed = DRONE_SPEED
                elif (center[0] - HFUZZ > tracking_center[0]):
                    action += "turn left "
                    drone.speed = TURN_SPEED
                    drone.turn_left()
                    drone.speed = DRONE_SPEED
                elif w < FOLLOW_WIDTH and w > 25:
                    action += "move forward "
                    drone.move_forward()
                elif w > 2*FOLLOW_WIDTH:
                    action += "move backward "
                    drone.move_backward()
                elif w > 25:
                    action += "circle"
                    drone.speed = CIRCLE_SPEED
                    drone.move_right()
                    drone.speed = DRONE_SPEED
                else:
                    action += "hover"
                    drone.turn_left()
                putText(frame, action, (0, 20),
                        FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                try:
                  putText(frame, str(drone.navdata.get(0, dict())["battery"]), (0, 50),
                          FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                except KeyError:
                  pass
            imshow("output", frame)
        else:
            print "frame is none"

    print("Shutting down...")
    drone.halt()
    print("Ok.")

if __name__ == '__main__':
    main()
