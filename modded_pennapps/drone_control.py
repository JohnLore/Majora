import pygame

from libardrone import libardrone

from cv2 import *
import numpy as np

from kalman2d import Kalman2D

import urllib2
import sys
import time
from __builtin__ import max,min

def report_hue_lower_change(x):
  print "hue lower changed to %d" % x

def report_hue_upper_change(x):
  print "hue upper changed to %d" % x

def report_sat_lower_change(x):
  print "sat lower changed to %d" % x

def report_sat_upper_change(x):
  print "sat upper changed to %d" % x

def report_bri_lower_change(x):
  print "bri lower changed to %d" % x

def report_bri_upper_change(x):
  print "bri upper changed to %d" % x

def report_erosion_change(x):
  print "erosion changed to %d" % x

def report_dilation_change(x):
  print "dilation changed to %d" % x

def main():
    HUE_LOWERB = 50
    HUE_UPPERB = 100
    SAT_LOWERB = 40
    SAT_UPPERB = 200
    BRI_LOWERB = 5
    BRI_UPPERB = 200
    EROSION = 2
    DILATION = 4
    testing=False

    if len(sys.argv) > 1 and sys.argv[1] == 'testing':
        testing = True
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

    clock = pygame.time.Clock()
    namedWindow("threshold", 1)
    namedWindow("control", WINDOW_NORMAL)
    resizeWindow("control", 1024, 300)
    moveWindow("control", 0, 1000)
    namedWindow("output", 1)
    createTrackbar("hue lower", "control", HUE_LOWERB, 255, report_hue_lower_change)
    createTrackbar("hue upper", "control", HUE_UPPERB, 255, report_hue_upper_change)
    createTrackbar("sat lower", "control", SAT_LOWERB, 255, report_sat_lower_change)
    createTrackbar("sat upper", "control", SAT_UPPERB, 255, report_sat_upper_change)
    createTrackbar("bri lower", "control", BRI_LOWERB, 255, report_bri_lower_change)
    createTrackbar("bri upper", "control", BRI_UPPERB, 255, report_bri_upper_change)
    createTrackbar("erosion", "control", EROSION, 10, report_erosion_change)
    createTrackbar("dilation", "control", DILATION, 10, report_dilation_change)

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
                elif event.key == pygame.K_1:
                    drone.speed = 0.1
                elif event.key == pygame.K_2:
                    drone.speed = 0.2
                elif event.key == pygame.K_3:
                    drone.speed = 0.3
                elif event.key == pygame.K_4:
                    drone.speed = 0.4
                elif event.key == pygame.K_5:
                    drone.speed = 0.5
                elif event.key == pygame.K_6:
                    drone.speed = 0.6
                elif event.key == pygame.K_7:
                    drone.speed = 0.7
                elif event.key == pygame.K_8:
                    drone.speed = 0.8
                elif event.key == pygame.K_9:
                    drone.speed = 0.9
                elif event.key == pygame.K_0:
                    drone.speed = 1.0

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
            processed_frame = erode(processed_frame, kernel, iterations=EROSION)
            processed_frame = dilate(processed_frame, kernel, iterations=DILATION)
            #processed_frame = GaussianBlur(processed_frame, (5, 5), 0.5, 0.5)
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

            circle(frame, (estimated[0], estimated[1]), 4, 1234)
            rectangle(frame, (x1, y1), (x2, y2), 0xFF0000)

            if not testing:
                FUZZ = 100
                if (center[1] + FUZZ < tracking_center[1]):
                    #print "move forward"
                    #drone.move_forward()
                    pass
                elif (center[1] - FUZZ > tracking_center[1]):
                    #print "move backward"
                    #drone.move_backward()
                    pass
                if (center[0] + FUZZ < tracking_center[0]):
                    print "move right"
                    drone.turn_right()
                    pass
                elif (center[0] - FUZZ > tracking_center[0]):
                    print "move left"
                    drone.turn_left()
                    pass
            imshow("output", frame)
        else:
            print "frame is none"

    print("Shutting down...")
    drone.halt()
    print("Ok.")

if __name__ == '__main__':
    main()
