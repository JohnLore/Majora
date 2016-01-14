import pygame

from libardrone import libardrone

from cv2 import *
import numpy as np

import urllib2
import sys
import time
from __builtin__ import max,min

GREENHUE_LOWERB = 40
GREENHUE_UPPERB = 100

def main():
    testing=False
    if len(sys.argv) > 1 and sys.argv[1] == 'testing':
        testing = True
    pygame.init()
    print "initialized"
    drone = libardrone.ARDrone()
    drone.reset()

    navdata_0 = drone.navdata.copy()
    tracking_center = False

    clock = pygame.time.Clock()
    namedWindow("output", 1)

    if testing:
        capture = VideoCapture(0)
        while (not capture.isOpened()):
            print "not opened"

    running = True
    while running:
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
                    print("return")
                    drone.takeoff()
                elif event.key == pygame.K_SPACE:
                    print("space")
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
            frame = drone.image
        if frame != None:
            processed_frame = cvtColor(frame, COLOR_BGR2HSV)
            processed_frame = inRange(
              processed_frame,
              (GREENHUE_LOWERB, 0, 0),
              (GREENHUE_UPPERB, 255, 255)
            )
            kernel = np.ones((5, 5))
            processed_frame = erode(processed_frame, kernel, iterations=2)
            processed_frame = dilate(processed_frame, kernel, iterations=2)
            contours = findContours(
              processed_frame,
              RETR_EXTERNAL,
              CHAIN_APPROX_SIMPLE,
              (0, 0)
            )[0]

            hat = None
            for contour in contours:
                if hat is None:
                    hat = contour
                if contourArea(contour) > contourArea(hat):
                    hat = contour
            if hat is None:
                continue
            (x1, y1, w, h) = boundingRect(hat)
            (x2, y2) = (x1 + w, y1 + h)

            tracking_center = ((x1+x2)/2, (y1+y2)/2)
            rectangle(frame, (x1, y1), (x2, y2), 0xFF0000)

            center = (frame.shape[1]/2, frame.shape[0]/2)
            print center, tracking_center
            FUZZ = 100
            if (center[1] + FUZZ < tracking_center[1]):
                print "move forward"
                drone.move_forward()
            elif (center[1] - FUZZ > tracking_center[1]):
                print "move backward"
                drone.move_backward()
            if (center[0] + w/2 + FUZZ < tracking_center[0]):
                print "move left"
                drone.turn_left()
            elif (center[0] > tracking_center[0]):
                print "move right"
                drone.turn_right()
            imshow("output", frame)
        else:
            print "frame is none"

    print("Shutting down...")
    drone.halt()
    print("Ok.")

if __name__ == '__main__':
    main()
