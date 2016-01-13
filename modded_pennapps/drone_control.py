import pygame
import pygame.surfarray
import pygame.transform

from libardrone import libardrone
#import Armband
#import myo
#from myo.lowlevel import pose_t

from cv2 import *
from kalman2d import Kalman2D
import numpy as np

import urllib2
import sys
import time
from __builtin__ import max,min

CASCADE_FILE="/usr/local/share/OpenCV/haarcascades/haarcascade_mcs_upperbody.xml"

def main():
    testing=False
    if len(sys.argv) > 1 and sys.argv[1] == 'testing':
        testing = True
    pygame.init()
    drone = libardrone.ARDrone(True)
    drone.reset()
    navdata_0 = drone.navdata.copy()
    calibrated = False
    control_start = False
    controlled = False

    tracking_start = False
    tracking = False
    tracking_center = False

    k2d = Kalman2D(1, 0.0001, 0.1)
    estimated = False

 #   armband = Armband.Armband()
 #   yaw_0 = armband.collector.yaw_w
 #   roll_0 = armband.collector.roll_w
 #   pitch_0 = armband.collector.pitch_w

    clock = pygame.time.Clock()
    namedWindow("output", 1)
    person_cascade = CascadeClassifier(CASCADE_FILE)
    
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
                elif event.key == pygame.K_t:
                    tracking = not tracking
   #             elif event.key == pygame.K_c:
    #                yaw_0 = armband.collector.yaw_w
     #               roll_0 = armband.collector.roll_w
      #              pitch_0 = armband.collector.pitch_w
       #             calibrated = True
        #            print "calibrated!"
                # speed
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
        # MUST WEAR armband with light to outside

        if (not calibrated):
            frame = drone.get_image()
            imshow("output", frame)
            continue

 #       yaw_d = armband.collector.yaw_w - yaw_0
 #       pitch_d = armband.collector.pitch_w - pitch_0
 #       roll_d = armband.collector.roll_w - roll_0
 #       if (roll_d > 270): roll_d = roll_d - 360

 #       if (not controlled and not tracking and pitch_d > 40 and roll_d > 60):
 #           tracking_start = True
 #           armband.collector.myo.vibrate('short')
 #       if (tracking_start and not (pitch_d > 40 or roll_d > 60)):
 #           tracking_start = False
 #       if (tracking_start and armband.collector.current_pose == pose_t.fist):
 #           tracking_start = False
 #           tracking = True
 #           armband.collector.myo.vibrate('long')

 #       if (not controlled and roll_d > 60 and not (pitch_d > 10 or pitch_d < -10)):
 #           control_start = True
 #           armband.collector.myo.vibrate('short')
 #       if (control_start and (pitch_d > 10 or pitch_d < -10)):
 #           control_start = False
 #       if (control_start and roll_d < 10):
 #           yaw_0 = armband.collector.yaw_w
 #           roll_0 = armband.collector.roll_w
 #           controlled = True
 #           control_start = False
 #           tracking = False
 #           armband.collector.myo.vibrate('long')
        #print yaw_d, pitch_d, roll_d
 #       if (controlled):
 #           if (yaw_d < -20):
 #               print "turn right"
 #               drone.turn_right()
 #           if (yaw_d > 20):
 #               print "turn left"
 #               drone.turn_left()
 #           if (pitch_d < -15):
 #               print "move backward"
 #               drone.move_backward()
 #           if (pitch_d > 20):
 #               print "move forward"
 #               drone.move_forward()
 #           if (roll_d > 30):
 #               print "move right"
 #               drone.move_right()
 #           if (roll_d < -30):
 #               print "move left"
 #               drone.move_left()
 #           if (pitch_d > 30):
 #               controlled = False
 #           if (armband.collector.current_pose == pose_t.double_tap):
 #               print "shoot"
 #               urllib2.urlopen("http://droneduino.local/arduino/shoot/lj")
 #               armband.collector.current_pose = pose_t.rest
 #           if (armband.collector.current_pose == pose_t.fingers_spread):
 #               print "rise"
 #               armband.collector.current_pose = pose_t.rest
 #          if (armband.collector.current_pose == pose_t.fist):
 #               print "descend"
 #               armband.collector.current_pose = pose_t.rest
            frame = drone.get_image()
            imshow("output", frame)
            continue

        frame = drone.get_image()
        if testing:
            ret, frame = capture.read()
            frame = flip(frame, 1)
        if frame != None:
            if not estimated:
                estimated = np.array((frame.shape[1], frame.shape[0]))
            b,g,gray = split(frame)
            people = person_cascade.detectMultiScale(gray, 1.05, 3, 0, (150, 150))
            best_candidate = (0, 0, 0, 0)
            best_distance = 9000 
            for (x, y, w, h) in people:
                rectangle(frame, (x, y), (x+w, y+h), 0)
                distance = np.linalg.norm(np.array((x+w/2,y+h/2))-estimated)
                if distance < best_distance:
                    best_candidate = (x, y, w, h)
                    best_distance = distance
            (x, y, w, h) = best_candidate
            k2d.update(x+w/2, y+h/2)
            estimated = [int(c) for c in k2d.getEstimate()]
            circle(frame, (estimated[0], estimated[1]), 4, 1234)
            rectangle(frame, (x, y), (x+w, y+h), 1234)
            if tracking:
                if not tracking_center:
                    tracking_center = [x+w/2, y+h/2]
                circle(frame, (tracking_center[0], tracking_center[1]), 15, 0x00FF00)
                circle(frame, (x+w/2, y+w/2), 10, 0xFF0000)
                print tracking_center, x+w/2, y+h/2,  
                #if (y + h/2 < frame.shape[0]/2):
                if False:
                    print "unable to track"
                else:
                    FUZZ = 100
                    if (y + h/2 + FUZZ < tracking_center[1]):
                        print "move forward"
                        drone.move_forward()
                    elif (y + h/2 - FUZZ > tracking_center[1]):
                        print "move backward"
                        drone.move_backward()
                    if (x + w/2 + FUZZ < tracking_center[0]):
                        print "move left"
                        drone.turn_left()
                    elif (x + w/2 - FUZZ > tracking_center[0]):
                        print "move right"
                        drone.turn_right()
            imshow("output", frame)
        else:
            print "frame is none"
        #t = getTickCount() - t
        #print "detection time = %gms" % (t/(getTickFrequency()*1000.))

    print("Shutting down...")
    drone.halt()
    print("Ok.")

if __name__ == '__main__':
    main()
