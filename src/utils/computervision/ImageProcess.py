# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import socket
import struct
import time
import numpy as np
import json
import math

from threading       import Thread
from threading       import Timer
from multiprocessing import Pipe
from multiprocessing import Process
from multiprocessing import Queue

from sys import getsizeof


import cv2
import matplotlib.pyplot as plt

from src.templates.workerprocess import WorkerProcess
from src.utils.control.RcBrainThread                import RcBrainThread

from src.utils.finitestatemachine.FiniteStateMachine import FiniteStateMachine

from gpiozero import Servo
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory

Device.pin_factory = PiGPIOFactory('127.0.0.1')

#from pynput import keyboard 
#from src.utils.tflite import ObjectDetector







# slow down when sign is detected from far away









def nothing(x):
    pass

class ImageProcess(WorkerProcess):
    
    # ===================================== INIT =========================================
    def __init__(self, inPipes, outPipes):
        """Process used for sending images over the network to a targeted IP via UDP protocol 
        (no feedback required). The image is compressed before sending it. 

        Used for visualizing your raspicam images on remote PC.
        
        Parameters
        ----------
        inPs : list(Pipe) 
            List of input pipes, only the first pipe is used to transfer the captured frames. 
        outPs : list(Pipe) 
            List of output pipes (not used at the moment)
        """
        super(ImageProcess,self).__init__(inPipes, outPipes)
        self.HEIGHT = 300
        self.WIDTH = 300
        self.inPs = inPipes[0]
        self.inDetectedPs = inPipes[1]
        self.outPs = outPipes[0]
        self.outFrame = outPipes[1]
        self.curr_steer_angle = 0.0

        self.rcBrain = RcBrainThread()
        self.servo = Servo(12)
        self.fStateMachine = FiniteStateMachine()
        self.confThreshold = 0.5
        self.state = 'lane_keeping'
        self.previousState = 'lane_keeping'
        self.turns = [] # the set path for the intersection turns
        self.current_turn_index = 0
        self.detected = []

        # self.net = cv2.dnn.readNet('/home/pi/repos/Brain/src/utils/openvino/ssd_mobilenet/bosch_model_0/saved_model.xml', '/home/pi/repos/Brain/src/utils/openvino/ssd_mobilenet/bosch_model_0/saved_model.bin')
        # self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)
        
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        #self._init_socket()
        super(ImageProcess,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the sending thread.
        """
        
        if self._blocker.is_set():
            return
        #self.listener.daemon = self.daemon
        #self.threads.append(self.listener)
        
        streamTh = Thread(name='ProcessImageThread',target = self._process_image, args= (self.inPs, self.inDetectedPs, self.outPs, self.outFrame))
        streamTh.daemon = True
        self.threads.append(streamTh)

    

    # centers car
    # set state to lane centering
    # 
    def _restart_car(self, outPs):
        self._send_command(outPs, ['forward_normal'])
        self.fStateMachine.force_restart()

        

    
    # ===================================== SEND THREAD ==================================
    def _process_image(self, inP, inDetections, outPs, outFrame):
        """Sending the frames received thought the input pipe to remote client by using the created socket connection. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe to read the frames from CameraProcess or CameraSpooferProcess. 
        """        
        
        cmds = ['pid', 'stop']
        self._send_command(outPs, cmds)

        timer1 = time.time()
        while True:
            timer2 = time.time()
            if timer2 - timer1 > 3:
                break

        # self._send_command(outPs, ['forward_normal'])
        
        # self._send_command(outPs, ['forward_normal'])
        

        stencil_no_gap = np.zeros((self.HEIGHT, self.WIDTH))
        stencil_no_gap = stencil_no_gap.astype('uint8')
        
        stencil_gap = np.zeros((self.HEIGHT, self.WIDTH))
        stencil_gap = stencil_gap.astype('uint8')

        stencil_gap_prime = np.zeros((self.HEIGHT, self.WIDTH))
        stencil_gap_prime = stencil_gap_prime.astype('uint8')

        
        polygon1 = np.array([[0, 300], [0,150], [90,150], [90, 300]])
        polygon2 = np.array([[210,300], [210, 150], [300,150], [300, 300]])
        cv2.fillPoly(stencil_gap, [polygon1, polygon2], 1)

        polygon3 = np.array([[90, 300], [90, 150], [180, 150], [180, 300]])
        polygon4 = np.array([[210,300], [210, 150], [300,150], [300, 300]])
        cv2.fillPoly(stencil_gap_prime, [polygon3, polygon4], 1)
        
        
        polygon = np.array([[0, 300], [0,150], [300,150], [300, 300]])
        cv2.fillConvexPoly(stencil_no_gap, polygon, 1)
        
        
        stencil = stencil_gap
        stencil_prime = stencil_gap_prime
        
        winname = 'RebelDynamics'
        cv2.namedWindow(winname)
        


        cv2.createTrackbar('Thresh0', winname, 0, 255, nothing)
        cv2.createTrackbar('Thresh1', winname, 0, 255, nothing)
        cv2.createTrackbar('HoughLines', winname, 0, 255, nothing)
        cv2.createTrackbar('HoughGap', winname, 1, 255, nothing)
        
        thresh0 = thresh1 = hough0 = hough1 =  1
        Pthresh0 = Pthresh1 = Phough0 = Phough1 = 1



        ###########################################
        # For object detection
        ###########################################
        
        # inputQueue = Queue(maxsize=1)
        # outputQueue = Queue(maxsize=1)
        # detectionOut = None
        # print("[INFO] starting process...")
        # p = Process(target=self.classify_frame, args=(self.net,inputQueue,outputQueue,))
        # p.daemon = True
        # p.start()
        
        ###########################################

        self.fStateMachine.force_restart()


        timer1 = time.time()

        # self._send_command(outPs, ['forward_normal'])
        # self._enter_roundabout(outPs)

        while True:
            try:

                timer2 = time.time()
                # get image
                stamps, rawImage = inP.recv()
                # resize image to 300x300
                image = cv2.resize(rawImage, (300, 300))


                outFrame.send(image)
                
                '''
                    Testing reading image from file
                    Need to provide path to file
                '''
                #image = cv2.imread('path/to/file/...')
                #image = cv2.resize(rawImage, (300, 300))

                '''
                    start object detection
                '''
                self.detected = []
                detections = inDetections.recv()
                print('test0')
                if detections is not None:
                    for detection in detections:
                        objID = detection[0]
                        confidence = detection[1]
                        xmin = detection[2]
                        ymin = detection[3]
                        xmax = detection[4]
                        ymax = detection[5]
                        
                        if confidence >= self.confThreshold:
                            self.detected.append(detection)
                        
                        # car found
                        if objID == 0:
                            # Need to estimate where car is (look for bottom)
                            # self._overtake(outPs)
                            print('found car')
                '''
                    end object detection
                '''



                '''
                    Lane keeping
                '''
                if self.state == 'lane_keeping':
                    print('test1')
                    # self._lane_keeping(image)
                    # convert to rgb
                    rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    # convert to grayscale
                    #gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    # crop with mask
                    #img_crop_gray = cv2.bitwise_and(gray_img, gray_img, mask=stencil)
                    # img_crop = cv2.bitwise_and(image, image, mask=stencil)
                    # convert to grayscale
                    # img_crop_gray = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
                    img_crop_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    # blur
                    #blur_img = cv2.blur(img_crop_gray, (10,10))
                    blur_img = cv2.GaussianBlur(img_crop_gray, (5,5), 0)
                    # get threshold
                    # (199, 170) for plastic ground
                    ## Test track
                    # ret, thresh = cv2.threshold(blur_img, 228, 169, cv2.THRESH_BINARY)
                    ## Final Track (afternoon)
                    # ret, thresh = cv2.threshold(blur_img, 180, 158, cv2.THRESH_BINARY)
                    '''
                        Commented line is used to adjust parameters during testing!
                    '''
                    ret, thresh = cv2.threshold(blur_img, thresh0, thresh1, cv2.THRESH_BINARY)


                    '''
                        Looking for stopline (intersection!)
                    '''

                    # if self._find_stopline(thresh):
                    #     self.state = 'at_intersection'
                    #     continue




                    # get edges using Canny algorithm
                    edges = cv2.Canny(image=thresh, threshold1=100, threshold2=200)
                    
                    # Crop image for lane detection
                    cropped_lane_detection = cv2.bitwise_and(edges, edges, mask=stencil)

                    # Crop image for stop-line detection
                    cropped_stop_line = cv2.bitwise_and(edges, edges, mask=stencil_prime)

                    # get lines for lane detection
                    lane_detection_lines = cv2.HoughLinesP(cropped_lane_detection, 1, np.pi/180, 25, maxLineGap=200)
                    '''
                        Commented line is used to adjust parameters during testing!
                    '''
                    # lines = cv2.HoughLinesP(edges, 1, np.pi/180, hough1, maxLineGap=200)
                    

                    # get lines for lane detection
                    # stop_line_detections = cv2.HoughLinesP(lane_detection_lines, 1, np.pi/180, 25, maxLineGap=10)

                    # if lines is not None:
                    #     for jj in range(0, len(lines)):
                    #         ll = lines[jj][0]
                    #         cv2.line(rgb_img, (ll[0], ll[1]), (ll[2], ll[3]), (0,0,255), 3, cv2.LINE_AA)
                    
                    # convert to rgb
                    #rgb_img = cv2.cvtColor(img_crop, cv2.COLOR_BGR2RGB) 
                    
                    # get lane lines
                    lane_lines = self._avg_slope_intersect(lane_detection_lines)
                    
                    #frame_objects = rgb_img
                    
                    # draw lines to grayscale image
                    #lane_lines_img, lane_centering_cmds = self._display_lines(frame_objects, lane_lines)
                    #lane_lines_img, steering_angle, num_lines = self._display_lines(img_crop, lane_lines)
                    lane_lines_img, steering_angle, num_lines, stopLine = self._display_lines(rgb_img, lane_lines)
                    
                    # self.curr_steer_angle = self.stabilize_steering_angle(self.curr_steer_angle, steering_angle, num_lines, )
                    # self._change_steering(steering_angle)



                    cv2.imshow(winname, thresh)
                    cv2.waitKey(1)

                    thresh0 = cv2.getTrackbarPos('Thresh0', 'RebelDynamics')
                    thresh1 = cv2.getTrackbarPos('Thresh1', 'RebelDynamics')
                    hough0 = cv2.getTrackbarPos('HoughGap', 'RebelDynamics')
                    hough1 = cv2.getTrackbarPos('HoughLines', 'RebelDynamics')

                    if (thresh0 != Pthresh0) | (thresh1 != Pthresh1) | (hough1 != Phough1) | (hough0 != Phough0):
                        print("(thresh0 = %d , thresh1 = %d, hough1 = %d, hough0 = %d)" % (thresh0, thresh1, hough1, hough0))
                        Pthresh0 = thresh0
                        Pthresh1 = thresh1
                        Phough0 = hough0
                        Phough1 = hough1
                    '''
                        end lanekeeping
                    '''
                # elif self.state == 'at_intersection':
                #     print('at intersection')
                #     cmds = ['stop']
                #     self._send_command(outPs, cmds)


                cv2.imshow(winname, thresh)
                cv2.waitKey(1)

                thresh0 = cv2.getTrackbarPos('Thresh0', 'RebelDynamics')
                thresh1 = cv2.getTrackbarPos('Thresh1', 'RebelDynamics')
                hough0 = cv2.getTrackbarPos('HoughGap', 'RebelDynamics')
                hough1 = cv2.getTrackbarPos('HoughLines', 'RebelDynamics')

                if (thresh0 != Pthresh0) | (thresh1 != Pthresh1) | (hough1 != Phough1) | (hough0 != Phough0):
                    print("(thresh0 = %d , thresh1 = %d, hough1 = %d, hough0 = %d)" % (thresh0, thresh1, hough1, hough0))
                    Pthresh0 = thresh0
                    Pthresh1 = thresh1
                    Phough0 = hough0
                    Phough1 = hough1



                # print(self.curr_steer_angle, stopLine)
                # cv2.imshow(winname, rgb_img)
                # cv2.imshow(winname, thresh)
                #cv2.imshow(winname, edges)
                # cv2.waitKey(1)
                # print('image')


                passed_time = timer2 - timer1
                
                                                
            except Exception as e:
                print("CameraStreamerProcess failed to stream images:",e,"\n")
                # Reinitialize the socket for reconnecting to client.  
                #self.connection = None
                #self._init_socket()
                pass


    def _lane_keeping(self, image):

        winname = 'RebelDynamics'
        cv2.namedWindow(winname)
        #cv2.moveWindow(winname, 0,0)
        


        # cv2.createTrackbar('Thresh0', 'RebelDynamics', 0, 255, nothing)
        # cv2.createTrackbar('Thresh1', 'RebelDynamics', 0, 255, nothing)
        # cv2.createTrackbar('HoughLines', 'RebelDynamics', 0, 255, nothing)
        # cv2.createTrackbar('HoughGap', 'RebelDynamics', 1, 255, nothing)


        # thresh0 = thresh1 = hough0 = hough1 =  1
        # Pthresh0 = Pthresh1 = Phough0 = Phough1 = 1
        stencil_no_gap = np.zeros((self.HEIGHT, self.WIDTH))
        stencil_no_gap = stencil_no_gap.astype('uint8')

        # if (thresh0 != Pthresh0) | (thresh1 != Pthresh1) | (hough1 != Phough1) | (hough0 != Phough0):
        #     print("(thresh0 = %d , thresh1 = %d, hough1 = %d, hough0 = %d)" % (thresh0, thresh1, hough1, hough0))
        #     Pthresh0 = thresh0
        #     Pthresh1 = thresh1
        #     Phough0 = hough0
        #     Phough1 = hough1
        
        
        
        stencil_gap = np.zeros((self.HEIGHT, self.WIDTH))
        stencil_gap = stencil_gap.astype('uint8')

        stencil_gap_prime = np.zeros((self.HEIGHT, self.WIDTH))
        stencil_gap_prime = stencil_gap_prime.astype('uint8')

        polygon1 = np.array([[0, 300], [0,150], [90,150], [90, 300]])
        polygon2 = np.array([[210,300], [210, 150], [300,150], [300, 300]])
        cv2.fillPoly(stencil_gap, [polygon1, polygon2], 1)

        polygon3 = np.array([[90, 300], [90, 150], [180, 150], [180, 300]])
        polygon4 = np.array([[210,300], [210, 150], [300,150], [300, 300]])
        cv2.fillPoly(stencil_gap_prime, [polygon3, polygon4], 1)
        
        polygon = np.array([[0, 300], [0,150], [300,150], [300, 300]])
        cv2.fillConvexPoly(stencil_no_gap, polygon, 1)
                
        stencil = stencil_gap
        stencil_prime = stencil_gap_prime

        # convert to rgb
        rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # convert to grayscale
        #gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # crop with mask
        #img_crop_gray = cv2.bitwise_and(gray_img, gray_img, mask=stencil)
        # img_crop = cv2.bitwise_and(image, image, mask=stencil)
        # convert to grayscale
        # img_crop_gray = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
        img_crop_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # blur
        #blur_img = cv2.blur(img_crop_gray, (10,10))
        blur_img = cv2.GaussianBlur(img_crop_gray, (5,5), 0)
        # get threshold
        # (199, 170) for plastic ground
        ## Test track
        # ret, thresh = cv2.threshold(blur_img, 228, 169, cv2.THRESH_BINARY)
        ## Final Track (afternoon)
        ret, thresh = cv2.threshold(blur_img, 180, 158, cv2.THRESH_BINARY)
        '''
            Commented line is used to adjust parameters during testing!
        '''
        # ret, thresh = cv2.threshold(blur_img, thresh0, thresh1, cv2.THRESH_BINARY)


        '''
            Looking for stopline (intersection!)
        '''

        if self._find_stopline(thresh):
            self.state = 'at_intersection'

            return

        # get edges using Canny algorithm
        edges = cv2.Canny(image=thresh, threshold1=100, threshold2=200)
        
        # Crop image for lane detection
        cropped_lane_detection = cv2.bitwise_and(edges, edges, mask=stencil)

        # Crop image for stop-line detection
        cropped_stop_line = cv2.bitwise_and(edges, edges, mask=stencil_prime)

        # get lines for lane detection
        lane_detection_lines = cv2.HoughLinesP(cropped_lane_detection, 1, np.pi/180, 25, maxLineGap=200)
        '''
            Commented line is used to adjust parameters during testing!
        '''
        # lines = cv2.HoughLinesP(edges, 1, np.pi/180, hough1, maxLineGap=200)
        

        # get lines for lane detection
        # stop_line_detections = cv2.HoughLinesP(lane_detection_lines, 1, np.pi/180, 25, maxLineGap=10)

        # if lines is not None:
        #     for jj in range(0, len(lines)):
        #         ll = lines[jj][0]
        #         cv2.line(rgb_img, (ll[0], ll[1]), (ll[2], ll[3]), (0,0,255), 3, cv2.LINE_AA)
        
        # convert to rgb
        #rgb_img = cv2.cvtColor(img_crop, cv2.COLOR_BGR2RGB) 
        
        # get lane lines
        lane_lines = self._avg_slope_intersect(lane_detection_lines)
        
        #frame_objects = rgb_img
        
        # draw lines to grayscale image
        #lane_lines_img, lane_centering_cmds = self._display_lines(frame_objects, lane_lines)
        #lane_lines_img, steering_angle, num_lines = self._display_lines(img_crop, lane_lines)
        lane_lines_img, steering_angle, num_lines, stopLine = self._display_lines(rgb_img, lane_lines)
        
        # self.curr_steer_angle = self.stabilize_steering_angle(self.curr_steer_angle, steering_angle, num_lines, )
        # self._change_steering(steering_angle)

        # cv2.imshow(winname, thresh)
        # cv2.waitKey(1)


    # Function for car to make a right turn at an intersection
    def _right_turn(self):
        print('right turn')

         # from stop line to right turn:
        #     go forward 0.5 seconds
        #     make right turn (0.9) for 11.5 seconds
        #     go straight
        
        timer1 = time.time()
        flag = True
        while flag:
            timer2 = time.time()
            passed_time = timer2 - timer1

            if passed_time > 0.1 and steerFlag == 0:
                self._test_steering(0.9)
                steerFlag = 1

            if timer2 - timer1 > 12 and steerFlag == 1:
                self._test_steering(0.0)
                steerFlag = 2
                flag = False
        self.state = 'lane_keeping'
        

    # Function for car to make a left turn at an intersection
    def _left_turn(self):
        print('left turn')

        timer1 = time.time()
        flag = True
        while flag:
            timer2 = time.time()
            passed_time = timer2 - timer1

            if passed_time > 5 and steerFlag == 0:
                self._test_steering(-0.75)
                steerFlag = 1

            if timer2 - timer1 > 14 and steerFlag == 1:
                self._test_steering(0.0)
                steerFlag = 2
                flag = False
        self.state = 'lane_keeping'
    

    # Function for car to go straight ahead at an intersection
    def _straight_ahead(self):
        print('straight ahead')
        timer1 = time.time()
        flag = True
        while flag:
            timer2 = time.time()
            passed_time = timer2 - timer1

            if passed_time >= 0 and steerFlag == 0:
                self._test_steering(0.1)
                steerFlag = 1

            if timer2 - timer1 > 11 and steerFlag == 1:
                steerFlag = 2
                flag = False
        self.state = 'lane_keeping'


    # Function for car to look for a stopline
    def _find_stopline(self, thresh):
        # expecting frame to be:
        #   grayscale -> threshold

        y = 140
        h = 140
        x = 0
        w = 300

        img_crop = thresh[y:y+h, x:x+w]

        kernel_horizontal = np.ones((1, 25), dtype=np.uint8)
        erode = cv2.erode(img_crop, kernel_horizontal)

        # edges = cv2.Canny(image=erode, threshold1=100, threshold2=200)    
        minLineLength = 150
        maxLineGap = 5
        lines = cv2.HoughLinesP(erode,1,np.pi/180,25,minLineLength,maxLineGap)
        if lines is not None:
            sum = 0
            count = 0
            for x1,y1,x2,y2 in lines[0]:
                sum = sum +  y1 + y2
                count = count + 2
            avg = sum / count
            if avg >= 160:
            #     cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
                print('found stopline')
                return True

    # Function for car to look for crosswalk lines 
    def _find_crosswalk(self):
        print('found crosswalk')
        # not sure if needed...

    # Classifying traffic light color (red or green)
    # try finding average or convert to grayscale and get intensity?
    def _traffic_light_classification(self):
        print('classifying traffic light color')
        # get bounding box of traffic light
        # break 

    def _intersection_action(self):
        print('doing intersection')
        # is stopped
        # rotate servo left to right
        #   put primary focus on right for detection of signs

    def _enter_roundabout(self, outPs):
        print('entering roundabout')
        # stop at stop-line
        # look at sign
        # go inside and keep right turn
        # take second exit by turning right and then do lane centering


        go_cmds = ['forward_normal']
        cmds = ['stop']

        self._send_command(outPs, go_cmds)
        
        timer1 = time.time()
        flag = True
        self._test_steering(0.1)
        steerFlag = 0
        while flag:
            timer2 = time.time()
            passed_time = timer2 - timer1

            if passed_time > 4 and steerFlag == 0:
                self._test_steering(0.75)
                steerFlag = 1

            if timer2 - timer1 > 9 and steerFlag == 1:
                self._test_steering(-0.75)
                steerFlag = 2

            if timer2 - timer1 > 17.5 and steerFlag == 2:
                self._test_steering(0.75)
                steerFlag = 3

            if timer2 - timer1 > 20 and steerFlag == 3:
                self._test_steering(0.1)
                steerFlag = 3
                flag = False
                self._send_command(outPs, cmds)


    def _overtake(self, outPs):
        print('overtaking car')
        # wait for my car to approach the car
        # look at bounding box bottom line
        # if close enough, do manuever (on dotted line)

        cmds = ['stop']
        
        timer1 = time.time()
        flag = True
        self._test_steering(-0.75)
        steerFlag = 0
        while flag:
            timer2 = time.time()
            passed_time = timer2 - timer1

            if passed_time > 2.5 and steerFlag == 0:
                self._test_steering(0.1)
                steerFlag = 1

            if timer2 - timer1 > 8 and steerFlag == 1:
                self._test_steering(0.75)
                steerFlag = 2

            if timer2 - timer1 > 15 and steerFlag == 2:
                self._test_steering(-0.75)
                steerFlag = 3
            
            if timer2 - timer1 > 18 and steerFlag == 3:
                self._test_steering(0.1)
                flag = False
                self._send_command(outPs, cmds)

        self.state = 'lane_keeping'

    def _wait_pedestrian_cross(self):
        print('waiting for pedestrian to cross')
        # slow down at pedestrian sign
        # if pedestrian is spotted, stop
        # track where its going
        # take note where it came from (left or right)
        # turn camera to left/right and see if pedestrian stops moving


    def _parallel_park(self, outPs):
        print('parallel parking')
        # detected parking sign
        # slow down
        # approach first parking slot
        #   look right
        #   see if spot is empty
        #   if so, engage parking
        #   if not, approach second spot (using lane centering)
        #       engage parking

        stop_cmd = ['stop']
        reverse = ['reverse']
        forward = ['forward_slow']
        
        timer1 = time.time()
        flag = True
        self._send_command(outPs, reverse)
        self._test_steering(0.85)
        steerFlag = 0
        while flag:
            timer2 = time.time()
            passed_time = timer2 - timer1

            if passed_time > 4 and steerFlag == 0:
                self._test_steering(-0.85)
                steerFlag = 1

            if timer2 - timer1 > 8 and steerFlag == 1:
                self._test_steering(0.1)
                steerFlag = 2

            if timer2 - timer1 > 10 and steerFlag == 2:
                steerFlag = 3
                self._send_command(outPs, forward)

            if timer2 - timer1 > 12 and steerFlag == 3:
                steerFlag = 4
                flag = False
                self._send_command(outPs, stop_cmd)



    def classify_frame(self, net, inputQueue, outputQueue):
    # keep looping
        while True:
            # check to see if there is a frame in our input queue
            if not inputQueue.empty():
                # grab the frame from the input queue, resize it, and
                # construct a blob from it
                frame = inputQueue.get()
                # resframe = cv2.resize(frame, (300, 300))
                blob = cv2.dnn.blobFromImage(frame, 1, size=(300, 300), mean=(127.5,127.5,127.5), swapRB=False, crop=False)
                net.setInput(blob)
                out = net.forward()

                data_out = []

                for detection in out.reshape(-1, 7):
                    inference = []
                    obj_type = int(detection[1]-1)
                    confidence = float(detection[2])
                    xmin = int(detection[3] * frame.shape[1])
                    ymin = int(detection[4] * frame.shape[0])
                    xmax = int(detection[5] * frame.shape[1])
                    ymax = int(detection[6] * frame.shape[0])

                    if confidence > 0: #ignore garbage
                        inference.extend((obj_type,confidence,xmin,ymin,xmax,ymax))
                        data_out.append(inference)

                outputQueue.put(data_out)
        


    def _avg_slope_intersect(self, lines):
        lane_lines = []
        if lines is None:
            return lane_lines
        
        left_fit = []
        right_fit = []
        boundary = 1/2 #1/3
        left_region_boundary = self.WIDTH * (1-boundary)
        right_region_boundary = self.WIDTH * boundary
        
        
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 == x2:
                    continue
                fit = np.polyfit((x1, x2), (y1, y2,), 1)
                slope = fit[0]
                intercept = fit[1]

                angle = (np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi)
                
                if True:
                    
                    if x1 < left_region_boundary and x2 < left_region_boundary and abs(angle) % 90 > 30.0:
                        left_fit.append((slope, intercept))
                    elif x1 > right_region_boundary and x2 > right_region_boundary and abs(angle) % 90 > 30.0:
                        right_fit.append((slope, intercept))
                            
        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self._make_points(left_fit_average))
        
        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self._make_points(right_fit_average))
            
        return lane_lines
    
    

    def _make_points(self, lines):
        slope, intercept = lines
        y1 = self.HEIGHT
        y2 = int(y1 * 2/3)
        
        x1 = max(-self.WIDTH, min(2*self.WIDTH, int(y1 - intercept)/slope))
        x2 = max(-self.WIDTH, min(2*self.WIDTH, int(y2 - intercept)/slope))
        
        y1 = int(y1)
        x1 = int(x1)
        x2 = int(x2)
        
        return [[x1, y1, x2, y2]]



    def _display_lines(self, frame, lines):
        line_img = np.zeros((self.HEIGHT, self.WIDTH, 3))
        stopLine = False
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    #cv2.line(line_img, (x1, y1), (x2, y2), (0,0,255), 2)
                    
                    # Checking if stop line is found...
                    if x1 < 0 and x2 > 300 or x1 > 300 and x2 < 0:
                        # drive for 4 seconds
                        #self.CURRENT_STATE == 'found_stop_line'
                        stopLine = True
                        cv2.line(line_img, (x1, y1), (x2, y2), (255,255,0), 2)
                    else:
                        cv2.line(line_img, (x1, y1), (x2, y2), (0,0,255), 2)
        
        frame = frame.astype('uint8')
        line_img = line_img.astype('uint8')
        num_lines = 0
        
        commands = 90
        ##################
        # draw center line
        ##################
        # Detected 2 lanes
        #print(lines)
        
        mid = int(self.WIDTH / 2)
        y_offset = int(self.HEIGHT / 2)
        if len(lines) == 2:
            left_x1, _, left_x2, _ = lines[0][0]
            right_x1, _, right_x2, _ = lines[1][0]
            #print(lines[0][0])
            x_offset = (left_x2 + right_x2) / 2
            y_offset = int(self.HEIGHT *  2 / 3)
            
                        
            #check for stop line
            if right_x2 < left_x2 and right_x2 > 150 and right_x2 < 320:
               # print('one')
                x1, _, x2, _ = lines[1][0]
                x_offset = int(x2 - x1)
                #print(x_offset)
                x_offset = x_offset + mid
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (int(mid), self.HEIGHT), (0,255,0), 3)
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, y_offset,)
                num_lines = 2
            elif right_x2 < left_x2 and (right_x2 < 50 and right_x1 > mid) or (right_x2 > 320 and right_x1 < mid):
                #print('two')
                x1, _, x2, _ = lines[0][0]
                x_offset = int(x2 - x1)
                #print(x_offset)
                x_offset = x_offset + mid
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (int(mid), self.HEIGHT), (0,255,0), 3)
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, y_offset,)
                num_lines = 2

            elif abs(left_x1 - right_x1) < 50:
                #print('three')
                #mid = int(self.WIDTH / 2)
                #mid = 100
                x1, _, x2, _ = lines[0][0]
                x_offset = int(x2 - x1)
                
                x_offset = x_offset + mid
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (int(mid), self.HEIGHT), (0,255,0), 3)
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, y_offset,)
                num_lines = 2
            else:
                #print('four')
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (mid, self.HEIGHT), (0,255,0), 3)
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, y_offset)
                num_lines = 2
        ### detected one lane
        elif len(lines) == 1:
            #mid = 100
            #print('five')
            x1, _, x2, _ = lines[0][0]
            x_offset = int(x2 - x1)
            #print(x_offset)
            #x_offset = x_offset + mid
            #y_offset = int(self.HEIGHT * 2 / 3)
            y_offset = self.HEIGHT // 2
            num_lines = 1
            
            #fit = np.polyfit((x1,x2), (320, 320), 1)
            
            # draw center line
            
            
            if x1 < self.WIDTH / 2 and x_offset < self.WIDTH / 2:
                #x_offset = x_offset + mid
                x_offset = self.WIDTH - 1
            if x1 > self.WIDTH / 2 and x_offset > self.WIDTH / 2:
                #offset = x_offset - mid
                offset = 0
            
            
            #print(offset, x1, x2)
            
            #if x1 > 320 and x2 < 130:
            
            #print(x1, x2, x_offset)
            
            cv2.line(frame, (int(x_offset), int(y_offset)), (mid, self.HEIGHT), (0,255,0), 3)
            #print(self._get_slope(x_offset, y_offset, self.WIDTH/2, self.HEIGHT))
            overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
            commands = self._steering_cmd(x_offset, y_offset)
            
        # No lanes detected
        else:
            overlay_img = frame
            x_offset = 0
            commands = 90
        #cv2.line(frame, (int(x_offset), int(y_offset)), (int(self.WIDTH/2), self.HEIGHT), (0,255,0), 3)
        
        #overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
        #print(x_offset, commands)
        return overlay_img, commands, num_lines, stopLine
    


    def stabilize_steering_angle(
          self,
          curr_steering_angle, 
          new_steering_angle, 
          num_of_lane_lines, 
          max_angle_deviation_two_lines=5, 
          max_angle_deviation_one_lane=1):
        '''
        Using last steering angle to stabilize the steering angle
        if new angle is too different from current angle, 
        only turn by max_angle_deviation degrees
        '''
        if num_of_lane_lines == 2 :
            # if both lane lines detected, then we can deviate more
            max_angle_deviation = max_angle_deviation_two_lines
        else :
            # if only one lane detected, don't deviate too much
            max_angle_deviation = max_angle_deviation_one_lane
        
        angle_deviation = new_steering_angle - curr_steering_angle

        if abs(angle_deviation) > max_angle_deviation:
            stabilized_steering_angle = int(curr_steering_angle + max_angle_deviation * angle_deviation / abs(angle_deviation))
        else:
            stabilized_steering_angle = new_steering_angle
        #print(new_steering_angle, stabilized_steering_angle)
        return stabilized_steering_angle



    def _steering_cmd(self, x_offset, y_offset):
        return int(math.atan(x_offset / y_offset) * 180.0 / math.pi) + 90

        '''
        print(x1)
        if x1 <= 200 and x1 >= 140:
            #return ['forward', 'left', 'left']
            return ['forward', 'leftleft']
        elif x1 < 140:
            return ['forward', 'leftleft']
        elif x1 >= 340 and x1 <= 400:
            #return ['forward', 'right', 'right']
            return ['forward', 'rightright']
        elif x1 > 400:
            return ['forward', 'rightright']
        else:
            return ['forward', 'straight']
        '''



    def _test_steering(self, angle):
        print(angle)
        self.servo.value = angle


    def _change_steering(self, angle):
        # angle values:
        #       0-89:   turn left
        #       90:     center
        #       90-180: turn right
        # max_angle = 0.75
        max_angle = 0.85
        correct_angle = 0

        if angle < 90:
            calculated_angle = -0.75 * (angle / 90)
            if calculated_angle < -max_angle:
                correct_angle = -max_angle
            else:
                correct_angle = calculated_angle
        elif angle > 90:
            calculated_angle = 0.75 * ((angle - 90)/90)
            if calculated_angle > max_angle:
                correct_angle = max_angle
            else:
                correct_angle = calculated_angle
        else:
            correct_angle = 0
            
        #print(angle, correct_angle)
        
        self.servo.value = correct_angle
    

    def _get_slope(self, x1, y1, x2, y2):
        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)
        
        
        #return (y2 - y1) / (x2 - x1)
        return x1
    


        # ===================================== SEND COMMANDS =================================
    def _send_command(self, outPs, commands):
        for cmd in commands:
            cmd_ =  self.rcBrain.getMessage(cmd)
            print(cmd_)
            if cmd_ is not None:
                encode = json.dumps(cmd_).encode()
                decode = encode.decode()
                command = json.loads(decode)
                #for outP in outPs:
                for ii in range(0,20):
                    outPs.send(command)
            # translated_cmd = self.controller.get_commands(cmd)
            # print (translated_cmd)
            # if translated_cmd is not None:
            #     for ii in range(0,7):
            #         #for outP in outPs:
            #         outPs.send(cmd)
    