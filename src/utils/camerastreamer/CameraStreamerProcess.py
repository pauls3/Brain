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

import cv2
import matplotlib.pyplot as plt

from src.templates.workerprocess import WorkerProcess
from src.utils.control.RcBrainThread                import RcBrainThread

from gpiozero import Servo
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory

Device.pin_factory = PiGPIOFactory('127.0.0.1')

#from pynput import keyboard 
#from src.utils.tflite import ObjectDetector

def nothing(x):
    pass

class CameraStreamerProcess(WorkerProcess):
    
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
        super(CameraStreamerProcess,self).__init__(inPipes, outPipes)
        self.HEIGHT = 300
        self.WIDTH = 300
        self.inPs = inPipes[0]
        #self.inDetectedPs = inPipes[1]
        self.outPs = outPipes[0]
        self.curr_steer_angle = 0.0

        self.rcBrain = RcBrainThread()
        self.servo = Servo(12)
        
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        #self._init_socket()
        super(CameraStreamerProcess,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the sending thread.
        """
        
        if self._blocker.is_set():
            return
        #self.listener.daemon = self.daemon
        #self.threads.append(self.listener)
        
        streamTh = Thread(name='ProcessImageThread',target = self._process_image, args= (self.inPs, self.outPs, ))
        streamTh.daemon = True
        self.threads.append(streamTh)
        

    
    # ===================================== SEND THREAD ==================================
    def _process_image(self, inP, outPs):
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

        #self._send_command(outPs, ['forward_normal'])
        
        '''timer1 = time.time()
        while True:
            timer2 = time.time()
            if timer2 - timer1 > 10:
                break

        self._send_command(outPs, ['forward_slow'])
        '''
        self._send_command(outPs, ['forward_normal'])
        

        stencil_reg = np.zeros((self.HEIGHT, self.WIDTH))
        stencil_reg = stencil_reg.astype('uint8')
        #stencilX = np.zeros((self.HEIGHT, self.WIDTH))
        #stencilX = stencilX.astype('uint8')
        # specify coordinates of the polygon
        #polygon = np.array([[0, 480], [0,300], [75, 230], [550, 230], [640, 300], [640, 480]])
        #polygon = np.array([[0, 320], [0,200], [30,170], [170,170], [320,200], [320, 320]])
        
        #polygon = np.array([[0, 320], [0,170], [320,170], [320, 320]])
        #polygon = np.array([[0, 320], [0,150], [320,150], [320, 320]])
        #polygon = np.array([[0, 640], [0,300], [640,300], [640, 640]])

        # polygon1 = np.array([[0, 640], [0,320], [140,320], [140, 640]])
        # polygon2 = np.array([[500,640], [500, 320], [640,320], [640, 640]])
        #polygon1 = np.array([[0, 300], [0,150], [90,150], [90, 300]])
        #polygon2 = np.array([[210,300], [210, 150], [300,150], [300, 300]])
        #cv2.fillPoly(stencil_reg, [polygon1, polygon2], 1)
        #polygon = polygon.astype('uint8')
        #polygon = np.array([[0, 640], [0,425], [640,425], [640, 640]])
        #polygon = np.array([[0, 320], [0,140], [85, 140], [85, 300], [245, 300], [245,140], [320,140], [320, 320]])
        
        polygon = np.array([[0, 300], [0,150], [300,150], [300, 300]])
        cv2.fillConvexPoly(stencil_reg, polygon, 1)
        #poly1 = np.array([[0, 320], [0,145], [60, 145], [70, 300]])
        #poly2 = np.array([[260, 300], [260,140], [320,140], [320, 320]])
        
        #poly1 = np.array([[0, 320], [0,275], [60, 275], [70, 320]])
        #poly2 = np.array([[260, 320], [260,275], [320,275], [320, 320]])
        #cv2.fillPoly(stencilX, [poly1, poly2], 1)
        #ii = 0
        
                
        YMAX = 0
            
                
        stencil = stencil_reg
        
        winname = 'RebelDynamics'
        cv2.namedWindow(winname)
        #cv2.moveWindow(winname, 0,0)
        

        # time.sleep(5)

        print('_image')

        cv2.createTrackbar('Thresh0', 'RebelDynamics', 0, 255, nothing)
        cv2.createTrackbar('Thresh1', 'RebelDynamics', 0, 255, nothing)
        cv2.createTrackbar('HoughLines', 'RebelDynamics', 0, 255, nothing)
        cv2.createTrackbar('HoughGap', 'RebelDynamics', 1, 255, nothing)
        
        thresh0 = thresh1 = hough0 = hough1 =  1
        Pthresh0 = Pthresh1 = Phough0 = Phough1 = 1

        left_turn = []

        steerFlag = 0

        timer1 = time.time()
        while True:
            try:

                timer2 = time.time()
                # get image
                stamps, rawImage = inP.recv()
                
                image = cv2.resize(rawImage, (300, 300))
                # send to object detection
                rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                

                # convert to grayscale
                #gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # crop with mask
                #img_crop_gray = cv2.bitwise_and(gray_img, gray_img, mask=stencil)
                img_crop = cv2.bitwise_and(image, image, mask=stencil)
                # convert to grayscale
                # img_crop_gray = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
                img_crop_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # blur
                #blur_img = cv2.blur(img_crop_gray, (10,10))
                blur_img = cv2.GaussianBlur(img_crop_gray, (5,5), 0)
                # get threshold
                # ret, thresh = cv2.threshold(blur_img, 110, 170, cv2.THRESH_BINARY)
                # (199, 170) for plastic ground
                ret, thresh = cv2.threshold(blur_img, thresh0, thresh1, cv2.THRESH_BINARY)
                # get edges
                # Canny 
                edges = cv2.Canny(image=thresh, threshold1=100, threshold2=200)
                # Sobel
                #edges = np.uint8(cv2.Sobel(src=thresh, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5))
                
                # get lines
                #lines = cv2.HoughLinesP(edges, 1, np.pi/180, 25, maxLineGap=200)
                lines = cv2.HoughLinesP(edges, 1, np.pi/hough0, hough1, maxLineGap=200)
                
                if lines is not None:
                    for jj in range(0, len(lines)):
                        ll = lines[jj][0]
                        cv2.line(rgb_img, (ll[0], ll[1]), (ll[2], ll[3]), (0,0,255), 3, cv2.LINE_AA)
                
                # convert to rgb
                #rgb_img = cv2.cvtColor(img_crop, cv2.COLOR_BGR2RGB) 
                
                # get lane lines
                lane_lines = self._avg_slope_intersect(lines)
                
                #frame_objects = rgb_img
                
                # draw lines to grayscale image
                #lane_lines_img, lane_centering_cmds = self._display_lines(frame_objects, lane_lines)
                #lane_lines_img, steering_angle, num_lines = self._display_lines(img_crop, lane_lines)
                lane_lines_img, steering_angle, num_lines, stopLine = self._display_lines(rgb_img, lane_lines)
                
                self.curr_steer_angle = self.stabilize_steering_angle(self.curr_steer_angle, steering_angle, num_lines, )
                
                #plt.imshow(lane_lines_img)
                #plt.show()
                #cv2.putText(lane_lines_img,'VID FPS: '+str(fps), (225, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3,(0, 255, 255), 1, cv2.LINE_AA)
                #out_img = cv2.resize(lane_lines_img, (640, 640))
                #cv2.imshow(winname, out_img)
                
                #for outP in self.outImgPs:
                out_arry = [lane_lines_img, self.curr_steer_angle, stopLine]
                # self.outPs.send([self.curr_steer_angle, stopLine])
                

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
                #cv2.imshow(winname, rgb_img)
                #cv2.imshow(winname, edges)
                #cv2.waitKey(1)
                # print('image')



                # Testing intersection crossing
                # time.sleep(5)
                # time.sleep(5)



                
                
                passed_time = timer2 - timer1
                
                # from stop line to left turn:
                #     go forward 5 seconds
                #     make left turn (-0.75) for 8 seconds
                #     go straight
                '''
                if passed_time > 5 and steerFlag == 0:
                    self._test_steering(-0.75)
                    steerFlag = 1

                if timer2 - timer1 > 13 and steerFlag == 1:
                    self._test_steering(0.0)
                    steerFlag = 2
                '''

                if passed_time > 1 and steerFlag == 0:
                    self._test_steering(0.75)
                    steerFlag = 1

                if timer2 - timer1 > 13 and steerFlag == 1:
                    self._test_steering(0.0)
                    steerFlag = 2
                # time.sleep(3)
                # self._test_steering(0.0)


                                


                #self._change_steering(steering_angle)
                
            except Exception as e:
                print("CameraStreamerProcess failed to stream images:",e,"\n")
                # Reinitialize the socket for reconnecting to client.  
                #self.connection = None
                #self._init_socket()
                pass
            


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
        max_angle = 0.75
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
    