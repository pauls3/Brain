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

from threading       import Thread
from threading       import Timer
from multiprocessing import Pipe

import cv2
import matplotlib.pyplot as plt

from src.templates.workerprocess import WorkerProcess
from src.utils.control.RcBrainThread              import RcBrainThread

#from pynput import keyboard 
#from src.utils.tflite import ObjectDetector

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
        #self.inDetectedPs = inPipes[1]
        self.outPs = outPipes[0]
        #self.outImgPs = outPipes[1]
        
        self.CURRENT_STATE = 'null'
        self.FLAG = True
        self.delayState = 'null'

        self.CURRENT_DRIVE_STATE = "null"
        self.PARKING = True
        self.STOP_SIGN = True
        self.PRIORITY = True
        self.PEDESTRIAN = True
        
        self.rcBrain = RcBrainThread()

        #self.inDetected, self.outImg = Pipe(duplex=False)
        
        #self.listener = ObjectDetector([self.inDetected], [self.outImg])
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
        
        streamTh = Thread(name='StreamSendingThread',target = self._process_image, args= (self.inPs, self.outPs, ))
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
        stencil_reg = np.zeros((self.HEIGHT, self.WIDTH))
        stencil_reg = stencil_reg.astype('uint8')
        #stencilX = np.zeros((self.HEIGHT, self.WIDTH))
        #stencilX = stencilX.astype('uint8')
        # specify coordinates of the polygon
        #polygon = np.array([[0, 480], [0,300], [75, 230], [550, 230], [640, 300], [640, 480]])
        #polygon = np.array([[0, 320], [0,200], [30,170], [170,170], [320,200], [320, 320]])
        
        #polygon = np.array([[0, 320], [0,170], [320,170], [320, 320]])
        #polygon = np.array([[0, 320], [0,220], [320,220], [320, 320]])
        polygon = np.array([[0, 300], [0,220], [300,220], [300, 300]])
        #polygon = np.array([[0, 640], [0,425], [640,425], [640, 640]])
        #polygon = np.array([[0, 320], [0,140], [85, 140], [85, 300], [245, 300], [245,140], [320,140], [320, 320]])
        cv2.fillConvexPoly(stencil_reg, polygon, 1)
        #poly1 = np.array([[0, 320], [0,145], [60, 145], [70, 300]])
        #poly2 = np.array([[260, 300], [260,140], [320,140], [320, 320]])
        
        #poly1 = np.array([[0, 320], [0,275], [60, 275], [70, 320]])
        #poly2 = np.array([[260, 320], [260,275], [320,275], [320, 320]])
        #cv2.fillPoly(stencilX, [poly1, poly2], 1)
        #ii = 0
        cmdOut = "none"
                
        # Testing parallel parking
        
        frameCounter = 0
        YMAX = 0
        
        flag = True
        
        stencil = stencil_reg
        
        winname = 'RebelDynamics'
        cv2.namedWindow(winname)
        cv2.moveWindow(winname, 0,0)
        start = 0
        end = 0

        idx = 0
        
        while True:
            try:
            
                stamps, image = inP.recv()
                #image = cv2.resize(image, (320, 320))
                
                # send to object detection
                rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                
                # convert to grayscale
                gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # crop with mask
                #img_crop_gray = cv2.bitwise_and(gray_img, gray_img, mask=stencil)
                img_crop = cv2.bitwise_and(image, image, mask=stencil)
                # convert to grayscale
                img_crop_gray = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
                # blur
                #blur_img = cv2.blur(img_crop_gray, (10,10))
                blur_img = cv2.GaussianBlur(img_crop_gray, (3,3), 0)
                # get threshold
                ret, thresh = cv2.threshold(blur_img, 110, 170, cv2.THRESH_BINARY) 
                
                # get edges
                # Canny 
                edges = cv2.Canny(image=thresh, threshold1=100, threshold2=200)
                # Sobel
                #edges = np.uint8(cv2.Sobel(src=thresh, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5))
                
                # get lines
                lines = cv2.HoughLinesP(edges, 1, np.pi/180, 25, maxLineGap=200)
                
                # convert to rgb
                #rgb_img = cv2.cvtColor(img_crop, cv2.COLOR_BGR2RGB) 
                
                # get lane lines
                lane_lines = self._avg_slope_intersect(lines)
                
                
                # draw lines to grayscale image
                #lane_lines_img, lane_centering_cmds = self._display_lines(frame_objects, lane_lines)
                #lane_lines_img, lane_centering_cmds = self._display_lines(img_crop, lane_lines)
   
                
                #plt.imshow(lane_lines_img)
                #plt.show()
                
                #out_img = cv2.resize(lane_lines_img, (640, 480))
                #cv2.imshow(winname, out_img)
                #cv2.imshow(winname, lane_lines_img)
                cv2.imshow(winname, edges)
                cv2.waitKey(1)
                
                
                #frameCounter = (frameCounter + 1) % 5
                
                #for outP in outPs:
                
            except Exception as e:
                print("CameraStreamer failed to stream images:",e,"\n")
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
                    else:
                        if x1 > right_region_boundary and x2 > right_region_boundary and abs(angle) % 90 > 30.0:
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


    def _set_delay_state(self, ):
        print('-----delay state-----')
        if self.CURRENT_STATE == 'found_stop_sign':
            self.CURRENT_STATE = 'start_stopsign_proc'
        #elif self.CURRENT_STATE == 'right_of_way':
        #    self.CURRENT_STATE = 'right_of_way_proc'
        #elif self.CURRENT_STATE == 'start_parking':
        #    self.CURRENT_STATE = 'null'
        #self.CURRENT_STATE = 'right_of_way'
        

    def _display_lines(self, frame, lines):
        line_img = np.zeros((self.HEIGHT, self.WIDTH, 3))
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_img, (x1, y1), (x2, y2), (0,0,255), 2)
                    
                    if x1 < 0 and x2 > 200 or x1 > 200 and x2 < 0 and self.CURRENT_STATE == 'find_stop_line':
                        # drive for 4 seconds
                        self.CURRENT_STATE == 'found_stop_line'
                        
                        
                    else:
                        cv2.line(line_img, (x1, y1), (x2, y2), (0,0,255), 2)
        
        frame = frame.astype('uint8')
        line_img = line_img.astype('uint8')
        
        commands = []
        ##################
        # draw center line
        ##################
        # Detected 2 lanes
        #print(lines)
        
        #mid = 100
        mid = int(self.WIDTH / 2)
        if len(lines) == 2:
            left_x1, _, left_x2, _ = lines[0][0]
            right_x1, _, right_x2, _ = lines[1][0]
            #print(lines[0][0])
            x_offset = (left_x2 + right_x2) / 2
            y_offset = int(self.HEIGHT *  2 / 3)
            
            
            #print(left_x1, left_x2, right_x1, right_x2)
            
            #check for stop line
            if right_x2 < left_x2 and right_x2 > 150 and right_x2 < 320:
                #print('one')
                x1, _, x2, _ = lines[1][0]
                x_offset = int(x2 - x1)
                #print(x_offset)
                x_offset = x_offset + mid
                #y_offset = int(self.HEIGHT / 2)
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (int(mid), self.HEIGHT), (0,255,0), 3)
                #print(self._get_slope(x_offset, y_offset, self.WIDTH/2, self.HEIGHT))
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, )
            elif right_x2 < left_x2 and (right_x2 < 50 and right_x1 > mid) or (right_x2 > 320 and right_x1 < mid):
                #print('two')
                x1, _, x2, _ = lines[0][0]
                x_offset = int(x2 - x1)
                #print(x_offset)
                x_offset = x_offset + mid
                #y_offset = int(self.HEIGHT / 2)
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (int(mid), self.HEIGHT), (0,255,0), 3)
                #print(self._get_slope(x_offset, y_offset, self.WIDTH/2, self.HEIGHT))
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, )
            
            elif abs(left_x1 - right_x1) < 50:
                #print('three')
                #mid = int(self.WIDTH / 2)
                #mid = 100
                x1, _, x2, _ = lines[0][0]
                x_offset = int(x2 - x1)
                
                x_offset = x_offset + mid
                #y_offset = int(self.HEIGHT / 2)
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (int(mid), self.HEIGHT), (0,255,0), 3)
                #print(self._get_slope(x_offset, y_offset, self.WIDTH/2, self.HEIGHT))
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, )
            else:
                #print('four')
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (mid, self.HEIGHT), (0,255,0), 3)
                #print(self._get_slope(x_offset, y_offset, self.WIDTH/2, self.HEIGHT))
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, )
        # detected one lane
        elif len(lines) == 1:
            #mid = 100
            x1, _, x2, _ = lines[0][0]
            x_offset = int(x2 - x1)
            #print(x_offset)
            x_offset = x_offset + mid
            y_offset = int(self.HEIGHT * 2 / 3)
            # draw center line
            
            
            
            #print(offset, x1, x2)
            
            #if x1 > 320 and x2 < 130:
                
            
            cv2.line(frame, (int(x_offset), int(y_offset)), (mid, self.HEIGHT), (0,255,0), 3)
            #print(self._get_slope(x_offset, y_offset, self.WIDTH/2, self.HEIGHT))
            overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
            commands = self._steering_cmd(x_offset, )
        # No lanes detected
        else:
            overlay_img = frame
        #cv2.line(frame, (int(x_offset), int(y_offset)), (int(self.WIDTH/2), self.HEIGHT), (0,255,0), 3)
        
        #overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
        return overlay_img, commands
    
    
    
    def _steering_cmd(self, x1):
        #if x1 <= 110:
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
    
    
    def _get_slope(self, x1, y1, x2, y2):
        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)
        
        
        #return (y2 - y1) / (x2 - x1)
        return x1
    