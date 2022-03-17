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

from threading import Thread

import cv2
import matplotlib.pyplot as plt

from src.templates.workerprocess import WorkerProcess

class CameraStreamerProcess(WorkerProcess):
    HEIGHT = 480
    WIDTH = 640
    
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
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
        super(CameraStreamerProcess,self).__init__( inPs, outPs)
        self.HEIGHT = 480
        self.WIDTH = 640
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
        streamTh = Thread(name='StreamSendingThread',target = self._process_image, args= (self.inPs[0], self.outPs, ))
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
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        
        stencil = np.zeros((self.HEIGHT, self.WIDTH))
        stencil = stencil.astype('uint8')
        # specify coordinates of the polygon
        #polygon = np.array([[50,270], [220,160], [360,160], [480,270]])
        polygon = np.array([[0, 480], [0,300], [50,170], [590,170], [640,300], [640, 480]])
        cv2.fillConvexPoly(stencil, polygon, 1)
        #ii = 0
        cmdOut = "none"
        while True:
            try:
                # get image
                stamps, image = inP.recv()
                # convert to grayscale
                gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # crop with mask
                #img_crop_gray = cv2.bitwise_and(gray_img, gray_img, mask=stencil)
                img_crop = cv2.bitwise_and(image, image, mask=stencil)
                # convert to grayscale
                img_crop_gray = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
                # blur
                blur_img = cv2.blur(img_crop_gray, (10,10))
                # get threshold
                ret, thresh = cv2.threshold(blur_img, 125, 170, cv2.THRESH_BINARY) 
                # get lines
                lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 50, maxLineGap=200)
                
                # convert gray to rgb (3 channels for gray)
                rgb_img = cv2.cvtColor(img_crop, cv2.COLOR_BGR2RGB) 
                
                # get lane lines
                lane_lines = self._avg_slope_intersect(lines)
                # draw lines to grayscale image
                lane_lines_img, commands = self._display_lines(rgb_img, lane_lines)
                
                cv2.imshow('Image', lane_lines_img)
                cv2.waitKey(1)
                
                
                for outP in outPs:
                    #print(outP)
                    #outP.send(ii)
                    outP.send(commands)

                #ii = ii + 1
                #time.sleep(1)
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
        boundary = 1/3
        left_region_boundary = self.WIDTH * (1-boundary)
        right_region_boundary = self.WIDTH * boundary
        
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 == x2:
                    continue
                fit = np.polyfit((x1, x2), (y1, y2,), 1)
                slope = fit[0]
                intercept = fit[1]
                if True:
                #if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                    else:
                        if x1 > right_region_boundary and x2 > right_region_boundary:
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
        y2 = int(y1 * 1/3)
        
        x1 = max(-self.WIDTH, min(2*self.WIDTH, int(y1 - intercept)/slope))
        x2 = max(-self.WIDTH, min(2*self.WIDTH, int(y2 - intercept)/slope))
        
        y1 = int(y1)
        x1 = int(x1)
        x2 = int(x2)
        
        return [[x1, y1, x2, y2]]


    def _display_lines(self, frame, lines):
        
        line_img = np.zeros((self.HEIGHT, self.WIDTH, 3))
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_img, (x1, y1), (x2, y2), (0,0,255), 2)
        
        frame = frame.astype('uint8')
        line_img = line_img.astype('uint8')
        
        commands = []
        ###
        # draw center line
        ###
        # Detected 2 lanes
        if len(lines) == 2:
            left_x1, _, left_x2, _ = lines[0][0]
            right_x1, _, right_x2, _ = lines[1][0]
            #print(lines[0][0])
            x_offset = (left_x2 + right_x2) / 2
            y_offset = int(self.HEIGHT / 2)
            if abs(left_x1 - right_x1) < 200:
                mid = int(self.WIDTH / 2)
                x1, _, x2, _ = lines[0][0]
                x_offset = int(x2 - x1)
                #print(x_offset)
                x_offset = x_offset + mid
                y_offset = int(self.HEIGHT / 2)
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (int(self.WIDTH/2), self.HEIGHT), (0,255,0), 3)
                #print(self._get_slope(x_offset, y_offset, self.WIDTH/2, self.HEIGHT))
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, )
            else:
                # draw center line
                cv2.line(frame, (int(x_offset), int(y_offset)), (int(self.WIDTH/2), self.HEIGHT), (0,255,0), 3)
                #print(self._get_slope(x_offset, y_offset, self.WIDTH/2, self.HEIGHT))
                overlay_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
                commands = self._steering_cmd(x_offset, )
        # detected one lane
        elif len(lines) == 1:
            mid = int(self.WIDTH / 2)
            x1, _, x2, _ = lines[0][0]
            x_offset = int(x2 - x1)
            #print(x_offset)
            x_offset = x_offset + mid
            y_offset = int(self.HEIGHT / 2)
            # draw center line
            cv2.line(frame, (int(x_offset), int(y_offset)), (int(self.WIDTH/2), self.HEIGHT), (0,255,0), 3)
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
        if x1 <= 200:
            return ['forward', 'right']
        elif x1 >= 450:
            return ['forward', 'left']
        else:
            return ['forward', 'straight']
    
    
    def _get_slope(self, x1, y1, x2, y2):
        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)
        
        
        #return (y2 - y1) / (x2 - x1)
        return x1