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

from threading       import Thread
from threading       import Timer
from multiprocessing import Pipe

import cv2
import matplotlib.pyplot as plt

from src.templates.workerprocess import WorkerProcess
#from pynput import keyboard 
#from src.utils.tflite import ObjectDetector

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
        self.HEIGHT = 320
        self.WIDTH = 320
        self.inPs = inPipes[0]
        self.inDetectedPs = inPipes[1]
        self.outPs = outPipes[0]
        self.outImgPs = outPipes[1]
        
        self.PARKING = False
        self.STOP = False
        self.CURRENT_STATE = 'null'
        self.FLAG = True
        self.delayState = 'null'
        
        #self.inDetected, self.outImg = Pipe(duplex=False)
        
        #self.listener = ObjectDetector([self.inDetected], [self.outImg])
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
        stencilX = np.zeros((self.HEIGHT, self.WIDTH))
        stencilX = stencilX.astype('uint8')
        # specify coordinates of the polygon
        #polygon = np.array([[0, 480], [0,300], [75, 230], [550, 230], [640, 300], [640, 480]])
        #polygon = np.array([[0, 320], [0,200], [30,170], [170,170], [320,200], [320, 320]])
        
        polygon = np.array([[0, 320], [0,130], [320,130], [320, 320]])
        #polygon = np.array([[0, 320], [0,140], [85, 140], [85, 300], [245, 300], [245,140], [320,140], [320, 320]])
        cv2.fillConvexPoly(stencil_reg, polygon, 1)
        poly1 = np.array([[0, 320], [0,145], [60, 145], [70, 300]])
        poly2 = np.array([[260, 300], [260,140], [320,140], [320, 320]])
        cv2.fillPoly(stencilX, [poly1, poly2], 1)
        #ii = 0
        cmdOut = "none"
        
        t = Timer(0.5, self._set_delay_state)
        
        
        # Testing parallel parking
        self.PARKING = False
        
        frameCounter = 0
        YMAX = 0
        
        stop_flag = False
        park_flag = False
        keep_driving = True
        priority_flag = False
        pedestrian_flag = False
        crosswalk_flag = False
        flag = True
        
        stencil = stencil_reg
        
    
        winname = 'RebelDynamics'
        cv2.namedWindow(winname)
        cv2.moveWindow(winname, 0,0)
        start = 0
        end = 0
        
        while self.FLAG:
            try:
                
                #if self.CURRENT_STATE == 'found_stop_sign':
                if self.CURRENT_STATE == 'ped_on_crosswalk':
                    end = time.time()
                    if end - start >= 5:
                    #self.delayState = 'right_of_way'
                        self.CURRENT_STATE = 'drive_forward'
                    
                    #t.start()
                    #t.join()
                    
                
                # get image
                stamps, image = inP.recv()
                #image = cv2.resize(image, (320, 320))
                
                #for ii in self.inDetected:
                #    for detected in ii.recv():
                #        print(detected)
                
                
                # send to object detection
                rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                
                
                if frameCounter == 0:
                #for out_frames in outImg:
                    self.outImgPs.send([rgb_img])
                
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
                
                # convert to rgb
                #rgb_img = cv2.cvtColor(img_crop, cv2.COLOR_BGR2RGB) 
                
                # get lane lines
                lane_lines = self._avg_slope_intersect(lines)
                
                
                
                
                if frameCounter == 0:
                    frame_objects, detected_objects, YMAX = self.inDetectedPs.recv()
                # check if list is not empty
                    if detected_objects and YMAX:
                        #print(detected_objects, YMAX)
                        
                        
                        for objects in detected_objects:
                            #print(objects)
                            '''
                            if 'stop' in objects:
                                #self.CURRENT_STATE = 'stop_sign'
                                self.CURRENT_STATE = 'found_stop_sign'
                                print('**************************')
                                print('FOUND STOP SIGN')
                                print('**************************')
                            '''
                            
                            #if 'park' in objects and self.CURRENT_STATE != 'found_parking' and self.CURRENT_STATE != 'start_parking':
                            '''
                            if 'park' in objects and flag:
                                #self.CURRENT_STATE = 'found_parking'
                                self.CURRENT_STATE = 'start_parking'
                                print('**************************')
                                print('FOUND PARKING SIGN')
                                print('**************************')
                                flag = False
                            '''
                            '''
                            if 'priority' in objects and flag:
                                #self.CURRENT_STATE = 'right_of_way'
                                self.CURRENT_STATE = 'priority'
                                print('**************************')
                                print('FOUND PRIORITY SIGN')
                                print('**************************')
                                flag = False
                                start = time.time()
                            '''
                            if 'crosswalk' or 'pedestrian' in objects and flag:
                                #self.CURRENT_STATE = 'right_of_way'
                                self.CURRENT_STATE = 'pedestrian_crossing'
                                #print('**************************')
                                #print('FOUND PEDESTRIAN/CROSSWALK SIGN')
                                #print('**************************')
                                #flag = False
                                pedestrian_flag = True
                                #crosswalk_flag = True
                                stencil = stencilX
                                start = time.time()
                            
                            #print(pedestrian_flag)
                            
                            if pedestrian_flag:
                                
                                if 'pedestrian' in objects:
                                    start = time.time()
                                else:
                                    end = time.time()
                                    print('waiting for pedestrian', end - start)
                                    if abs(end - start) > 10:
                                        print('no pedestrian in site')
                                        self.CURRENT_STATE = 'null'
                                        start = time.time()
                                        pedestrian_flag = False
                                        flag = False
                                        crosswalk_flag = True
                                #print(objects, int(end - start))
                            elif crosswalk_flag:
                                print('going through crosswalk')
                                end = time.time()
                                if end - start > 4:
                                    stencil = stencil_reg
                                    crosswalk_flag = False
                            
                            #if not flag:
                            #    print(flag)
                    elif pedestrian_flag:
                        
                        end = time.time()
                        print('waiting for pedestrian', end - start)
                        if abs(end - start) > 5:
                            print('no pedestrian in site')
                            self.CURRENT_STATE = 'null'
                            start = time.time()
                            pedestrian_flag = False
                            flag = False
                            crosswalk_flag = True
                        #print(objects, int(end - start))
                    elif crosswalk_flag:
                        print('going through crosswalk')
                        end = time.time()
                        if end - start > 4:
                            stencil = stencil_reg
                            crosswalk_flag = False
                else:
                    frame_objects = rgb_img
                    
                
                # draw lines to grayscale image
                lane_lines_img, lane_centering_cmds = self._display_lines(frame_objects, lane_lines)
                #lane_lines_img, lane_centering_cmds = self._display_lines(img_crop, lane_lines)
   
                
                #plt.imshow(lane_lines_img)
                #plt.show()
                
                out_img = cv2.resize(lane_lines_img, (1080, 1080))
                cv2.imshow(winname, out_img)
                cv2.waitKey(1)
                
                
                frameCounter = (frameCounter + 1) % 5
                
                #for outP in outPs:
                # check if parking mode
                
                '''
                if keep_driving:
                    outPs.send(lane_centering_cmds)
                    #keep_driving = False
                '''   
                    
                if self.CURRENT_STATE == 'pedestrian_crossing':
                    #print('setting up pedestrian crossing commands')
                    outPs.send(['pedestrian_crossing'])
                    
                    #for jj in range(0,5):
                    #    outPs.send(['pedestrian_crossing'])
                    #keep_driving = False
                elif keep_driving:
                    outPs.send(lane_centering_cmds)
                
                '''
                if priority_flag and self.CURRENT_STATE == 'right_of_way':
                    print('setting right of way commands')
                    outPs.send(['right_of_way'])
                    for jj in range(0,5):
                        outPs.send(['right_of_way'])
                    priority_flag = False
                    self.CURRENT_STATE = 'null'
                    #keep_driving = False
                elif keep_driving:
                    outPs.send(lane_centering_cmds)
                
                '''
                '''
                if park_flag:
                    outPs.send(['parallel_park'])
                    park_flag = False
                
                
                if self.CURRENT_STATE == 'start_parking' and park_flag:
                    outPs.send(['parallel_park'])
                    self.CURRENT_STATE = 'null'
                    park_flag = False
                    keep_driving = False
                elif keep_driving:
                    outPs.send(lane_centering_cmds)
                
                
                elif self.CURRENT_STATE == 'start_stopsign_proc' and stop_flag:
                    
                    
                    outPs.send(['stop_sign'])
                    self.CURRENT_STATE == 'null'
                    stop_flag = False
                '''
                ### else only focus on lane centering
                #else:
                #    outPs.send(lane_centering_cmds)
                
                
                
                #if stop_flag:
                    #print('testing stop sign')
                #    outPs.send(['stop_sign'])
                    #stop_flag = False

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


    def _set_delay_state(self, ):
        print('-----delay state-----')
        self.CURRENT_STATE = 'right_of_way'


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
            y_offset = int(self.HEIGHT / 2)
            
            
            #print(left_x1, left_x2, right_x1, right_x2)
            
            #check for stop line
            if right_x2 < left_x2 and right_x2 > 150 and right_x2 < 320:
                #print('one')
                x1, _, x2, _ = lines[1][0]
                x_offset = int(x2 - x1)
                #print(x_offset)
                x_offset = x_offset + mid
                y_offset = int(self.HEIGHT / 2)
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
                y_offset = int(self.HEIGHT / 2)
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
                y_offset = int(self.HEIGHT / 2)
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
            y_offset = int(self.HEIGHT / 2)
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
        if x1 <= 110:
            return ['forward', 'left', 'left']
        elif x1 >= 210:
            return ['forward', 'right', 'right']
        else:
            return ['forward', 'straight']
    
    
    def _get_slope(self, x1, y1, x2, y2):
        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)
        
        
        #return (y2 - y1) / (x2 - x1)
        return x1
    