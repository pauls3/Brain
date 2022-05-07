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
from src.utils.control.RemoteControlReceiverProcess import RemoteControlReceiverProcess
from src.utils.control.RcBrainThread                import RcBrainThread
from src.utils.finitestatemachine.FiniteStateMachine import FiniteStateMachine

from gpiozero import Servo
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory

Device.pin_factory = PiGPIOFactory('127.0.0.1')

#from pynput import keyboard 
#from src.utils.tflite import ObjectDetector

class BrainControl(WorkerProcess):
    
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
        super(BrainControl,self).__init__(inPipes, outPipes)
        self.HEIGHT = 640
        self.WIDTH = 640
        self.inPVision = inPipes[0]
        self.inPDetections = inPipes[1]
        #self.inPStopLine = inPipes[2]
        #self.inDetectedPs = inPipes[1]
        self.outPs = outPipes[0]
        #self.outImgPs = outPipes[1]
        self.confidenceThreshold = 0.5

        self.curr_steer_angle = 90
        self.currentState = "null"
        self.curr_view_angle = 'c'
        self.curr_speed = 0.0
        
        self.controller = RemoteControlReceiverProcess()
        self.rcBrain = RcBrainThread()
        self.fStateMachine = FiniteStateMachine()
        self.servo = Servo(12)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        #self._init_socket()
        super(BrainControl,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the sending thread.
        """
        
        if self._blocker.is_set():
            return
        #self.listener.daemon = self.daemon
        #self.threads.append(self.listener)
        
        streamTh = Thread(name='BrainControlThread', target=self._generate_command, args=(self.inPVision, self.inPDetections, self.outPs, ))
        streamTh.daemon = True
        self.threads.append(streamTh)
        

    # ===================================== SEND COMMANDS =================================
    def _send_command(self, outPs, commands):
        #print(command)
        for cmd in commands:
            cmd_ =  self.rcBrain.getMessage(cmd)
            if cmd_ is not None:
                encode = json.dumps(cmd_).encode()
                decode = encode.decode()
                command = json.loads(decode)
                #for outP in outPs:
                for ii in range(0,1):
                    outPs.send(command)
            # translated_cmd = self.controller.get_commands(cmd)
            # print (translated_cmd)
            # if translated_cmd is not None:
            #     for ii in range(0,7):
            #         #for outP in outPs:
            #         outPs.send(cmd)
    
    
    # ===================================== SEND THREAD ==================================
    def _generate_command(self, inPVision, inPDetections, outPs):
        """Sending the frames received thought the input pipe to remote client by using the created socket connection. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe to read the frames from CameraProcess or CameraSpooferProcess. 
        """               
        frameCounter = 0
        YMAX = 0
        
        labels = [
            'car',
            'pedestrian',
            'crosswalk sign',
            'parking sign',
            'priority sign',
            'stop sign',
            'roundabout sign',
            'highway sign',
            'highway end sign',
            'no entry sign',
            'one-way sign',
            'traffic light',
            'barricade'
            ]
        
        obj_detect_flag = True
                        
        winname = 'RebelDynamics'
        cv2.namedWindow(winname)
        #cv2.moveWindow(winname, 0,0)
        
        tmp_state = ""

        # time.sleep(20)

        print('**************************')
        print('Starting PID')
        print('**************************')
        cmds = ['pid', 'stop']
        self._send_command(outPs, cmds)
        # time.sleep(5)
        
        timer1 = time.time()
        frames = 0
        timer2 = 0
        t2secs = 0
        fps = 0

        while True:
            timer2 = time.time()
            if timer2 - timer1 > 20:
                break
        
        
        while True:
            try:
                #timer2 = time.time()
                # get image
                inArray = inPVision.recv()
                image = inArray[0]
                steer_angle = inArray[1]
                foundStopLine = inArray[2]
                #[image, steer_angle, foundStopLine] = inPVision.recv()   
                #rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                #foundStopLine = inPStopline.recv()
                
                '''
                if frameCounter == 0 and obj_detect_flag:
                #for out_frames in outImg:
                    self.outImgPs.send([rgb_img])
                '''

                                
                # Draw bounding boxes of detected objects
                detections = inPDetections.recv()
                drawn_image = self.draw_image(image, detections, labels)
                #out_img = cv2.resize(drawn_image, (640, 640))
                #cv2.imshow(winname, image)
                #cv2.waitKey(1)
                detectionObjects = self._check_detections(detections, labels)


                #self._send_command(outPs, fake_cmds)
                #self._send_command(outPs, steering_angle)
                



                # Try to change state
                if foundStopLine:
                    print('found stop line')
                    ''' 
                        at intersection:
                        rotate servo left and right
                        figure out what to do next
                    '''
                    # at_intersection
                    self.fStateMachine.change_state('at_intersection')


                # Get current state
                #self.currentState = self.fStateMachine.get_state()
                print(self.fStateMachine.get_state())
                # Conduct actions according to current state


                if self.currentState == "lane_centering":
                    print('lane centering')

                    if self.curr_speed != 10.0:
                        self.curr_speed = 10.0
                        self._send_command(outPs, ['forward_normal'])

                    self.curr_steer_angle = steer_angle
                    self._change_steering(self.curr_steer_angle)
                    #if self.curr_speed == 0.0:
                    #    self.curr_speed = 10.5
                elif self.currentState == 'at_intersection':
                    # look around and cross intersection
                    print("at intersection")

                '''
                frames = frames + 1
                end1 = time.time()
                t1secs = end1-timer1
                fps = round(frames/t1secs,2)
                '''

            except Exception as e:
                print("Error in brain control:",e,"\n")
                # Reinitialize the socket for reconnecting to client.  
                #self.connection = None
                #self._init_socket()
                pass
        


    def _check_detections(self, detections, labels):
        print('checking detections')

        # iterate and check objects in regards to where they are in the image:



    def draw_image(self, image, detections, labels):
        font = cv2.FONT_HERSHEY_SIMPLEX
        if detections is not None:
            for detection in detections:
                objID = detection[0]
                confidence = detection[1]
                xmin = detection[2]
                ymin = detection[3]
                xmax = detection[4]
                ymax = detection[5]

                if confidence > self.confidenceThreshold:
                    # bbox
                    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color=(0,255,255))
                    # label box
                    cv2.rectangle(image, (xmin-1, ymin-1), (xmin+70, ymin-10), (0,255,255), -1)
                    # label text
                    cv2.putText(image, ' '+labels[objID]+' ' + str(round(confidence, 2)), (xmin, ymin-2), font, 0.3, (0,0,0),1,cv2.LINE_AA)





    
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
    