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
from multiprocessing import Queue

import cv2
import matplotlib.pyplot as plt

from src.templates.workerprocess import WorkerProcess

#from pynput import keyboard 
#from src.utils.tflite import ObjectDetector
import sys
#np.set_printoptions(threshold=sys.maxsize)

    
def nothing(x):
    pass

class ObjectDetection(WorkerProcess):
    
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
        super(ObjectDetection,self).__init__(inPipes, outPipes)
        self.HEIGHT = 300
        self.WIDTH = 300
        self.inPs = inPipes[0]
        self.outPs = outPipes[0]
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        #self._init_socket()
        super(ObjectDetection,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the sending thread.
        """
        
        if self._blocker.is_set():
            return
        #self.listener.daemon = self.daemon
        #self.threads.append(self.listener)
        
        streamTh = Thread(name='ObjectDetectionThread',target = self._detect_objects, args= (self.inPs, self.outPs))
        streamTh.daemon = True
        self.threads.append(streamTh)
    
    
    
    # ===================================== SEND THREAD ==================================
    def _detect_objects(self, inP, outPs):
        """Sending the frames received thought the input pipe to remote client by using the created socket connection. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe to read the frames from CameraProcess or CameraSpooferProcess. 
        """

        
        inputQueue = Queue(maxsize = 1)
        outputQueue = Queue(maxsize = 1)
        confThreshold = 0.5

        # Load the model
        print('Loading model')
        xml_path = '/home/pi/repos/Brain/src/utils/openvino/ssd_mobilenet/bosch_model_0/saved_model.xml'
        bin_path = '/home/pi/repos/Brain/src/utils/openvino/ssd_mobilenet/bosch_model_0/saved_model.bin'
        labels_file = '/home/pi/repos/Brain/src/utils/openvino/labels.txt'
        net = cv2.dnn.readNet(xml_path, bin_path)

        # Specify target device
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

        print('Loaded model!')

        with open(labels_file, 'r') as f:
            labels = [x.strip() for x in f]
        print(labels)

        

        # time.sleep(10)



        while True:
            try:
                print('***')

                if not inputQueue.empty():
                    print(1)
                    frame = inputQueue.get()
                    # resframe = cv2.resize(frame, (300, 300))
                    blob = cv2.dnn.blobFromImage(frame, 1, size=(300, 300), mean=(127.5,127.5,127.5), swapRB=True, crop=False)
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
                else:
                    print(2)
        

                    image = inP.recv()
                    # print(image)
                    #inputQueue.put(image.array)
                    if image is not None:
                        inputQueue.put(image)

                    print("2...")

                
                if not outputQueue.empty():
                    print(3)
                    out = outputQueue.get()
                    '''
                        output the detections
                    '''
                    #for outP in outPs:
                    outPs.send(out)
                    print(out)

                print(4)
                if image is not None:
                    image = inP.recv()
                print(5)
                # rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

                
            except Exception as e:
                print("Failure in object detection:",e,"\n")
                # Reinitialize the model  
                net = cv2.dnn.readNet(xml_path, bin_path)
                # Specify target device
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)
                pass
        
        
    