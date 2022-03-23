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

import os
import cv2
import numpy as np
from threading import Thread
from multiprocessing import Pipe
import importlib.util


from src.templates.workerprocess import WorkerProcess

class ObjectDetector(WorkerProcess):

    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
        """A thread for capturing the keyboard events. 
        
        Parameters
        ----------
        outPs : list(Pipe)
            List of output pipes.
        """
        super(ObjectDetector,self).__init__(inPs, outPs)

        self.outPs = outPs
        self.inPs = inPs


    # ===================================== START ========================================
    def run(self):
        super(ObjectDetector,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes. 
        """
        runCar = Thread(name='ObjectDetect',target = self._object_detect, args = (self.outPs, self.inPs, ))
        self.threads.append(runCar)
    
    
    def _object_detect(self, outP, inP):       
            
        MODEL_NAME = "/home/pi/repos/Brain/src/utils/tflite" #args.modeldir
        GRAPH_NAME = 'detect.tflite'
        LABELMAP_NAME = 'labelmap.txt'
        min_conf_threshold = float(0.30)
        resW = 300
        resH = 300
        imW = 300
        imH = 300
        #use_TPU = args.edgetpu

        # Import TensorFlow libraries
        # If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
        # If using Coral Edge TPU, import the load_delegate library
        pkg = importlib.util.find_spec('tflite_runtime')
        if pkg:
            from tflite_runtime.interpreter import Interpreter
            #if use_TPU:
            #    from tflite_runtime.interpreter import load_delegate
        else:
            from tensorflow.lite.python.interpreter import Interpreter
            #if use_TPU:
            #    from tensorflow.lite.python.interpreter import load_delegate

        # If using Edge TPU, assign filename for Edge TPU model
        #if use_TPU:
            # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
        #    if (GRAPH_NAME == 'detect.tflite'):
        #        GRAPH_NAME = 'edgetpu.tflite'       

        # Get path to current working directory
        CWD_PATH = os.getcwd()

        # Path to .tflite file, which contains the model that is used for object detection
        PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

        # Path to label map file
        PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

        # Load the label map
        with open(PATH_TO_LABELS, 'r') as f:
            labels = [line.strip() for line in f.readlines()]

        # Load the Tensorflow Lite model.
        # If using Edge TPU, use special load_delegate argument
        #if use_TPU:
        #    interpreter = Interpreter(model_path=PATH_TO_CKPT,
        #                              experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
        #    print(PATH_TO_CKPT)
        #else:
        interpreter = Interpreter(model_path=PATH_TO_CKPT)

        interpreter.allocate_tensors()

        # Get model details
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        height = input_details[0]['shape'][1]
        width = input_details[0]['shape'][2]

        floating_model = (input_details[0]['dtype'] == np.float32)

        input_mean = 128
        input_std = 128
        print(floating_model)

        # Check output layer name to determine if this model was created with TF2 or TF1,
        # because outputs are ordered differently for TF2 and TF1 models
        outname = output_details[0]['name']

        if ('StatefulPartitionedCall' in outname): # This is a TF2 model
            boxes_idx, classes_idx, scores_idx = 1, 3, 0
        else: # This is a TF1 model
            boxes_idx, classes_idx, scores_idx = 0, 1, 2

        # Initialize frame rate calculation
        #frame_rate_calc = 1
        #freq = cv2.getTickFrequency()

        # Initialize video stream
        #videostream = VideoStream(resolution=(imW,imH),framerate=5).start()
        #time.sleep(1)

        #for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        while True:

            # Start timer (for calculating frame rate)
            #t1 = cv2.getTickCount()

            # Grab frame from video stream
            #frame1 = videostream.read()
            YMAX = 0
            for sent_frames in inP:
                frame_in = sent_frames.recv()
                if frame_in is not None:
                    frame_rgb = frame_in[0]
                    # Acquire frame and resize to expected shape [1xHxWx3]
                    #frame = frame1.copy()
                    #frame_rgb = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
                    frame_resized = cv2.resize(frame_rgb, (width, height))
                    input_data = np.expand_dims(frame_resized, axis=0)
                    #input_data = np.expand_dims(frame_rgb, axis=0)

                    # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
                    if floating_model:
                        input_data = (np.float32(input_data) - input_mean) / input_std

                    # Perform the actual detection by running the model with the image as input
                    interpreter.set_tensor(input_details[0]['index'],input_data)
                    interpreter.invoke()

                    # Retrieve detection results
                    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
                    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
                    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects
                    
                    detected_objects = []
                                        
                    # Loop over all detections and draw detection box if confidence is above minimum threshold
                    for i in range(len(scores)):
                        flag = False
                        
                        ymin = int(max(1,(boxes[i][0] * imH)))
                        ymax = int(min(imH,(boxes[i][2] * imH)))
                        YMAX = abs((ymax - ymin))
                        
                        if ('ped' in labels[int(classes[i])] and YMAX > 40 and scores[i] >= 0.75):
                            flag = True
                        elif 'priority' in labels[int(classes[i])] and scores[i] >= 0.25 and YMAX > 30:
                            flag = True
                        elif 'parking' in labels[int(classes[i])] and scores[i] >= 0.20 and YMAX > 30:
                            flag = True
                        elif 'cross' in labels[int(classes[i])] and YMAX > 30 and scores[i] >= 0.55:
                            flag = True
                        elif 'stop' in labels[int(classes[i])] and scores[i] >= 0.25:
                            flag = True
                    
                        #if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
                        if flag:
                            # Get bounding box coordinates and draw box
                            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                            ymin = int(max(1,(boxes[i][0] * imH)))
                            xmin = int(max(1,(boxes[i][1] * imW)))
                            ymax = int(min(imH,(boxes[i][2] * imH)))
                            xmax = int(min(imW,(boxes[i][3] * imW)))
                            
                            #YMAX = boxes[i][2] * imH
                            YMAX = abs((ymax - ymin))
                            
                            
                            cv2.rectangle(frame_rgb, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                            # Draw label
                            object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                            label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
                            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                            cv2.rectangle(frame_rgb, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                            cv2.putText(frame_rgb, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text
                            
                            '''
                            if ('ped' in label and YMAX > 40 and scores[i] >= 0.75):
                                detected_objects.append(label)
                            elif 'priority' in label and YMAX > 35:
                                detected_objects.append(label)
                            elif 'parking' in label and YMAX > 35:
                                detected_objects.append(label)
                            elif 'cross' in label and YMAX > 30 and scores[i] >= 0.55:
                                detected_objects.append(label)
                            elif 'stop' in label and scores[i] >= 0.45:
                                detected_objects.append(label)
                            '''
                            detected_objects.append(label)

                    for outPipe in outP:
                       outPipe.send([frame_rgb, detected_objects, YMAX])

            