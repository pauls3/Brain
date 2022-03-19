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

import json
import socket
import time

from threading       import Thread
from multiprocessing import Pipe

from src.templates.workerprocess import WorkerProcess
from src.utils.remotecontrol.RcBrainThread              import RcBrainThread
import cv2
import matplotlib.pyplot as plt


class RemoteControlReceiverProcess(WorkerProcess):
    COMMAND = "null"
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
        """Run on raspberry. It forwards the control messages received from socket to the serial handler
        
        Parameters
        ------------
        inPs : list(Pipe)
            List of input pipes (not used at the moment)
        outPs : list(Pipe) 
            List of output pipes (order does not matter)
        """

        super(RemoteControlReceiverProcess,self).__init__( inPs, outPs)
        self.rcBrain   =  RcBrainThread()
        self.lisBrR, self.lisBrS = Pipe(duplex=False)
        self.COMMAND = "null"
        self.PARKING = False

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        super(RemoteControlReceiverProcess,self).run()

        

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes. 
        """
        runCar = Thread(name='RunCar',target = self._run_car, args = (self.outPs, self.inPs, ))
        self.threads.append(runCar)
    

    # returns array of commands
    # for parking (for now...)
    def _get_commands(self, command):
        f = 'forward'       # forward
        b = 'reverse'       # reverse/back
        l = 'left'
        r = 'right'
        stop = 'stop'
        straight = 'straight'
        return [f, b]


    def _run_car(self, outPs, inP):       
        f = 'forward'      
        b = 'reverse'      
        l = 'left'
        r = 'right'
        stop = 'stop'
        straight = 'straight'
       
        jj = 0
        commands = []
        #commands = [f, r, r, r, f, stop, straight, l, l, l, f, straight, stop]
        try:           
            self._start_pid(outPs, )
            time.sleep(1)

            flag = True
           
            while flag:
                
                # ignore commands if currently parking
                if not self.PARKING:
                    for ii in self.inPs:
                        if 'parallel_park' in ii.recv():
                            self.PARKING = True
                            self._parallel_park(outPs, )
                        else:
                            for cmd in ii.recv():
                                self._send_command(outPs, cmd)
                
                #time.sleep(0.5)

       
        except Exception as e:
            print(e)
    
    
    
    # sends commands for the car to stop and straighten out
    def _stop_straight(self, outPs):
        cmds = ['stop', 'straight']
        for ii in cmds:
            self._send_command(outPs, ii,)
            time.sleep(1)
                  
                  
    def _start_pid(self, outPs):
       print('Starting PID')
       cmds = ['pid', 'stop','straight']
       for ii in cmds:
           command_ =  self.rcBrain.getMessage(ii)
           if command_ is not None:
              encode = json.dumps(command_).encode()
              decode = encode.decode()
              command = json.loads(decode)
              for outP in outPs:
                 outP.send(command)
        
       time.sleep(1)
    
    
    
    def _send_command(self, outPs, command):
        print(command)
        command_ =  self.rcBrain.getMessage(command)
        if command_ is not None:
           encode = json.dumps(command_).encode()
           decode = encode.decode()
           command = json.loads(decode)
           for ii in range(0,5):
               for outP in outPs:
                   outP.send(command)
        #time.sleep(3)
    

    def _parallel_park(self, outPs):
        print('**************************')
        print('Parallel Parking')
        print('**************************')
        commands = ['right', 'right', 'reverse', 'stop', 'straight', 'left', 'left', 'reverse', 'stop', 'straight', 'right', 'right', 'forward', 'stop']
        
        # first stop and straighten out
        self._stop_straight(outPs,)
        
        flag = True
        # start paralle parking
        for ii in commands:
            self._send_command(outPs, ii,)
            if ii == 'reverse' and flag:
                time.sleep(4)
                flag = False
            elif ii == 'reverse' and not flag:
                time.sleep(3)
            elif ii == 'forward':
                time.sleep(1)
            else:
                time.sleep(1.5)
        
        self._stop_straight(outPs,)