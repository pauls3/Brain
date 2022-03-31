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
        self.CURRENT_STATE = "null"
        self.PARKING = True
        self.STOP_SIGN = True
        self.PRIORITY = True
        self.PEDESTRIAN = True

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
        commands = [r, l, r, l]
        #commands = [f, r, r, r, f, stop, straight, l, l, l, f, straight, stop]
        try:           
            self._start_pid(outPs, )
            time.sleep(1)

            flag = True
           
            while flag:              
                
                # ignore commands if currently parking
                if self.CURRENT_STATE == 'null':
                    for ii in self.inPs:
                        if 'pedestrian_crossing' in ii.recv():
                            self.CURRENT_STATE = 'pedestrian_crossing'
                            self._crosswalk_wait(outPs,)                            
                        elif 'right_of_way' in ii.recv() and self.PRIORITY:
                            self.CURRENT_STATE = 'right_of_way'
                            self._right_of_way(outPs,)
                            self.PRIORITY = False
                        elif 'parallel_park' in ii.recv() and self.PARKING:
                            #self.PARKING = True
                            self.CURRENT_STATE = 'parking'
                            self._parallel_park(outPs, )
                            self.PARKING = False
                        elif 'stop_sign' in ii.recv() and self.STOP_SIGN:
                            #self.STOP_SIGN = True
                            self.CURRENT_STATE = 'stop_sign'
                            self._stop_sign(outPs, self.inPs, )
                            self.STOP_SIGN = False
                        elif 'stop' in ii.recv():
                            self._stop_straight(outPs, )
                        elif 'forward_test' in ii.recv():
                            self._forward_test(outPs, )
                        else:
                            #for cmd in ii.recv():
                            #    self._send_command(outPs, cmd)
                            for ii in commands:
                                self._send_command(outPs, ii)
                else:
                    continue
                
                #time.sleep(0.5)

       
        except Exception as e:
            print(e)
    
    
    
    # sends commands for the car to stop and straighten out
    def _forward_test(self, outPs):
        cmds = ['forward', 'stop']
        for ii in cmds:
            self._send_command(outPs, ii,)
            time.sleep(5)
    
    
    # sends commands for the car to stop and straighten out
    def _stop_straight(self, outPs):
        cmds = ['stop', 'straight']
        for ii in cmds:
            self._send_command(outPs, ii,)
            time.sleep(1)
                  
                  
    def _start_pid(self, outPs):
       print('**************************')
       print('Starting PID')
       print('**************************')
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
        #print(command)
        command_ =  self.rcBrain.getMessage(command)
        if command_ is not None:
           encode = json.dumps(command_).encode()
           decode = encode.decode()
           command = json.loads(decode)
           for ii in range(0,1):
               for outP in outPs:
                   outP.send(command)
           
           #time.sleep(1)
          
          
          
    def _crosswalk_wait(self, outPs):
        #print('**************************')
        #print('CROSSWALK PROCEDURE')
        #print('**************************')        
        commands = ['stop']
        
        #time.sleep(2)
        
        for ii in commands:
            self._send_command(outPs, ii,)

        
        self.CURRENT_STATE = 'null'
                   
    def _right_of_way(self, outPs):
        print('**************************')
        print('PRIORITY SIGN PROCEDURE')
        print('**************************')
        #commands = ['stop', 'right', 'reverse', 'stop', 'straight', 'left', 'left', 'reverse', 'stop', 'straight', 'right', 'right', 'forward', 'stop']
        
        commands = ['straight', 'forward', 'right', 'forward', 'straight', 'forward']
        
        
        # first stop and straighten out
        #self._stop_straight(outPs,)
        #time.sleep(2)
        
        flag = 0
        # start stop sign manuever
        for ii in commands:
            
            print(ii)
            if ii == 'forward' and flag == 0:
                self._send_command(outPs, ii,)
                time.sleep(3)
                flag = flag + 1
            elif ii == 'forward' and flag == 1:
                self._send_command(outPs, ii,)
                time.sleep(7)
                flag = flag + 1
            elif ii == 'forward' and flag == 2:
                #self._send_command(outPs, ii,)
                time.sleep(2)
                flag = flag + 1
            else:
                self._send_command(outPs, ii,)
                time.sleep(1)
                
        
        self.CURRENT_STATE = 'null'
    


    def _stop_sign(self, outPs, inPs):
        print('**************************')
        print('STOP SIGN PROCEDURE')
        print('**************************')
        #commands = ['stop', 'right', 'reverse', 'stop', 'straight', 'left', 'left', 'reverse', 'stop', 'straight', 'right', 'right', 'forward', 'stop']
        
        commands = ['straight', 'forward', 'left', 'forward', 'straight', 'forward']
        
        
        # first stop and straighten out
        self._stop_straight(outPs,)
        seconds = 3
        #for ii in inPs:
        #    seconds = int(ii.recv()[1])
        
        
        #self._send_command(outPs, 'forward',)
        time.sleep(seconds)
    
        
        flag = 0
        # start stop sign manuever
        for ii in commands:
            print(ii)
            self._send_command(outPs, ii,)
            if ii == 'forward' and flag == 0:
                time.sleep(4)
                flag = flag + 1
            elif ii == 'forward' and flag == 1:
                time.sleep(10)
                flag = flag + 1
            elif ii == 'forward' and flag == 2:
                time.sleep(5)
                flag = flag + 1
            else:
                time.sleep(1)
                
        
        self.CURRENT_STATE = 'null'


    def _parallel_park(self, outPs):
        print('**************************')
        print('Parallel Parking')
        print('**************************')
        in_commands  = ['right', 'reverse', 'stop', 'left', 'reverse', 'stop', 'right', 'forward', 'stop', 'straight']
        #out_commands = ['left', 'left', 'forward', 'stop', 'straight', 'right', 'right', 'forward', 'straight', 'stop']
        out_commands = ['right', 'reverse', 'stop', 'left', 'forward', 'straight', 'right', 'forward', 'straight']
        
        
        
        # first stop and straighten out
        self._stop_straight(outPs,)
        
        flag = True
        # start paralle parking
        for ii in in_commands:
            self._send_command(outPs, ii,)
            #print(ii)
            if ii == 'reverse' and flag:
                time.sleep(6)
                flag = False
            elif ii == 'reverse' and not flag:
                time.sleep(3)
            elif ii == 'forward':
                time.sleep(2)
            else:
                time.sleep(2)
        
        time.sleep(5)
        
        flag = True
        for ii in out_commands:
            self._send_command(outPs, ii,)
            #print(ii)
            if ii == 'forward' and flag:
                time.sleep(2)
                flag = False
            elif ii == 'reverse':
                time.sleep(2)
            elif ii == 'forward' and not flag:
                time.sleep(1)
            elif ii == 'right'  and not flag:
                time.sleep(4)
            else:
                time.sleep(2)
       
       
        #self._stop_straight(outPs,)
        #time.sleep(2)
        self.CURRENT_STATE = 'null'