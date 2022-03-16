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

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        super(RemoteControlReceiverProcess,self).run()

        

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes. 
        """
        runCar = Thread(name='RunCar',target = self._run_car, args = (self.outPs, ))
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


    def _run_car(self, outPs):       
        f = 'forward'      
        b = 'reverse'      
        l = 'left'
        r = 'right'
        stop = 'stop'
        straight = 'straight'
       

        #commands = [f, s, b, s, f, s, l, l, r, r]
        try:           
            self._start_pid(outPs, )
            time.sleep(1)

        #    for ii in commands:
            while True:
                # take car commands (global)
                if self.COMMAND != "null":
                    print("null")

       
        except Exception as e:
            print(e)
    

    def _start_pid(self, outPs):
       command_ =  self.rcBrain.getMessage('p.p')
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
    
           
    def _send_command_thread(self, inP):
        """Transmite the command to the remotecontrol receiver. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe. 
        """
        while True:
            key = inP.recv()

            command = self.rcBrain.getMessage(key)
            if command is not None and command != 'stop':
                command = json.dumps(command).encode()

                self.client_socket.sendto(command,(self.serverIp,self.port))
