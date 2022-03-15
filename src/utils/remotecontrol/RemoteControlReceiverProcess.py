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


class RemoteControlReceiverProcess(WorkerProcess):
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

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        #self._init_threads()
        #self._run_stop(self.outPs, )
        #self._run_stop(self.outPs, )
        super(RemoteControlReceiverProcess,self).run()

        

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes. 
        """
        #self.listener.daemon = self.daemon
        #self.threads.append(self.listener)
        #sendTh = Thread(name = 'SendCommandThread',target = self._send_command_thread, args=(self.lisBrR, ),daemon=self.daemon)
        
        
        # readTh = Thread(name='ReceiverCommandThread',target = self._read_stream, args = (self.outPs, ))
        runCar = Thread(name='RunCar',target = self._run_stop, args = (self.outPs, ))
        self.threads.append(runCar)
        

    # ===================================== READ STREAM ==================================
    def _read_stream(self, outPs):
        """Receive the message and forwards them to the SerialHandlerProcess. 
        
        Parameters
        ----------
        outPs : list(Pipe)
            List of the output pipes.
        """
        try:
            while True:

                
                bts, addr = self.server_socket.recvfrom(1024)

                bts     =  bts.decode()
                command =  json.loads(bts)

                for outP in outPs:
                    outP.send(command)

        except Exception as e:
            print(e)

        finally:
            self.server_socket.close()



    def _run_stop(self, outPs):
       #commands = ['p.w', 'p.w', 'p.s', 'p.s', 'p.s', 'p.s', 'p.r', 'p.r', 'p.r']
       #commands = ['p.w', 'p.s', 'p.s', 'p.w', 'p.r']
       
       f = 'p.w'       # forward
       b = 'p.s'       # reverse/back
       l = 'p.a'
       r = 'p.d'
       s = 'p.stop'
       

       commands = [f, s, b, s, f, s, l, l, r, r]
       try:           
           self._start_pid(outPs, )
           #time.sleep(2)
           for ii in commands:
               #self._start_pid(outPs, )
               self._send_command(outPs, ii, )

               time.sleep(2)
       
           print('---\n---')              
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
       time.sleep(2)
    
    
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
