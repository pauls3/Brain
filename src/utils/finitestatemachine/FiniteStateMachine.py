import os
import copy

class FiniteStateMachine:
    
    # ===================================== INIT =========================================
    def __init__(self):
        """It's an example to process the keyboard events and convert them to commands for the robot.
        """

        self.steeringAngle = 0.0
        self.stopLineLocation = 0
        self.detectedObjects = []
        self.previousState = 'null'
        self.currentState = 'lane_centering'



    # ===================================== STATE DICT ===================================
    def _stateDict(self):
        """It generates a dictionary with the robot current states. 
        
        Returns
        -------
        dict
            It contains the robot current control state, speed and angle. 
        """
        # Set steering angle
        if self.currentState == 'lane_centering':
            '''
                set steering angle
            '''
        # Speed car up
        elif self.currentState == 'speed_up':
            '''
                speed up
            '''
        # Slow car down
        elif self.currentState == 'slow_down':
            '''
                slow down
            '''
        # Detected static car. Slow down and check for dashed lines
        elif self.currentState == 'static_car':
            '''
                slow down
                check for dashed lines
                enter overtake
            '''
        # Overtake car once near
        elif self.currentState == 'overtake':
            '''
                perform overtake manuver
                enter lane_centering
            '''
        # Follow car (OPTIONAL)
        elif self.currentState == 'follow':
            '''
                follow car
                enter ???
            '''
        # Roundabout approach. Slow down
        elif self.currentState == 'roundabout_approach':
            '''
                slow down
                enter roundabout_enter
            '''
        # Roundabout enter
        elif self.currentState == 'roundabout_enter':
            '''
                turn right
                lane centering
                find 1st or 2nd exit
                enter roundabout_exit
            '''
        # Roundabout exit
        elif self.currentState == 'roundabout_exit':
            '''
                speed up
                enter lane centering
            '''
        # highway entrance
        elif self.currentState == '':
            '''
                detected highway sign (entrance or exit)
                speed up car
                enter lane_centering
                **ignore barrier on left side
            '''
        # highway exit
        elif self.currentState == '':
            '''
               detected highway sign (entrance or exit)
               slow down car to reg speed
               enter lane_centering 
            '''
        # parking state (found sign)
        elif self.currentState == '':
            '''
                detected parking sign
                visibly slow down
                look for parallel parking
                if parallel parking available:
                    enter parallel_parking
            '''
        # parking state (paralle parking)
        elif self.currentState == 'parallel_parking':
            '''
                look for open spot
                park in open spot
                wait for ???
                exit
                enter lane_centering
            '''
        # Crosswalk Sign
        elif self.currentState == '':
            '''
                detected crosswalk sign
                slow down
                look for pedestrian, but continue lane centering
                if pedestrian detected:
                    enter pedestrian_cross
            '''
        # pedestrian crossing
        elif self.currentState == 'pedestrian_cross':
            '''
               if pedestrian near car:
                    stop car
                    wait for pedestrian to cross road/crosswalk
                    (turn camera L or R)
                    after ped crosses
                        enter lane_centering
            '''
        # priority sign
        elif self.currentState == 'priority':
            '''
                slow down
                approach intersection
                enter intersection_approach_priority
            '''
        # intersection (priority)
        elif self.currentState == 'intersection_approach_priority':
            '''
               stop
               enter check_intersection_priority
            '''
        # stop sign
        elif self.currentState == 'stop_sign':
            '''
                slow down
                approach intersection
                enter intersection_approach_stop
            '''
        # traffic light
        elif self.currentState == 'traffic_light_stop':
            '''
                detected traffic light
                slow down
                approach stop line
                stop
                enter 
                check for intersection (choose what turn to make)
                look at traffic light
                go on green
            '''
        # traffic light (check intersection)
        elif self.currentState == 'traffic_light_check_intersection':
            '''
                check intersection for turn to make
                determine turn to make
                enter traffic_light_wait_green
            '''
        # traffic light (wait for green)
        elif self.currentState == 'traffic_light_wait_green':
            '''
                focus camera on traffic light (right side)
                go on green (straigten camera)
            '''
        # intersection (stop sign)
        elif self.currentState == 'intersection_approach_stop':
            '''
                get close to stop line
                stop for 3 seconds minimum (longer to check intersection)
                enter check_intersection_stop
            '''
        # Check intersection for no entry sign or if car can't make a turn (3-way intersection)
        elif self.currentState == 'check_intersection_stop':
            '''
                choose best turn to make
                wait for cars to cross
            ''' 
        # Check intersection for no entry sign or if car can't make a turn (3-way intersection)
        elif self.currentState == 'check_intersection_priority':
            '''
                choose best turn to make
                enter intersection_crossing
            '''
        # intersection crossing
        elif self.currentState == 'intersection_crossing':
            '''
                cross intersection
                enter lane_centering
            '''
        # one way
        elif self.currentState == '':
            '''
               lane centering
               look for barrier
                    if barrier:
                        enter manuver_barrier 
            '''
        # manuver around barrier
        elif self.currentState == 'manuver_barrier':
            '''
                found barrier, so will manuver to left/right lane (where arrows are pointing)
                *** keep track which lane car moved to (Left or Right)
                lane centering until intersection found
                ***ONLY TURN LEFT OR RIGHT, based on the lane car is in.
                enter lane_centering
            '''
        # Intersection approach (NO SIGN)
        elif self.currentState == 'intersection_no_sign':
            '''
                slow down and stop at stop line
                enter intersection_right_way
            '''
        # intersection (no sign)
        elif self.currentState == '':
            '''
                No sign was found
                    Use "right priority rule"
                        right-of-way given to car to the right
                car is stopped at stop line
                look left and right
                make sure car on right crosses first
                enter intersection_crossing
            '''
        else:
            return None
            
        #print(data)
        return data
    # ========================= CALLBACK =================================================
    def getMessage(self,data):
        """ Generate the message based on the current pressed or released key and the current state. 
        
        Parameters
        ----------
        data : string
            The filtered and encoded keyboard event.
        Returns
        -------
        string
            The encoded command.
        """
        
        self._updateMotionState(data)

        self._updateSpeed()
        self._updateSteerAngle()
        self._updatePID(data)
        self._updateParameters(data)
        
        #print(self.currentState)
        
        return self._stateDict()        

    # ===================================== UPDATE SPEED =================================

    # If we keep the two buttons pressed the each states are active and the reached value remains constant.
    # When each two keys are released, it sets to zero the value??? to rapid reseting.
    def _updateSpeed(self):
        """Update the speed based on the current state and the keyboard event.
        """
        if self.currentState[4]:
            self.currentState[0] = False
            self.currentState[1] = False
            self.speed = 0
            return
        if self.currentState[8]:
            #self.speed = 9.0
            self.startSpeed = 9.0
            self.currentState[8] = False
            return

        #forward
        if self.currentState[0]:
            if self.speed == 0:
                self.speed = self.startSpeed
            else:
                self.currentState[0] = False
            # elif self.speed == -self.startSpeed:
            #     self.speed = 0
            # elif self.speed < self.configParam.maxSpeed:
            #     if  self.configParam.maxSpeed - self.speed < self.configParam.speedStep:
            #         self.speed = self.configParam.maxSpeed
            #     else:
            #         self.speed += self.configParam.speedStep
            #self.currentState[0] = False
        #backwards
        elif self.currentState[1]:
            if self.speed == 0:
                self.speed = - self.startSpeed
            elif self.speed == self.startSpeed:
                self.speed = 0
            else:
                self.currentState[1] = False
            # elif self.speed >  -self.configParam.maxSpeed:
            #     if  abs(self.configParam.maxSpeed + self.speed) < self.configParam.speedStep:
            #         self.speed = - self.configParam.maxSpeed
            #     else:
            #         self.speed -= self.configParam.speedStep

    # ===================================== UPDATE STEER ANGLE ===========================
    def _updateSteerAngle(self):
        """Update the steering angle based on the current state and the keyboard event.
        """
        # straighten out
        if self.currentState[7]:
                self.steerAngle = 0
        # left steer
        elif self.currentState[2] == True:
            '''
            if self.steerAngle == 0:
                self.steerAngle = -self.startSteerAngle
            elif self.steerAngle > -self.configParam.maxSteerAngle:
                if self.configParam.maxSteerAngle + self.steerAngle < self.configParam.steerAngleStep:
                    self.steerAngle = - self.configParam.maxSteerAngle
                else:
                    self.steerAngle -= self.configParam.steerAngleStep
            '''
            #if self.steerAngle == 0:
            if self.steerKey == 'leftleft':
                self.steerAngle = -self.configParam.maxSteerAngle
            else:
                self.steerAngle = -10.0
            
        #right steer    
        elif self.currentState[3] == True:
            '''
            if self.steerAngle == 0:
                self.steerAngle = self.startSteerAngle
            elif self.steerAngle < self.configParam.maxSteerAngle:
                if self.configParam.maxSteerAngle - self.steerAngle < self.configParam.steerAngleStep:
                    self.steerAngle = self.configParam.maxSteerAngle
                else:
                    self.steerAngle += self.configParam.steerAngleStep
            '''
            #if self.steerAngle == 0:
            #self.steerAngle = self.configParam.maxSteerAngle
            if self.steerKey == 'rightright':
                self.steerAngle = self.configParam.maxSteerAngle
            else:
                self.steerAngle = 10.0
        #elif not self.currentState[2] and not self.currentState[3]:
        #        self.steerAngle = 0

    # ===================================== UPDATE PARAMS ================================
    def _updateParameters(self, currentKey):
        """Update the parameter of the control mechanism (limits and steps).
        
        Parameters
        ----------
        currentKey : string
            Keyboard event encoded in string.
        """
        #--------------- RESET ---------------------------------
        if currentKey == 'p.stop':
            self.speed = 0.0
            #self.steerAngle = 0.0
            self.configParam = copy.deepcopy(self.default_configParam)
            
        #--------------- MAX SPEED ------------------------------
        elif currentKey == 'p.t':
            if self.configParam.maxSpeed < self.limit_configParam.maxSpeed:
                self.configParam.maxSpeed += self.parameterIncrement
        elif currentKey == 'p.g':
            if self.startSpeed < self.configParam.maxSpeed:
                self.configParam.maxSpeed  -= self.parameterIncrement
        #--------------- MAX STEER ANGLE ------------------------
        elif currentKey == 'p.y':
            if self.configParam.maxSteerAngle < self.limit_configParam.maxSteerAngle:
                self.configParam.maxSteerAngle += self.parameterIncrement
        elif currentKey == 'p.h':
            if self.startSteerAngle < self.configParam.maxSteerAngle:
                self.configParam.maxSteerAngle -= self.parameterIncrement
        #--------------- SPEED STEP ------------------------------
        elif currentKey == 'p.u':
            if self.configParam.speedStep < self.limit_configParam.speedStep:
                self.configParam.speedStep += self.parameterIncrement
        elif currentKey == 'p.j':
            if 0.1 < self.configParam.speedStep:
                self.configParam.speedStep -= self.parameterIncrement
        #--------------- STEER STEP ------------------------------
        elif currentKey == 'p.i':
            if self.configParam.steerAngleStep < self.limit_configParam.steerAngleStep:
                self.configParam.steerAngleStep += self.parameterIncrement
        elif currentKey == 'p.k':
            if 0.1 < self.configParam.steerAngleStep:
                self.configParam.steerAngleStep -= self.parameterIncrement


    def _updatePID(self, currentKey):
        """Update the parameter of the PID values.
        
        Parameters
        ----------
        currentKey : string
            Keyboard event encoded in string.
        """      
        #--------------- ACTIVATE/DEACTIVATE PID ------------------------------
        if currentKey == 'pid':
            #self.pida = not self.pida
            self.pida = True
            self.currentState[5] = True

        #--------------- KP PID ------------------------------
        elif currentKey == 'p.z':
            self.pids_kp += self.configParam.kpStep
            self.currentState[6] = True
        elif currentKey == 'p.x':
            self.pids_kp -= self.configParam.kpStep
            self.currentState[6] = True

        #--------------- KI PID ------------------------------
        elif currentKey == 'p.v':
            self.pids_ki += self.configParam.kiStep
            self.currentState[6] = True
        elif currentKey == 'p.b':
            self.pids_ki -= self.configParam.kiStep
            self.currentState[6] = True

        #--------------- KD PID ------------------------------
        elif currentKey == 'p.n':
            self.pids_kd += self.configParam.kdStep
            self.currentState[6] = True
        elif currentKey == 'p.m':
            self.pids_kd -= self.configParam.kdStep
            self.currentState[6] = True

    # ===================================== UPDATE MOTION STATE ==========================
    def _updateMotionState(self, currentKey):
        """ Update the motion state based on the current state and the pressed or released key. 
        
        Parameters
        ----------
        currentKey : string 
            Encoded keyboard event.
        """
        if currentKey == 'forward':
            self.currentState[0] = True
        elif currentKey == 'reverse':
            self.currentState[1] = True
        elif currentKey == 'left':
            self.currentState[2] = True
            self.steerKey = 'left'
        elif currentKey == 'leftleft':
            self.currentState[2] = True
            self.steerKey = 'leftleft'
        elif currentKey == 'right':
            self.currentState[3] = True
            self.steerKey = 'right'
        elif currentKey == 'rightright':
            self.currentState[3] = True
            self.steerKey = 'rightright'
        elif currentKey == 'stop':
            self.currentState[4] = True
        elif currentKey == 'straight':
            self.currentState[7] = True
            self.steerKey = ''
        elif currentKey == 'park':
            self.currentState[8] = True
        #elif currentKey == 'none':
            

        


