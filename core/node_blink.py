import numpy as np
import time
import rospy
import node
import random
import miro2 as miro

# miro eyes are cosmetic_joints[2/3]
class NodeBlink(node.Node):

    def __init__(self, sys):

        node.Node.__init__(self, sys, "blink")

        self.blinking = False
        self.blink_mode = ""
        self.eyes = self.nodes.express.output.cosmetic_joints
        self.curr_eyes = 0 
        self.blink_period = self.pars.express.blink_period
        self.blink_mult = float(random.randint(1,2))
        self.blink_time = 0   
        self.eyelid_opening = 1  

        # 0 = open, 1 = closed
        self.dance_eye_openness = 0

    
    def tick(self):

        # Idea is that different blinking modes can be checked from actions for 
        # more control over how often miro blinks

        if self.blink_mode == "dancing":
            if self.blinking:
                if self.blink_period == 0:
                    if np.random.uniform() < self.pars.express.double_blink_prob:
                        self.blink_period = self.pars.express.double_blink_period
                        self.blink_mult = float(2)
                        self.blink_time = 0
                    else:
                        self.blink_period = self.pars.express.blink_period
                        self.blink_mult = float(1)
                        self.blink_time = 0

                else:
                    # implement blink
                    #t = self.blink_mult * self.blink_time / self.blink_period
                    #t = np.mod(t, 1)
                    self.blink_time += 1

                    # set eyelid position
                    if self.blink_time < 3:
                        self.eyelid_opening = self.dance_eye_openness
                    else: 
                        self.eyelid_opening = 1
                    # if blink is finished
                    if self.blink_time > 4:
                        self.blinking = False
                        # clear blink
                        self.blink_period = 0
                
                if random.randint(0,30) == 0:
                        self.blinking = True
                
                self.nodes.express.output.cosmetic_joints[2] = 1.0 - self.eyelid_opening
                self.nodes.express.output.cosmetic_joints[3] = 1.0 - self.eyelid_opening


        # e.g he blinks more when responding to a gpt prompt 
        if not self.blink_mode == "":
            if self.blinking:
                if self.blink_period == 0:
                    if np.random.uniform() < self.pars.express.double_blink_prob:
                        self.blink_period = self.pars.express.double_blink_period
                        self.blink_mult = float(2)
                        self.blink_time = 0
                    else:
                        self.blink_period = self.pars.express.blink_period
                        self.blink_mult = float(1)
                        self.blink_time = 0

                else:
                    # implement blink
                    #t = self.blink_mult * self.blink_time / self.blink_period
                    #t = np.mod(t, 1)
                    self.blink_time += 1

                    # set eyelid position
                    if self.blink_time < 3:
                        self.eyelid_opening = 0
                    else: 
                        self.eyelid_opening = 1
                    # if blink is finished
                    if self.blink_time > 4:
                        self.blinking = False
                        # clear blink
                        self.blink_period = 0

            # if not currently blinking, loop until active again
            else:
                if self.blink_mode == "listening":
                    if random.randint(0,20) == 0:
                        self.blinking = True
                elif self.blink_mode == "responding":
                    if random.randint(0,25) == 0:
                        self.blinking = True
                elif self.blink_mode == "gestures":
                    if random.randint(0,50) == 0:
                        self.blinking = True 
                else:
                    if random.randint(0,15) == 0:
                        self.blinking = True

            self.nodes.express.output.cosmetic_joints[2] = 1.0 - self.eyelid_opening
            self.nodes.express.output.cosmetic_joints[3] = 1.0 - self.eyelid_opening
                    
                    

    