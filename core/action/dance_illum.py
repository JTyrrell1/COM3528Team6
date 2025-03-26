#!/usr/bin/env python3
import os
import rospy            # ROS Python interface
import random
from std_msgs.msg import UInt32MultiArray

class IllumPublisher(object):
    
    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Used to sync to the music 
        self.bpm = 0
        self.t_4bars = 0.0

        # Used to denote what type of lights to do
        self.genre = ""
        self.flash = True
        self.colours = []
        self.color1 = "red"
        self.color2 = "blue"
        self.colours = ["red","blue","orange","yellow","purple","green"]
        self.next_lights = 0
        self.flash_time = 0
        self.transition_start = 0
        self.color_change = UInt32MultiArray()
        # Subscribers and Publishers
        self.illumination = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )

    def set_tempo(self,tempo):
        self.bpm = 60/float(tempo)

    def set_colours_by_genre(self):
        if self.genre == "pop":
            self.colours = ["red","pink","purple","yellow","orange","white"]
        elif self.genre == "soul":
            self.colours = ["purple","blue","green"]
        elif self.genre == "electronic":
            self.colours = ["blue","red","pink"]
        elif self.genre == "rock":
            self.colours = ["black","blue",]
        elif self.genre == "blues":
            self.colours = ["blue","blue"]
        elif self.genre == "metal":
            self.colours = ["black","white"]
        elif self.genre == "classical":
            self.colours = ["yellow","orange","white"]
        else:
            self.colours =["red","blue","orange","yellow","purple","green"]

    def get_rgbs(self, colour_name):
        if colour_name == "red":
                return [255,0,0]
        elif colour_name == "orange":
                return [255,165,0]
        elif colour_name == "yellow":
                return [255,255,0]
        elif colour_name == "green":
                return [0,255,64]
        elif colour_name == "blue":
                return [0,153,255]
        elif colour_name == "purple":
                return [255,102,255]
        elif colour_name == "pink":
                return [255,105,180]
        elif colour_name == "black":
                return [0,0,0]
        elif colour_name == "white":
                return [255,255,255]
        elif colour_name == "black":
                return [0,0,0]
        else :
             # Check if this is no light or black lol 
             return [0,0,0]

    def set_all_lights(self, colour):
        colour_rgb = self.get_rgbs(colour)
        self.color_change = UInt32MultiArray()

        color = self.cvt_to_unsigned_int(colour_rgb) 

        self.color_change.data = [
            color,
            color,
            color,
            color,
            color,
            color
        ]
        #self.illumination.publish(self.color_change)
        #rospy.sleep(0.02)

    def cvt_to_unsigned_int(self,colour_rgb):
        color_detail = (int(colour_rgb[0]), int(colour_rgb[1]), int(colour_rgb[2]))
        color = '0xFF%02x%02x%02x'%color_detail
        color = int(color, 16)
        return color 

    def transition_lights(self, colour1, colour2, transition_length,t,t0):
        # MAX TRANSITION LENGTH = 50 
        colour1_rgb = self.get_rgbs(colour1)
        colour2_rgb = self.get_rgbs(colour2)

        step_time = 0.1    #secs
        self.color_change = UInt32MultiArray()
    
        no_of_steps = int(transition_length / step_time)

        r_step_amt = (colour2_rgb[0]-colour1_rgb[0]) / no_of_steps
        g_step_amt = (colour2_rgb[1]-colour1_rgb[1]) / no_of_steps
        b_step_amt = (colour2_rgb[2]-colour1_rgb[2]) / no_of_steps 

        t = t % transition_length
        transition_point = int(t / step_time)
        if t < transition_length / 2:
            new_r = int(colour1_rgb[0] + (r_step_amt*2*transition_point))
            new_g = int(colour1_rgb[1] + (g_step_amt*2*transition_point))
            new_b = int(colour1_rgb[2] + (b_step_amt*2*transition_point))
        else:
            transition_point = int((t/2) / step_time)
            new_r = int(colour2_rgb[0] - (r_step_amt*2*transition_point))
            new_g = int(colour2_rgb[1] - (g_step_amt*2*transition_point))
            new_b = int(colour2_rgb[2] - (b_step_amt*2*transition_point))

        new_colour = (new_r,new_g,new_b)
        color = self.cvt_to_unsigned_int(new_colour)

        self.color_change.data = [color, color, color, color, color, color]
        #self.illumination.publish(self.color_change)
        #rospy.sleep(step_time)
        
        #print("done")

    def flashing_lights(self, colour1, colour2, flash_length,t_cur,t0):
        colour1_rgb = self.get_rgbs(colour1)
        colour2_rgb = self.get_rgbs(colour2)
        self.color_change = UInt32MultiArray()
        cur_command = self.genre
        t = t_cur - self.flash_time
        if t < flash_length:
            color = self.cvt_to_unsigned_int(colour1_rgb)
        elif t < flash_length * 2:
            color = self.cvt_to_unsigned_int(colour2_rgb)
        else: 
            color = self.cvt_to_unsigned_int(colour1_rgb)
            self.flash_time = t_cur

        self.color_change.data = [color, color, color, color, color, color]    
        #self.illumination.publish(self.color_change)
        #rospy.sleep(0.02)
        self.flash = not self.flash
    
    # FUNCTIONS NEED TO MAKE
    # Searching for Song Lights 
    # Genre Specific Lights
    def set_new_light_mods(self):
        self.next_lights = random.randint(0,1)
        len_of_colours = len(self.colours) - 1
        color1 = random.randint(0,len_of_colours)
        color2 = random.randint(0,len_of_colours)
        self.color1 = self.colours[color1]
        self.color2 = self.colours[color2]

    def loop(self,t,t0):
        if self.genre == "done":
             self.flashing_lights("white","white",0.5,t,t0)
        if self.genre == "localising":
                self.flashing_lights("green","white",1,t,t0)
        elif self.genre == "listening":
                self.flashing_lights("red","red",1,t,t0)
        elif self.genre == "recording":
                self.flashing_lights("orange","pink",0.2,t,t0)
        elif self.genre == "processing":
                self.transition_lights("blue","yellow",1,t,t0)
                self.transition_lights("yellow","red",1,t,t0)
                self.transition_lights("red","blue",1,t,t0)

        elif self.bpm != 0.0 and self.genre != "done":
            if self.next_lights == 0:
                self.transition_point = 0
                self.flashing_lights(self.color1,self.color2,self.bpm,t,t0)
            if self.next_lights == 1:
                self.transition_lights(self.color1,self.color2,self.bpm*2,t,t0)



            if self.t_4bars <= t:
                print("4 bars")
                print(self.t_4bars)
                print(t)
                self.t_4bars = t + (32*self.bpm)
                self.set_new_light_mods()
        
        return self.color_change

    def red(self,t,t0):
        colour1_rgb = self.get_rgbs("red")
        color = self.cvt_to_unsigned_int(colour1_rgb)
        self.color_change.data = [color, color, color, color, color, color]    
        return self.color_change



        #self.command="all_lights"
        #if self.command == "all_lights":
        #      self.set_all_lights("red")
        #if self.command == "flashing lights":
        #      self.flashing_lights("red","blue",2)
        #if self.command == "transition":
        #      self.transition_lights("red","blue", 3)
        #      self.transition_lights("blue","red", 3)



#illum = IllumPublisher()
#t0 = rospy.get_time()
#while not rospy.is_shutdown():
#    t = rospy.get_time()
#    illum.loop(t,t0)