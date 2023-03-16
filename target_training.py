# -*- coding: utf-8 -*-
"""
Created on Wed Nov  9 09:47:19 2022

@author: Samuel Senneka
"""

import serial
import struct
import time
import numpy as np
import random
from dlclive import Processor
import math

class training(Processor):
    #__init__ performs initializes the connection to the DLC model. This is
    #where all global variables are initialized and all connections are setup.
    #Variables that are not prefaced with "self" will not be accessible in
    #other functions in the processor class. (Hint: if there are arrays
    #that you would like to access without remaking it each time the function
    #is called, you should define them here. Other variables can be helpful to
    #define if you don't want them reset each time the function is called.)
    def __init__(self, lik_thresh=0.5, baudrate = int(9600)):
    
        super().__init__()

        self.waiting = True
        #Serial Connections to Arduinos: Zero controls Visual cues and Leo controls reward dispensing.
        self.leo = serial.Serial('COM9', baudrate, timeout = 0)
        self.zero = serial.Serial('COM11', baudrate, timeout = 0)
        self.lik_thresh = lik_thresh #Global definition of likelihood threshold
        self.trial_num = 1 #Current trial number as well as total number of trials at the end
        self.targ_hit = True
        self.trial_start = 0 #variable for start time of each trial
        self.trial_current = 0
        self.successes = 0 #number of successful trials
        
        
        targ_1 = [[135,180], b'A']
        targ_2 = [[135,540], b'B']
        targ_3 = [[405,180], b'C']
        targ_4 = [[405,540], b'D']
        
        self.target_list = [targ_1, targ_2, targ_3, targ_4]
        self.curr_targ = []
        elec = np.array([i for i in range(1, 13)])
        nelec = len(elec)
        kappa = 8
        PD = (elec - 1) * (360/(0.5 * nelec))
        alldeg = np.array([i for i in range(0, 361)])
        baselinef = 200
        s = (nelec,361)
        self.TC = np.zeros(s)
        for i in range(nelec):
            for j in range(len(alldeg)):    
                self.TC[i,j] = (1/(2 * math.pi)) * math.exp(kappa * math.cos((alldeg[j] - PD[i]) * math.pi / 180))
            self.TC[i,:] = baselinef * self.TC[i,:] / max(self.TC[i,:])
            
        bits = 16
        channels = 2

        pygame.mixer.pre_init(44100, -bits, 0)
        pygame.init()

 
        self.channel_l = pygame.mixer.Channel(0)
        self.channel_r = pygame.mixer.Channel(1)
        
        print("Setup Complete.")

        
    def close_serial(self):
        self.leo.close()
        self.zero.close()
        
    def switch_led_on(self):
        self.leo.reset_input_buffer()
        self.out_time.append(time.time())
        self.leo.write(b"H")
        
    def switch_led_off(self):
        self.led.reset_input_buffer()
        self.out_time.append(time.time())
        self.led.write(b"O")

    def distance(self,p1,p2):
        x_dist = p1[0]-p2[0]
        y_dist = p1[1]-p2[1]
        dist = math.sqrt((x_dist)**2+(y_dist)**2)
        distances = [x_dist, y_dist, dist]
        return distances
    
    def play_sound(self, angle, dist):
        bits = 16
        channels = 2
        duration = .1
        sample_rate = 44100
        n_samples = int(round(duration*sample_rate))
        buf_l = numpy.zeros((n_samples,2), dtype = numpy.int16)
        buf_r = numpy.zeros((n_samples,2), dtype = numpy.int16)
        max_sample = 2**(bits-1) - 1    
        frequency = 13000 - 17*angle
        volume_r = .05+1*(max_dist - dist)
        volume_r = .2-1*(max_dist - dist)
        for s in range(n_samples):
            t = float(s)/sample_rate
            
            buf_l[s][0] = int(round(max_sample*math.sin(2*math.pi*frequency_l*t)))
            buf_l[s][1] = int(round(0 * max_sample*math.sin(2*math.pi*frequency_r*t)))
            buf_r[s][0] = int(round(0 * max_sample*math.sin(2*math.pi*frequency_l*t)))
            buf_r[s][1] = int(round(max_sample*math.sin(2*math.pi*frequency_r*t)))    
        left_chan = pygame.sndarray.make_sound(buf_l)
        right_chan = pygame.sndarray.make_sound(buf_r)
        self.channel_l.set_volume(volume_r)
        self.channel_r.set_volume(volume_l)
        self.channel_l.queue(left_chan)
        self.channel_r.queue(right_chan)
    
  
#Pose indices       
#pose: Main array: Pixel coordinates for each point on the mouse listed below.
#                  Sub-arrays: 0: x-coordinate, 1: y-coordinate, 2: confidence level
    #0 - Nose
    #1 - Head Center
    #2 - Right Ear
    #3 - Left Ear
    #4 - Right Hip
    #5 - Left Hip
    #6 - Tail Base     

       
        
    def process(self, pose, **kwargs):
        #print(pose[1][0],pose[1][1])

        if kwargs['record']:
        
            if self.targ_hit == True:
                self.curr_targ = random.choice(self.target_list)
                print("Target: ",self.curr_targ)
                self.zero.write(curr_targ[1])
                self.targ_hit = False
                self.trial_start = time.time()
                
            #This section of code calculates x,y and direct distances from the
            #mouse's nose(n) to the center of its head(h) and from both those 
            #points to the target(t).                
            nt_dists = self.distance(pose[0],curr_targ[0])
            ht_dists = self.distance(pose[1],curr_targ[0])
            nh_dist = self.distance(pose[0],pose[1])[2]
            pi_angle = math.acos((ht_dists[2]**2+nh_dist**2-nt_dists[2]**2)/(2*ht_dists[2]*nh_dist))
            deg_angle = pi_angle *(180.0 / math.pi)
            if ((time.time()-self.trial_start)%.1 == 0):
                self.get_sound(deg_angle, ht_dists[2])

            if (((pose[1][0] > self.curr_targ[0][0]) and (pose[1][0] < self.curr_targ[0][1])) and ((pose[1][1] > self.curr_targ[1][0]) and (pose[1][1] < self.curr_targ[1][1]))):
                self.zero.write(b'O')
                self.targ_hit = True
                self.trial_num += 1
                self.successes += 1
                print("Trials Completed: ", self.trial_num)
                self.switch_led_on()
                print(self.targ_hit)
                print("Successes: ", self.successes)
                print("Fails: ", self.fails)
                complete_time = time.time()
                time.sleep(30)
                self.switch_led_off()
            if (time.time() - self.trial_start) >= 60 and (self.targ_hit == False):
                self.switch_led_off()
                self.targ_hit = True
                self.trial_num += 1
                self.fails += 1
                print("Trials Completed: ",self.trial_num)
                time.sleep(10)

        return pose

    






    def save(self, filename):

        ### save stim on and stim off times

        filename += ".csv"
        trial_num = np.array(self.trial_num)
        successes = np.array(self.successes)
        fails = np.array(self.fails)
        try:
            np.savez(
                filename, pose = pose, trial_num = trial_num, successes  = successes, fails = fails
            )
            save_code = True
        except Exception:
            save_code = False

        return save_code
                

        
