# -*- coding: utf-8 -*-
"""
Created on Thu Mar 23 16:54:22 2023

@author: ssenneka
"""

"""
Created on Wed Nov  9 09:47:19 2022

@author: Samuel Senneka
"""

import serial
import time
import numpy as np
import random
from dlclive import Processor
import math
import pygame


class training(Processor):
    ##__init__ performs initializes the connection to the DLC model. This is
    ##where all global variables are initialized and all connections are setup.
    ##Variables that are not prefaced with "self" will not be accessible in
    ##other functions in the processor class. (Hint: if there are arrays
    ##that you would like to access without remaking it each time the function
    ##is called, you should define them here. Other variables can be helpful to
    ##define if you don't want them reset each time the function is called.)
    def __init__(self, baudrate = int(9600)):  
        super().__init__()
        ##
        self.curr_targ = []                                                    #current target for mouse to reach for give trial
        self.fail = False                                                      #trial ended in failure if true
        self.last_tone = time.time()                                           #last time a sound was played
        self.leo = serial.Serial('COM9', baudrate, timeout = 0)                #Serial connection to Arduino Leonardo        
        self.nosepokes = 0                                                     #number of times reward was collected by nosepoking reward port
        self.pos = []                                                          #pose coordinates to save
        self.success = False                                                   #trial ended in success if true
        self.successes = 0                                                     #number of successful trials     
        self.targets = []                                                      #list of targets for each trial during session
        self.trial_end = []                                                    #End times of each trial
        self.trial_init = True                                                 #trial is being initiated if true
        self.trial_ip = False                                                  #trial is in progress if true
        self.trial_num = 1                                                     #Current trial number as well as total number of trials at the end
        self.trial_start = []                                                  #Start times of each trial
        self.waiting = True                                                    #Waiting for recording to start if True
        self.zero = serial.Serial('COM11', baudrate, timeout = 0)              #Serial connection to Arduino Zero
        self.target_list = [[[240,157], b'A'],                                 #list of targets
                            [[240,275], b'B'],
                            [[240,393], b'C'],
                            [[357,155], b'D'],
                            [[357,275], b'E'],
                            [[360,393], b'F'],
                            [[474,153], b'G'],
                            [[477,272], b'H'],
                            [[478,390], b'I']]  
# [[219,152], b'A'], #list of targets for four target scenario
# [[420,152], b'B'],
# [[220,351], b'C'],
# [[427,351], b'D']
                                          
        ##
        elec = np.array([i for i in range(1, 13)])                             #array with length of the number of stimulating electrodes
        nelec = len(elec)                                                      #number of stimulating electrodes
        kappa = 8                                                              #coefficient for evaluating tuning curves
        PD = (elec - 1) * (360/(0.5 * nelec))                                  #
        alldeg = np.array([i for i in range(0, 361)])                          # array ranging from 0 to 360 to indicate potential degrees of a circle
        baselinef = 200                                                        #baseline firing rate
        s = (nelec,361)                                                        #
        self.TC = np.zeros(s)                                                  #
        for i in range(nelec):
            for j in range(len(alldeg)):    
                self.TC[i,j] = (1/(2 * math.pi)) * math.exp(kappa * math.cos((alldeg[j] - PD[i]) * math.pi / 180))
            self.TC[i,:] = baselinef * self.TC[i,:] / max(self.TC[i,:])
        ##
        bits = 16                                                              #number of bits for the sound quality
        channels = 2                                                           #number of channels for the pygame mixer
        pygame.mixer.pre_init(44100, -bits, channels)                          #pre initialization of the pygame mixer
        pygame.init()                                                          #initialization of the pygame mixer
        duration = 1                                                           #length of reward tone
        frequency = 8000                                                       #frequency of reward tone
        sample_rate = 44100                                                    #number of samples per second
        n_samples = int(round(duration*sample_rate))                           #number of samples to collect
        buf = np.zeros((n_samples,2), dtype = np.int16)                        #array for sound wave to be created in
        max_sample = 2**(bits-1) - 1                                           #maximum number of samples
        for s in range(n_samples):
            t = float(s)/sample_rate
            buf[s][0] = int(round(.2*max_sample*math.sin(2*math.pi*frequency*t)))
            buf[s][1] = int(round(.2*max_sample*math.sin(2*math.pi*frequency*t)))
        self.channel_l = pygame.mixer.Channel(0)                               #left channel of the pygame mixer
        self.channel_r = pygame.mixer.Channel(1)                               #right channel of the pygame mixer
        self.reward_tone = pygame.sndarray.make_sound(buf)                     #reward tone
        self.reward_tone.set_volume(.055)                                      #volume for reward tone
        print("Setup Complete.")
        
        
    def close_serial(self):
        self.leo.close()
        self.zero.close()
       
        
    def reward_on(self):
        self.leo.reset_input_buffer()
        self.leo.write(b"H")
  
        
    def reward_off(self):
        self.leo.reset_input_buffer()
        self.leo.write(b"L")
    
    
    def target_led_on(self, target):
        self.zero.reset_input_buffer()
        self.zero.write(target)
    
    
    def led_board_off(self):
        self.zero.reset_input_buffer()
        self.zero.write(b'O')
    

    def distance(self,p1,p2):
        x_dist = p1[0]-p2[0]
        y_dist = p1[1]-p2[1]
        dist = math.sqrt((x_dist)**2+(y_dist)**2)
        distances = [x_dist, y_dist, dist]
        return distances
 
    
    def play_sound(self, angle, dist):
        bits = 16
        duration = .1
        sample_rate = 44100
        max_dist = 590
        n_samples = int(round(duration*sample_rate))
        buf_l = np.zeros((n_samples,2), dtype = np.int16)
        buf_r = np.zeros((n_samples,2), dtype = np.int16)
        max_sample = 2**(bits-1) - 1    
        if angle <= 180:
            frequency = 13000 - 17*angle
        if angle > 180:
            frequency = 9040 + 17*(angle-180)
        volume_r = .008+.00155*((max_dist - dist)/19.455)
        volume_l = .055-.00155*((max_dist - dist)/19.455)
        for s in range(n_samples):
            t = float(s)/sample_rate           
            buf_l[s][0] = int(round(max_sample*math.sin(2*math.pi*frequency*t)))
            buf_l[s][1] = int(round(0 * max_sample*math.sin(2*math.pi*frequency*t)))
            buf_r[s][0] = int(round(0 * max_sample*math.sin(2*math.pi*frequency*t)))
            buf_r[s][1] = int(round(max_sample*math.sin(2*math.pi*frequency*t)))    
        left_chan = pygame.sndarray.make_sound(buf_l)
        right_chan = pygame.sndarray.make_sound(buf_r)
        self.channel_l.set_volume(volume_r)
        self.channel_r.set_volume(volume_l)
        self.channel_l.play(left_chan)
        self.channel_r.play(right_chan)
        self.last_tone = time.time()
   

    def process(self, pose, **kwargs):
    #Pose indices      
        #0 - Nose
        #1 - Head Center
        #2 - Right Ear
        #3 - Left Ear
        #4 - Right Hip
        #5 - Left Hip
        #6 - Tail Base            

        if self.waiting == True:
            print("Waiting For user to press record")
            self.waiting = False            
        if kwargs['record']:
            ##
            if self.trial_init:
                self.curr_targ = random.choice(self.target_list)
                self.targets.append(self.curr_targ[0])
                print("Target: ",self.curr_targ)
                self.target_led_on(self.curr_targ[1])
                self.trial_init = False
                self.trial_ip = True
                self.success = False
                self.fail = False
                self.trial_start.append(time.time())
            ##This section of code is outside of any if statements because 
            ##these values are needed for all trial phases    
            self.pos.append(pose)
            nt_dists = self.distance(pose[0],self.curr_targ[0])
            ht_dists = self.distance(pose[1],self.curr_targ[0])
            nh_dist = self.distance(pose[0],pose[1])[2]
            if nh_dist == 0:
                nh_dist = 5
            pi_angle = math.acos((ht_dists[2]**2+nh_dist**2-nt_dists[2]**2)/(2*ht_dists[2]*nh_dist))
            deg_angle = pi_angle *(180.0 / math.pi)      
            print('position', pose[1])            
            print('distance:', ht_dists[2])                
            if self.trial_ip:      
                if ((time.time()-self.last_tone) >= .099):
                    self.play_sound(deg_angle, ht_dists[2])
                    self.last_tone = time.time()
                if (ht_dists[2] <= 65) and (time.time()-self.trial_start[len(self.trial_start)-1] >= 1):
                    self.trial_end.append(time.time())
                    self.trial_ip = False
                    self.success = True
                    self.led_board_off()
                    self.reward_on()
                    self.reward_tone.play(loops = 0)
                if (time.time()-self.trial_start[len(self.trial_start)-1] >= 30) and self.success == False:
                    self.fail = True                    
            if self.success:
                if time.time() - self.trial_end[len(self.trial_end) - 1] <= 20:
                    byte = self.leo.readline()
                    print(byte)
                    if byte == b'P':
                        self.trial_init = True
                        self.nosepokes += 1
                        self.success = False
                        self.trial_num += 1
                        self.successes += 1
                        print("Trials Completed: ", self.trial_num)
                        print("Successes: ", self.successes)
                        time.sleep(5)
                if time.time() - self.trial_end[len(self.trial_end)-1] > 20:
                    self.trial_init = True
                    self.success = False
                    self.reward_off()
                    self.trial_num += 1
                    self.successes += 1
                    print("Trials Completed: ", self.trial_num)
                    print("Successes: ", self.successes)
                    time.sleep(5)                                
            if self.fail:
                self.reward_off()
                self.led_board_off()
                self.trial_init = True
                print('Trials Completed:', self.trial_num)
                self.trial_num += 1
                print("Successes: ", self.successes)
                time.sleep(5)

        return pose

   






    def save(self, filename):

        ### save stim on and stim off times
        self.zero.write(b'O')
        self.leo.write(b'L')
        self.close_serial()
        filename += ".npy"
        trial_num = np.array(self.trial_num)
        successes = np.array(self.successes)
        targets = np.array(self.targets)
        nosepokes = np.array(self.nosepokes)
        trial_start = np.array(self.trial_start)
        trial_end = np.array(self.trial_end)
        pos = self.pos
        try:
            np.savez(
                filename, pos = pos, trial_num = trial_num, successes  = successes, targets = targets, nosepokes=nosepokes, trial_start = trial_start, trial_end = trial_end)
            save_code = True
        except Exception:
            save_code = False

        return save_code