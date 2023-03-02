# -*- coding: utf-8 -*-
"""
Created on Thu Mar  2 12:46:20 2023

@author: ssenneka
"""
import pygame
import serial
import time
import numpy as np
import random
from dlclive import Processor
import math

class Habituation(Processor):

    def __init__(self, lik_thresh=0.5, baudrate = int(9600)):
    
        super().__init__()

        #Serial Connections to Arduinos: Zero controls Visual cues and Leo controls reward dispensing.
        self.leo = serial.Serial('COM9', baudrate, timeout = 0)
        self.trial_num = 1 #Current trial number as well as total number of trials at the end
        self.trial_start = 0 #variable for start time of each trial
        self.trial_current = 0
        self.successes = 0 #number of successful trials
        self.reward_dispense = True
        self.start_times = []
        self.pos = []
        bits = 16
        channels = 2
        pygame.mixer.pre_init(44100, -bits, channels)
        pygame.init()
        duration = 1
        frequency_l = 12000
        frequency_r = 12000
        sample_rate = 44100
        n_samples = int(round(duration*sample_rate))
        buf = np.zeros((n_samples,2), dtype = np.int16)
        max_sample = 2**(bits-1) - 1
        for s in range(n_samples):
            t = float(s)/sample_rate
            buf[s][0] = int(round(max_sample*math.sin(2*math.pi*frequency_l*t)))
            buf[s][1] = int(round(max_sample*math.sin(2*math.pi*frequency_r*t)))

        self.sound = pygame.sndarray.make_sound(buf)

        
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

    
    # def dispense_reward(self):
    #     print("Waiting for nose poke")
    #     while (time.time - self.complete_time) < 30 & self.dispensed == False:
    #         if int(ser.readline().decode().strip()) > 700:
                
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
            self.pos.append(pose)
            if self.reward_dispense == True:
                self.sound.play(loops = 0)
                self.trial_start = time.time()
                self.leo.write(b"H")
                self.reward_dispense = False
                self.start_times.append(self.tiral_start)
                
            if (time.time() - self.trial_start) >= 50:
                self.leo.write(b"O")
                self.reward_dispense ==True
                time.sleep(10)
                
            

        return pose
    
    def save(self, filename):
    
            ### save stim on and stim off times
    
            filename += ".npy"
            pos = self.pos
            trial_num = self.trial_num
            start_times = self.start_times
            try:
                np.savez(
                    filename, pos = pos, trial_num = trial_num, start_times = start_times
                )
                save_code = True
            except Exception:
                save_code = False
    
            return save_code