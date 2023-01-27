# -*- coding: utf-8 -*-
"""
Created on Wed Nov  9 09:47:19 2022

@author: mcmadmx
"""

import serial
import struct
import time
import numpy as np
import random
from dlclive import Processor
from multiprocessing import Pipe, Process
import math

class training(Processor):
    def __init__(self, lik_thresh=0.5, baudrate = int(9600)):

        super().__init__()
        print("Initializing")
        self.waiting = True
        self.leo = serial.Serial('COM9', baudrate, timeout = 0)
        self.zero = serial.Serial('COM11', baudrate, timeout = 0)
        self.lik_thresh = lik_thresh
        self.out_time = []
        self.in_time = []
        self.trial_num = 1
        self.targ_hit = True
        self.trial_start = time.time()
        self.trial_current = time.time()
        self.fails = 0
        self.successes = 0
        targ_1 = [[0,270],[0,360],[135, 180]]
        targ_2 = [[0,270],[360,720],[135,540]]
        targ_3 = [[270,540],[0,360],[405, 180]]
        targ_4 = [[270,540], [360,720],[405,540]]
        T1 = [[460,540],[400,540], b"R"]
        T2 = [[260,350],[400,540], b"L"]
        self.target_list = [targ_1, targ_2, targ_3, targ_4]
        self.target_list_2 = [T1, T2]
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
        print("Setup Complete.")

        
    def close_serial(self):
        self.ser.close()
        
    def switch_led(self):
        self.ser.reset_input_buffer()
        
        self.out_time.append(time.time())
        self.ser.write(b"H")
    def switch_led_off(self):
        self.ser.reset_input_buffer()
        self.out_time.append(time.time())
        self.ser.write(b"O")
    
    # def dispense_reward(self):
    #     print("Waiting for nose poke")
    #     while (time.time - self.complete_time) < 30 & self.dispensed == False:
    #         if int(ser.readline().decode().strip()) > 700:
                
        
#0 - Nose
#1 - Head Center
#2 - Right Ear
#3 - Left Ear
#4 - Right Hip
#5 - Left Hip
#6 - Tail Base     

  #  def stim(pose):
       
        
    def process(self, pose, **kwargs):
        #print(pose[1][0],pose[1][1])

        if kwargs['record']:
        
            if self.targ_hit == True:
                self.curr_targ = random.choice(self.target_list_2)
                print("Target: ",self.curr_targ)
                
                self.targ_hit = False
                self.ser.write(self.curr_targ[2])
                self.trial_start = time.time()
                
            
            print(pose[1][0],pose[1][1])

            

            if (((pose[1][0] > self.curr_targ[0][0]) and (pose[1][0] < self.curr_targ[0][1])) and ((pose[1][1] > self.curr_targ[1][0]) and (pose[1][1] < self.curr_targ[1][1]))):
                self.targ_hit = True
                self.trial_num += 1
                self.successes += 1
                print("Trials Completed: ", self.trial_num)
                self.switch_led()
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

        
        
        # while num_trial < 100:
            
        #     target = targ_1

        #     print(target)
        #     while ((pose[6][0] not in target[0]) or (pose[6][1] not in target[1])):
        #         pose = pose

        #     self.switch_led()    
        #     num_trial += 1
        #     print(num_trial) 

        # return pose






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
                

        
