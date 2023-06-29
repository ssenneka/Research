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
import xipppy as xp


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
        self.dropout_session = False
        self.curr_targ = []
        self.drop_audio = False
        self.drop_visual = False
        self.dropout = [1,2,0,0,0,0,0,0,0,0]                                   #current target for mouse to reach for give trial
        self.dropout_trials = []
        self.drop = False
        self.fail = False                                                      #trial ended in failure if true
        self.last_tone = time.time()                                           #last time a sound was played
        self.pos = []                                                          #pose coordinates to save\
        self.pose_data = []
        self.session_length = 60
        self.session_start = time.time()
        self.sum_ttt = 0
        self.success = False                                                   #trial ended in success if true
        self.successes = 0                                                     #number of successful trials     
        self.targets = []                                                      #list of targets for each trial during session
        self.trial_end = []                                                    #End times of each trial
        self.trial_init = True                                                 #trial is being initiated if true
        self.trial_ip = False                                                  #trial is in progress if true
        self.trial_num = 0                                                     #Current trial number as well as total number of trials at the end
        self.trial_start = []                                                  #Start times of each trial
        self.trial_data = []
        self.waiting = True                                                    #Waiting for recording to start if True
        self.zero = serial.Serial('COM11', baudrate, timeout = 0)
        # self.target_list = [[[268,189], b'A'], #list of targets for four target scenario
        #                     [[467,186], b'B'],
        #                     [[270,389], b'C'],
        #                     [[470,387], b'D']]
                            
        self.target_list = [[[214,141], b'A'],                                 #list of targets
                            [[214,254], b'B'],
                            [[216,368], b'C'],
                            [[328,140], b'D'],
                            [[329,254], b'E'],
                            [[329,367], b'F'],
                            [[441,141], b'G'],
                            [[444,255], b'H'],
                            [[441,366], b'I']]
       
        self.target_size = 32

                                          
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
        # with xp.xipppy_open(use_tcp=True):
        #     self.elecchans = xp.list_elec('stim')
        
        print("Setup Complete.")
        
    # def biphasic_stim(elec, amp, frequency, repeats):
    #     seq0 = []
    #     frequency_len = len(frequency)
    #     elec_len = len(elec)
    #     repeat_len = len(repeats)
    #     with xp.xipppy_open(use_tcp=True):
           
    #         if xp.stim_get_res(0) <= 1: #change stim res
    #             xp.stim_enable_set(False)
    #             #stim_set_res(elec,res)
    #         xp.stim_enable_set(True)
    #         phase_length = 6
    #         interphase_length = 8
    #         interphase_amp = 0
    #         pol_phaseone = 1
    #         pol_phasetwo = -1
    #         pol_interphase = pol_phaseone
    #         cathseg=xp.StimSegment(phase_length,amp,pol_phaseone)
    #         ipi=xp.StimSegment(interphase_length, interphase_amp, pol_interphase)                       
    #         anseg=xp.StimSegment(phase_length,amp, pol_phasetwo)                       
    #         for i in range(0,len(frequency)):
    #             if frequency[i] == 0:
    #                 frequency[i] = 1
    #             period = round(1 / frequency[i] * 10**6 / 33.33)
    #             seq0.append(xp.StimSeq(elec[i], period, repeats[i], cathseg, ipi, anseg))#create overall sequence header defining electrode, period, repeats
    #     #This will stimulate on electrode 1 at 30 hz for one second (1000 ms)
    #         xp.StimSeq.send_stim_seqs(seq0) #send sequence to NIP
    #         time.sleep(0.0001)
   
    def close_serial(self):                                                    #Closes all serial connections
        self.zero.close()
       
        
    def reward_on(self):                                                       #Turns on led reward port
        self.zero.reset_input_buffer()
        self.zero.write(b"R")
  
        
    def reward_off(self):                                                      #Turns off reward port led and reward availability
        self.zero.reset_input_buffer()
        self.zero.write(b"L")
    
    
    def target_led_on(self, target):                                           #Turns on the desired area of LED board underneath cage floor
        self.zero.reset_input_buffer()
        self.zero.write(target)
    
    
    def led_board_off(self):                                                   #Turns off LED board
        self.zero.reset_input_buffer()
        self.zero.write(b'O')
    

    def distance(self,p1,p2):                                                  #Calculates disatnce between two points
        x_dist = p1[0]-p2[0]
        y_dist = p1[1]-p2[1]
        dist = math.sqrt((x_dist)**2+(y_dist)**2)
        distances = [x_dist, y_dist, dist]
        return distances

    def pose_parse(self, pose, trial_num):
        data = np.concatenate(([trial_num],pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],pose[6]))
        return data
    
    def play_sound(self, angle, dist):                                         #Takes angle and distance and creates and plays the desired sound for each channel
        bits = 16
        duration = .1
        sample_rate = 44100
        max_dist = 550
        n_samples = int(round(duration*sample_rate))
        buf_l = np.zeros((n_samples,2), dtype = np.int16)
        buf_r = np.zeros((n_samples,2), dtype = np.int16)
        max_sample = 2**(bits-1) - 1    
        if angle <= 180:
            frequency = 20000 - 55.555*angle
        if angle > 180:
            frequency = 10000 + 55.555*(angle-180)
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
   

    def process(self, pose, **kwargs):                                         #main process function. Contains 4 phases of training:
        if self.waiting == True:                                               #1) trial initiation, 2) trial in progress, 3)trial success and 4)trial failure phases
            print("Waiting For user to press record")                          #Pose indices
            self.waiting = False                                                   #0 - Nose
        if kwargs['record']:                                                       #1 - Head Center
            ##Trial Initiation Phase                                              #2 - Right Ear
            if self.trial_init:
                if self.dropout_session:                                                    #3 - Left Ear
                    self.drop = random.choice(self.dropout)            
                self.curr_targ = random.choice(self.target_list)                   #4 - Right Hip
                if self.drop != 1:
                    self.target_led_on(self.curr_targ[1])
                self.targets.append(self.curr_targ[0])                             #5 - Left Hip
                print("Target: ",self.curr_targ)                                   #6 - Tail Base
                self.trial_init = False
                self.trial_ip = True
                self.success = False
                self.fail = False
                self.trial_start.append(time.time())
            ##This section of code is outside of any if statements because 
            ##these values are needed for the following trial phases 
            self.pos.append(pose)
            nt_dists = self.distance(pose[0],self.curr_targ[0])
            ht_dists = self.distance(pose[1],self.curr_targ[0])
            nh_dist = self.distance(pose[0],pose[1])[2]
            dist_calc = (ht_dists[2]**2+nh_dist**2-nt_dists[2]**2)/(2*ht_dists[2]*nh_dist)
            if abs(dist_calc) > 1:
                dist_calc = 1
            pi_angle = math.acos(dist_calc)
            deg_angle = pi_angle *(180.0 / math.pi)      
            print('position', pose[1])            
            print('distance:', ht_dists[2])
            ##Trial in progress phase               
            if self.trial_ip:
                poses = pose
                self.pose_data.append(self.pose_parse(poses,self.trial_num))
                if (time.time()-self.last_tone) >= .099 and self.drop != 2:
                    self.play_sound(deg_angle, ht_dists[2])
                    self.last_tone = time.time()
                    # frequency = [self.TC[0:8,deg_angle],self.TC[8:16,deg_angle]]
                    # repeats = frequency / 10
                    # self.biphasic_stim(10,frequency,repeats)
                if (ht_dists[2] <= self.target_size) and (time.time()-self.trial_start[len(self.trial_start)-1] >= 0.5):
                    self.trial_end.append(time.time())
                    self.trial_ip = False
                    self.success = True
                    self.led_board_off()
                    self.reward_on()
                    self.reward_tone.play(loops = 0)
                if (time.time()-self.trial_start[len(self.trial_start)-1] >= 18) and self.success == False:
                    self.fail = True 
                    self.trial_end.append(time.time())
                    
                    
            ##Trial success phase
            if self.success:
                byte = self.zero.readline()
                print(byte)
                if byte == b'P':
                    self.trial_init = True
                    self.success = False
                    self.trial_num += 1
                    self.successes += 1
                    print("Trials Completed: ", self.trial_num)
                    print("Successes: ", self.successes)
                    if self.drop == 2:
                        self.trial_data.append([self.trial_end[len(self.trial_end)-1]-self.trial_start[len(self.trial_start) -1],self.curr_targ, 1, 'vis'])
                    if self.drop == 1:
                        self.trial_data.append([self.trial_end[len(self.trial_end)-1]-self.trial_start[len(self.trial_start) -1],self.curr_targ, 1, 'aud'])
                    else:
                        self.trial_data.append([self.trial_end[len(self.trial_end)-1]-self.trial_start[len(self.trial_start) -1],self.curr_targ, 1, 'False'])

                    self.sum_ttt += (self.trial_end[len(self.trial_end) - 1] - self.trial_start[len(self.trial_start) - 1])
                    print("Success Rate ", self.successes/self.trial_num)
                    print("Time taken: ", (self.trial_end[len(self.trial_end) - 1] - self.trial_start[len(self.trial_start)-1]))
                    print("Average success Time: ", self.sum_ttt / self.successes)
                    time.sleep(5)
                      
            if self.fail:
                
                if self.drop == 2:
                    self.trial_data.append([self.trial_end[len(self.trial_end)-1]-self.trial_start[len(self.trial_start) -1],self.curr_targ, 0, 'vis'])
                elif self.drop == 1:
                    self.trial_data.append([self.trial_end[len(self.trial_end)-1]-self.trial_start[len(self.trial_start) -1],self.curr_targ, 0, 'aud'])
                else:
                    self.trial_data.append([self.trial_end[len(self.trial_end)-1]-self.trial_start[len(self.trial_start) -1],self.curr_targ, 0, 'False'])

                self.reward_off()
                self.led_board_off()
                self.trial_init = True
                self.trial_num += 1
                print('Trials Completed:', self.trial_num)
                print("Successes: ", self.successes)
                print("Success Rate: ", self.successes / self.trial_num)
                if self.successes > 0:
                    print("Average Success Time: ", self.sum_ttt / self.successes)
                time.sleep(5)
                


        return pose

    def save(self, filename):

        ### save stim on and stim off times
        self.led_board_off()
        self.reward_off()
        filename += ".npy"
        trial_num = np.array(self.trial_num)
        successes = np.array(self.successes)
        targets = np.array(self.targets)
        trial_start = np.array(self.trial_start)
        trial_end = np.array(self.trial_end)
        dropout_trials = np.array(self.dropout_trials, dtype = 'object')
        pos = self.pos
        pose_data = np.array(self.pose_data)
        trial_data = np.array(self.trial_data, dtype = 'object')
        try:
            np.savez(
                filename, pos = pos, trial_num = trial_num, successes  = successes,
                targets = targets, trial_start = trial_start,
                trial_end = trial_end, dropout_trials = dropout_trials, 
                pose_data = pose_data, trial_data = trial_data)
            save_code = True
        except Exception:
            save_code = False

        return save_code
    
class Habituation(Processor):
    
    def __init__(self, lik_thresh=0.5, baudrate = int(9600)):
    
        super().__init__()

        #Serial Connections to Arduinos: Zero controls Visual cues and Leo controls reward dispensing.
        self.leo = serial.Serial('COM9', baudrate, timeout = 0)
        self.nosepoke = 0
        self.trial_num = 1 #Current trial number as well as total number of trials at the end
        self.trial_start = 0 #variable for start time of each trial
        self.trial_current = 0
        self.successes = 0 #number of successful trials
        self.reward_dispense = True
        self.start_times = []
        self.end_times = []
        self.pos = []
        bits = 16
        channels = 2
        pygame.mixer.pre_init(44100, -bits, channels)
        pygame.init()
        duration = 1
        frequency_l = 8000
        frequency_r = 8000
        sample_rate = 44100
        n_samples = int(round(duration*sample_rate))
        buf = np.zeros((n_samples,2), dtype = np.int16)
        max_sample = 2**(bits-1) - 1
        for s in range(n_samples):
            t = float(s)/sample_rate
            buf[s][0] = int(round(max_sample*math.sin(2*math.pi*frequency_l*t)))
            buf[s][1] = int(round(max_sample*math.sin(2*math.pi*frequency_r*t)))

        self.sound = pygame.sndarray.make_sound(buf)
        self.sound.set_volume(0.055)

        
    def close_serial(self):
        self.leo.close()
        
    def rewardport_on(self):
        self.leo.reset_input_buffer()
        self.leo.write(b"H")
        
    def rewardport_off(self):
        self.leo.reset_input_buffer()
        self.leo.write(b"L")

    

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
            print(pose)
            print(len(pose))
            self.pos.append(pose)
            if (self.reward_dispense == True) or ((time.time()-self.trial_start)%50 == 0):
                self.sound.play(loops = 0)
                self.start_times.append(time.time())
                self.trial_start = time.time()
                self.rewardport_on()
                self.reward_dispense = False
            byte = self.leo.readline()
            print(byte)
            print("Time Elapsed: ", (time.time() - self.trial_start))
            if byte == b'P':
                self.nosepoke += 1
                self.end_times.append(self.trial_start)

            if (time.time() - self.trial_start) >= 25:
                self.rewardport_off()
                self.reward_dispense = True
                self.trial_num += 1
                print("Nosepokes: ", self.nosepoke)
                print("Trials Completed: ",self.trial_num-1)
                print("Rate: ", (self.nosepoke / (self.trial_num-1)))
                time.sleep(5)
                
            

        return pose
    
    def save(self, filename):
    
            ### save stim on and stim off times
            self.rewardport_off()
            self.close_serial()
            filename += ".npy"
            pos = np.array(self.pos)
            trial_num = np.array(self.trial_num)
            start_times = np.array(self.start_times)
            end_times = np.array(self.end_times)

            nosepoke = np.array(self.nosepoke)
            try:
                np.savez(
                    filename, pos = pos, trial_num = trial_num, start_times = start_times, nosepoke = nosepoke, end_times = end_times
                )
                save_code = True
            except Exception:
                save_code = False
    
            return save_code
        
  

