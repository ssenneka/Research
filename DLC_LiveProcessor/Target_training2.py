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
    """Audio-visual target detection training class for use with DLC-Live GUI.
    
    This class is for training mice move to a target location within their cage
    indicated by auditory and visual cues. This class works in conjunction with
    DeepLabCut-Live to track the mice in real-time and determine when they have
    reached the target and a reward has been earned.
    
    """
    def __init__(self, baudrate = int(9600)):
        """Initialization Function.
        
        Initializes connection with DLC model, arduinos for controlling 
        stimulus and reward dispensing, and Ripple-Neuro Grapevine 
        Processor. Also initializes all global variables needed for control
        of training sessions and saving of all relevant data.

        Parameters
        ----------
        baudrate : TYPE, optional
            DESCRIPTION. The default is int(9600).

        Returns
        -------
        None.

        """
        super().__init__()
        ##
        self.dropout_session = False
        self.curr_targ = []
        self.drop_audio = False
        self.drop_visual = False
        self.dropout = [1,2,0,0,0,0,0,0,0,0]                                  
        self.dropout_trials = []
        self.drop = False
        self.fail = False                                                      
        self.last_tone = time.time()                                           
        self.pos = []                                                          
        self.pose_data = []
        self.session_length = 60
        self.session_start = time.time()
        self.sum_ttt = 0
        self.success = False                                                   
        self.successes = 0                                                         
        self.targets = []                                                      
        self.trial_end = []                                                    
        self.trial_init = True                                                 
        self.trial_ip = False                                                  
        self.trial_num = 0                                                     
        self.trial_start = []                                                  
        self.trial_data = []
        self.waiting = True                                                    
        self.zero = serial.Serial('COM11', baudrate, timeout = 0)
        self.leo = serial.Serial('COM9', baudrate, timeout = 0)              
        # self.target_list = [[[268,189], b'A'], 
        #                     [[467,186], b'B'],
        #                     [[270,389], b'C'],
        #                     [[470,387], b'D']]
                           
        self.target_list = [[[244,165], b'A'],                                 
                            [[246,281], b'B'],
                            [[249,399], b'C'],
                            [[362,162], b'D'],
                            [[364,281], b'E'],
                            [[365,398], b'F'],
                            [[478,161], b'G'],
                            [[481,278], b'H'],
                            [[483,396], b'I']]
       
        self.target_size = 55

                                         
        ##
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
                self.TC[i,j] = (1/(2 * math.pi)) * math.exp(kappa 
                               * math.cos((alldeg[j] - PD[i]) * math.pi / 180))
            self.TC[i,:] = baselinef * self.TC[i,:] / max(self.TC[i,:])
        ##
        bits = 16                                                             
        channels = 2                                                           
        pygame.mixer.pre_init(44100, -bits, channels)                         
        pygame.init()                                                          
        duration = 1                                                          
        frequency = 8000                                                      
        sample_rate = 44100                                                   
        n_samples = int(round(duration*sample_rate))                          
        buf = np.zeros((n_samples,2), dtype = np.int16)                       
        max_sample = 2**(bits-1) - 1                                          
        for s in range(n_samples):
            t = float(s)/sample_rate
            buf[s][0] = int(round(.2 * max_sample
                                  * math.sin(2*math.pi*frequency*t)))
            buf[s][1] = int(round(.2 * max_sample
                                  * math.sin(2*math.pi*frequency*t)))
        self.channel_l = pygame.mixer.Channel(0)                              
        self.channel_r = pygame.mixer.Channel(1)                               
        self.reward_tone = pygame.sndarray.make_sound(buf)                     
        self.reward_tone.set_volume(.055)                                      
        # with xp.xipppy_open(use_tcp=True):
        #     self.elecchans = xp.list_elec('stim')
       
        print("Setup Complete.")
       
    # def biphasic_stim(elec, amp, frequency, repeats):
    #     seq0 = []
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
    #             #create overall sequence header defining electrode, period, repeats
    #             seq0.append(xp.StimSeq(elec[i], 
    #                         period,
    #                         repeats[i],
    #                         cathseg, 
    #                         ipi, 
    #                         anseg))
    #         #This will stimulate on electrode 1 at 30 hz for one second 
    #         #(1000 ms)
    #         xp.StimSeq.send_stim_seqs(seq0) #send sequence to NIP
    #         time.sleep(0.0001)
   
    def close_serial(self):
        '''Closes serial connections to Leo and Zero.
        

        Returns
        -------
        None.

        '''                                                    
        self.leo.close()
        self.zero.close()
       
       
    def reward_on(self):     
        ''' Turns on the LED in the reward port.
           
        Turns on LED and allows Arduino to dispense reward upon nosepoke.

        Returns
        -------
        None.

        '''                                                  
        self.leo.reset_input_buffer()
        self.leo.write(b"H")
 
       
    def reward_off(self):   
        ''' Turns off reward port LED and reward dispensing.
        
        Returns
        -------
        None.
        
        '''                                                   
        self.leo.reset_input_buffer()
        self.leo.write(b"L")
   
   
    def target_led_on(self, target):
        ''' Turns on LED board to display a visual target cue for mouse.
        
        
        Parameters
        ----------
        target : Single byte character.
        

        Returns
        -------
        None.

        '''                                           
        self.zero.reset_input_buffer()
        self.zero.write(target)
   
   
    def led_board_off(self):
        '''Turns off all LEDS in LED Matrix.
        

        Returns
        -------
        None.

        '''
        self.zero.reset_input_buffer()
        self.zero.write(b'O')
   

    def distance(self,p1,p2):        
        ''' Calculates distance between two 2-Dimensional points.
        

        Parameters
        ----------
        p1 : List or Tuple of point 1
        p2 : List or Tuple of point 2

        Returns
        -------
        distances : List containing the x-distance, y-distance, and shortest 
        connecting distance between the two points provided.

        '''                                          
        x_dist = p1[0]-p2[0]
        y_dist = p1[1]-p2[1]
        dist = math.sqrt((x_dist)**2+(y_dist)**2)
        distances = [x_dist, y_dist, dist]
        return distances

    def pose_parse(self, pose, trial_num):
        '''Concatenates all pose indices into single row of data.
        

        Parameters
        ----------
        pose : List of lists containing x,y, and likelihood calculations for 
        each body part being tracked by the DLC model.
        trial_num : variable indicating the current trial the mouse is on for 
        that session.
        
        
        Returns
        -------
        data : numpy array of dimensions 1x22.
        A single row containing the estimated pixel coordinates and likelihood
        predicted by the DLC tracking model.

        '''
        data = np.concatenate(([trial_num], pose[0], pose[1], pose[2], pose[3],
                               pose[4], pose[5], pose[6]))
        return data
   
    def play_sound(self, angle, dist):
        '''Utilizes pygame mixer to play pure-tone sound samples
        
        This function takes an angle and a distance to produce a pure-tone 
        continuous beep that changes in frequency with angle and distance
        between the mouse and the target.
        
        Parameters
        ----------
        angle : float
            Angle between mouse's forward heading vector and target direction
            vector.
        dist : float
            Distance between the mouse and the target in pixels.

        Returns
        -------
        None.

        '''                                         
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
            buf_l[s][0] = int(round(max_sample
                                    * math.sin(2*math.pi*frequency*t)))
            buf_l[s][1] = int(round(0 * max_sample
                                    * math.sin(2*math.pi*frequency*t)))
            buf_r[s][0] = int(round(0 * max_sample
                                    * math.sin(2*math.pi*frequency*t)))
            buf_r[s][1] = int(round(max_sample
                                    * math.sin(2*math.pi*frequency*t)))    
        left_chan = pygame.sndarray.make_sound(buf_l)
        right_chan = pygame.sndarray.make_sound(buf_r)
        self.channel_l.set_volume(volume_r)
        self.channel_r.set_volume(volume_l)
        self.channel_l.play(left_chan)
        self.channel_r.play(right_chan)
        self.last_tone = time.time()
   

    def process(self, pose, **kwargs): 
        '''Main process function.
        
        The main process function controls all aspects of target detection 
        training. Using the pose estimates from the DLC model, it determines
        when a trial is in progress, has been successfully or unsuccessfully
        concluded, and when a new trial needs to be initialiazed.
        
        Parameters
        ----------
        pose : List of Lists
            List of coordinates and likelihood thresholds for each body part
            being tracked.
        **kwargs : TYPE
            Keword arguments needed to properly receive and process information
            from the DLC model.

        Returns
        -------
        pose : List of Lists
            Returns an unaltered version of the pose variable to be used for 
            marker display on video feed.

        '''                                        
        if self.waiting == True:                                              
            print("Waiting For user to press record")                          
            self.waiting = False                                                   
        if kwargs['record']:                                                       
            ##Trial Initiation Phase                                           
            if self.trial_init:
                if self.dropout_session:                                        
                    self.drop = random.choice(self.dropout)            
                self.curr_targ = random.choice(self.target_list)                   
                if self.drop != 1:
                    self.target_led_on(self.curr_targ[1])
                self.targets.append(self.curr_targ[0])                             
                print("Target: ",self.curr_targ)                                  
                self.trial_init = False
                self.trial_ip = True
                self.success = False
                self.fail = False
                self.trial_start.append(time.time())
            ##This section of code is outside of any if statements because
            ##these values are needed for the following trial phases
            self.pos.append(pose)
            nt_dists = self.distance(pose[0], self.curr_targ[0])
            ht_dists = self.distance(pose[1], self.curr_targ[0])
            nh_dist = self.distance(pose[0], pose[1])[2]
            dist_calc = (ht_dists[2]**2+nh_dist**2-nt_dists[2]**2)/(2*ht_dists[2]*nh_dist)
            if abs(dist_calc) > 1:
                dist_calc = 1
            pi_angle = math.acos(dist_calc)
            deg_angle = pi_angle * (180.0/math.pi)      
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
                byte = self.leo.readline()
                print(byte)
                if byte == b'P':
                    self.trial_init = True
                    self.success = False
                    self.trial_num += 1
                    self.successes += 1
                    print("Trials Completed: ", self.trial_num)
                    print("Successes: ", self.successes)
                    if self.drop == 2:
                        self.trial_data.append(
                            [self.trial_end[len(self.trial_end)-1]
                             - self.trial_start[len(self.trial_start) - 1],
                             self.curr_targ[1], 1, 'vis'])
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
        '''Data saving function.
        
        Takes in file name generated by DLC-Live GUI and saves relevant data to
        a numpy zip file.
        Parameters
        ----------
        filename : string
            Name for file data is saved in generated by DLC-Live GUI.

        Returns
        -------
        save_code : bool
            Returns true or false depending on whether or not the data
            saved properly.

        '''
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