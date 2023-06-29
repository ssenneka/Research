# -*- coding: utf-8 -*-
"""
Created on Tue Jun  6 14:34:41 2023

@author: mcmadmx
"""

import math
import pygame
import numpy as np

bits = 16                                                              #number of bits for the sound quality
channels = 2                                                           #number of channels for the pygame mixer
pygame.mixer.pre_init(44100, -bits, channels)                          #pre initialization of the pygame mixer
pygame.init()                                                          #initialization of the pygame mixer
duration = 5                                                          #length of reward tone
frequency = 8000                                                       #frequency of reward tone
sample_rate = 44100                                                    #number of samples per second
n_samples = int(round(duration*sample_rate))                           #number of samples to collect
buf = np.zeros((n_samples,2), dtype = np.int16)                        #array for sound wave to be created in
max_sample = 2**(bits-1) - 1                                           #maximum number of samples
for s in range(n_samples):
    t = float(s)/sample_rate
    buf[s][0] = int(round(0*max_sample*math.sin(2*math.pi*frequency*t)))
    buf[s][1] = int(round(0.55*max_sample*math.sin(2*math.pi*frequency*t)))

sound = pygame.sndarray.make_sound(buf)
channel_l = pygame.mixer.Channel(0)                               #left channel of the pygame mixer
channel_r = pygame.mixer.Channel(1)
sound.play()