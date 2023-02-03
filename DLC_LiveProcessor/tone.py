# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 12:57:04 2023

@author: sjsen
"""

import pygame
import math
import numpy

bits = 16
channels = 2

pygame.mixer.pre_init(44100, -bits, channels)
pygame.init()

duration = .1

frequency_l = 400
frequency_r = 800

sample_rate = 44100

n_samples = int(round(duration*sample_rate))

buf = numpy.zeros((n_samples,2), dtype = numpy.int16)
max_sample = 2**(bits-1) - 1


for s in range(n_samples):
    t = float(s)/sample_rate
    
    buf[s][0] = int(round(max_sample*math.sin(2*math.pi*frequency_l*t)))
    buf[s][1] = int(round(max_sample*math.sin(2*math.pi*frequency_r*t)))
    
    
sound = pygame.sndarray.make_sound(buf)

sound.play(loops = 0)

def tone(frequency, left_vol, right_vol):
    

