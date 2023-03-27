# -*- coding: utf-8 -*-
"""
Created on Thu Mar 23 12:52:23 2023

@author: sjsen
"""

import numpy as np
import math
from matplotlib import pyplot as plt
import sys
import time

print(sys.path)
body_parts = ['nose', 'head_center', 'right_ear', 'left_ear', 'right_hip', 'left_hip', 'tail_base']

file_name = input("Please enter file name: \n")


data = np.load(file_name)

lst = data.files
data_dict = {i: data[i] for i in lst}
locals().update(data_dict)

pos_dict = {body_parts[i]: pos[:,i,:] for i in range(0,len(body_parts))}
locals().update(pos_dict)


plt.rcParams["figure.figsize"] = [7.40,5.20]
plt.rcParams["figure.autolayout"] = True

plt.xlim(0, 720)
plt.ylim(0,540)
plt.grid()

def 