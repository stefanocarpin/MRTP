#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 20 18:05:33 2018

@author: shamano
"""

import numpy as np
from math import pi
from rotations import *

R1 = rotx(pi/3)
R2 = roty(pi/4)
R = R1.dot(R2)
print('R=')
print(R)
print('Checking determinant')
print(np.linalg.det(R))

print('p=')
p = np.array([[0],[2],[1]])
print(p)

print('R*p')
print(R.dot(p))

