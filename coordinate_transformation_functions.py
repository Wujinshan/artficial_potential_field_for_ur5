import numpy as np
from math import sin, cos, pi

def rotx(angle):
          T = np.matrix([[1,0,0,0],[0,cos(angle),-1*sin(angle),0],[0,sin(angle),cos(angle),0],[0,0,0,1]])
          return T

def roty(angle):
          T = np.matrix([[cos(angle),0,sin(angle),0],[0,1,0,0],[-1*sin(angle),0,cos(angle),0],[0,0,0,1]])
          return T

def rotz(angle):
          T = np.matrix([[cos(angle),-1*sin(angle),0,0],[sin(angle),cos(angle),0,0],[0,0,1,0],[0,0,0,1]])
          return T

def trans(x,y,z):
          T=np.matrix([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
          return T