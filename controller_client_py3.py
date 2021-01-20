#!/usr/bin/env python

from __future__ import print_function
import os
import gym
from baselines import deepq
import sys
import rospy
# from beginner_tutorials.srv import *
from py3_server.srv import *

def ctrl_client(x,y,z,w):
    rospy.wait_for_service('tactile_motion_ctrl')
    try:
        # create a handle (function) for calling the service
        ctrl_cmd = rospy.ServiceProxy('tactile_motion_ctrl', CtrlSrv)
        # the arg in this handle is a request, and the handle returns the response which is provided by the server
        resp1 = ctrl_cmd(x,y,z,w)
        return resp1.srvrsp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z w]"%sys.argv[0]

if __name__ == "__main__":

        
    # if len(sys.argv) == 5:
    #     x = float(sys.argv[1])
    #     y = float(sys.argv[2])
    #     z = float(sys.argv[3])
    #     w = float(sys.argv[4])
    # else:
    #     print(usage())
    #     sys.exit(1)
    # print("client sent [%s], and get from service = [%s]" %(x, ctrl_client(x,y,z,w)))

    if len(sys.argv) == 2:
        step = int(sys.argv[1])
    i = 0 
    x = 0.4
    y = -0.4
    z = 0
    w = 1

    x_step = (0.1 - 0.4)/step
    y_step = (0.1 + 0.4)/step
    z_step = (1 - 0 )/step
    while True:
        # if i == 0:
        #     print("client sent goal position, and get from service = [%s]" %(ctrl_client(0.4, -0.4, 0, 1)))
        #     i = 1
        # elif i == 1 :
        #     print("client sent goal position, and get from service = [%s]" %(ctrl_client(0.1, 0.1, 1, 1)))
        #     i = 0


        print("client's sending the next goal position...")
        ctrl_client(x, y, z, w)
        x = x + x_step
        y = y + y_step 
        z = z + z_step 
        i = i +1
        print("step No.%s" %i + " is compelted.")
        if i == step:
            break
    print ('Interpolation ')
        