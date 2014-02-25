#!/usr/bin/env python

import roslib; 
roslib.load_manifest('changepoint')
import rospy 
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from changepoint.srv import *
from changepoint.msg import *


def makeDetectRequest(traj):
    try:
        dc = rospy.ServiceProxy('changepoint/detect_changepoints', DetectChangepoints)
        resp = dc(traj)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

        
if __name__ == '__main__':
    rospy.init_node('changepoint_test')
    
    #Create test data
    t = []  
    x = np.random.normal(-1.0, 0.1, 200)
    t += x.tolist()
    x = np.random.normal(2.0, 0.3, 30)
    t += x.tolist()
    x = np.random.normal(-2.0, 0.1, 40)
    t += x.tolist()
    x = np.random.normal(-0.5, 0.7, 20)
    t += x.tolist()
    x = np.random.normal(-3.0, 0.5, 20)
    t += x.tolist()
    plt.plot(t)
    plt.show()
    traj = [[x] for x in t]
    
    #Detect changepoints
    req = DetectChangepointsRequest()
    req.data = [DataPoint(x) for x in traj]
    resp = makeDetectRequest(req)
    
    print
    for seg in resp.segments:
        print seg
        print

    
    