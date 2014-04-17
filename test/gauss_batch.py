#!/usr/bin/env python

import roslib; 
roslib.load_manifest('changepoint')
import rospy 
import numpy as np
from pylab import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from changepoint.srv import *
from changepoint.msg import *
from mpl_toolkits.mplot3d import art3d
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import Circle
from itertools import product
import pickle

      
def makeDetectRequest(req):
    try:
        dc = rospy.ServiceProxy('changepoint/detect_changepoints', DetectChangepoints)  
        resp = dc(req)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

        
        
        
if __name__ == '__main__':
    rospy.init_node('changepoint_test')
    
    pf = open('gauss_pickle_final.txt','r')
    t = pickle.load(pf)
    fig = plt.figure()

    counts = [0.0]*len(t)
    n_trials = 100
    
    for i in xrange(n_trials):
        req = DetectChangepointsRequest()
        req.data = [DataPoint([x]) for x in t]
        req.model_type = 'changepoint/Gauss1DFitter'
        resp = makeDetectRequest(req)
    
        for seg in resp.segments[1:]:
            counts[seg.first_point] += 1
            
        print i
    
    for i in xrange(len(counts)):
        plt.vlines(i,0,counts[i]/n_trials)
        #plot(counts)
        xlim([0,len(counts)])
        ylim([0,1])
        
    show()
