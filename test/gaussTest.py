#!/usr/bin/env python

import roslib; 
roslib.load_manifest('changepoint')
import rospy 
import numpy as np
from pylab import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from changepoint.srv import *
from changepoint.msg import *
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
        
    pf = open('data/gauss_data.txt','r')
    t = pickle.load(pf)
    plot(t)

    req = DetectChangepointsRequest()
    req.data = [DataPoint([x]) for x in t]
    req.model_type = 'changepoint/Gauss1DFitter'
    
    req.cp_params.len_mean = 50.0
    req.cp_params.len_sigma = 10.0
    req.cp_params.min_seg_len = 2
    req.cp_params.max_particles = 100
    req.cp_params.resamp_particles = 100
    
    resp = makeDetectRequest(req)
    
    print
    for seg in resp.segments:
        print "Model:", seg.model_name, "   Length:", seg.last_point - seg.first_point + 1
        print "Start:", seg.first_point
        print "End:", seg.last_point
        for i in xrange(len(seg.model_params)):
            print "  ", seg.param_names[i], ":", seg.model_params[i]
        print
              
    for seg in resp.segments[1:]:
        vlines(seg.first_point,-8,8,linestyles='dashed',linewidth=2.0)
    show()
