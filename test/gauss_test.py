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
    
    #Create test data
    t = []  
    x = np.random.normal(0.0, 2.0, 40)
    t += x.tolist()
    x = np.random.normal(0.0, 1.0, 60)
    t += x.tolist()
    x = np.random.normal(0.0, 3.0, 30)
    t += x.tolist()
    x = np.random.normal(0.0, 1.5, 50)
    t += x.tolist()
    x = np.random.normal(0.0, 2.5, 70)
    t += x.tolist()
    #plot(t)
    #show()
    #pf = open('gauss_pickle.txt','w')
    #pickle.dump(t,pf)
    #pf.close()
    #asdasd
    
    pf = open('gauss_pickle_final.txt','r')
    t = pickle.load(pf)
    plot(t)

    req = DetectChangepointsRequest()
    req.data = [DataPoint([x]) for x in t]
    req.model_type = 'changepoint/Gauss1DFitter'
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
