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

def rotation_matrix(d):
    """
    Calculates a rotation matrix given a vector d. The direction of d
    corresponds to the rotation axis. The length of d corresponds to 
    the sin of the angle of rotation.

    Variant of: http://mail.scipy.org/pipermail/numpy-discussion/2009-March/040806.html
    """
    sin_angle = np.linalg.norm(d)

    if sin_angle == 0:
        return np.identity(3)

    d /= sin_angle

    eye = np.eye(3)
    ddt = np.outer(d, d)
    skew = np.array([[    0,  d[2],  -d[1]],
                  [-d[2],     0,  d[0]],
                  [d[1], -d[0],    0]], dtype=np.float64)

    M = ddt + np.sqrt(1 - sin_angle**2) * (eye - ddt) + sin_angle * skew
    return M

def pathpatch_2d_to_3d(pathpatch, z = 0, normal = 'z'):
    """
    Transforms a 2D Patch to a 3D patch using the given normal vector.

    The patch is projected into they XY plane, rotated about the origin
    and finally translated by z.
    """
    if type(normal) is str: #Translate strings to normal vectors
        index = "xyz".index(normal)
        normal = np.roll((1,0,0), index)

    normal /= norm(normal) #Make sure the vector is normalised

    path = pathpatch.get_path() #Get the path and the associated transform
    trans = pathpatch.get_patch_transform()

    path = trans.transform_path(path) #Apply the transform

    pathpatch.__class__ = art3d.PathPatch3D #Change the class
    pathpatch._code3d = path.codes #Copy the codes
    pathpatch._facecolor3d = pathpatch.get_facecolor #Get the face color    

    verts = path.vertices #Get the vertices in 2D

    d = np.cross(normal, (0, 0, 1)) #Obtain the rotation vector    
    M = rotation_matrix(d) #Get the rotation matrix

    pathpatch._segment3d = np.array([np.dot(M, (x, y, 0)) + (0, 0, z) for x, y in verts])

def pathpatch_translate(pathpatch, delta):
    """
    Translates the 3D pathpatch by the amount delta.
    """
    pathpatch._segment3d += delta

    
def makeDetectRequest(traj):
    try:
        dc = rospy.ServiceProxy('changepoint/detect_changepoints', DetectChangepoints)
        resp = dc(traj)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

        
if __name__ == '__main__':
    rospy.init_node('changepoint_test')
    
    f = open('bagfiles/2-21-14/diffpickle2', 'r')
    traj = pickle.load(f)
    
    #TODO: GET RID OF DATA WHEN NOT MOVING! (Have to do before taking diff...)
    
    X = np.array([traj[i][0] for i in xrange(len(traj))])
    Y = np.array([traj[i][1] for i in xrange(len(traj))])
    Z = np.array([traj[i][2] for i in xrange(len(traj))])
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('equal')
    
    req = DetectChangepointsRequest()
    req.data = [DataPoint(x) for x in traj]
    resp = makeDetectRequest(req)
    
    print
    for seg in resp.segments:
        print seg
        print
    
    GRAPH_ARTIC = False
    colors = []
    choices = ['red','orange','yellow','green','blue','purple','black']
    i = 0
    for seg in resp.segments:
        colors += [choices[i]] * (seg.last_point - seg.first_point + 1) 
        
        if GRAPH_ARTIC:
            params = seg.model_params
            if(seg.model_name == "rotational"):
                p = Circle((0,0), params[0], facecolor = 'None', edgecolor = choices[i], alpha = .6)
                ax.add_patch(p)
                pathpatch_2d_to_3d(p, z = 0, normal = (0, 0, 1))
                pathpatch_translate(p, (params[1], params[2], params[3]))
            if(seg.model_name == "prismatic"):
                llen = 100
                sx = params[0]
                sy = params[1]
                sz = params[2]
                px = params[3]  
                py = params[4]
                pz = params[5]  
                lx = [px-(sx*llen),px+(sx*llen)]
                ly = [py-(sy*llen),py+(sy*llen)]
                lz = [pz-(sz*llen),pz+(sz*llen)]
                ax.plot(lx,ly,lz, color=choices[i], alpha=0.6)
            
        i = (i+1) % len(choices)
    
    #Create equal aspect ratio that is just large enough for all points
    ax.scatter(X,Y,Z, c=colors)
    max_range = max(X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min())
    X_buffer = (max_range - (X.max()-X.min())) / 2.0
    Y_buffer = (max_range - (Y.max()-Y.min())) / 2.0
    Z_buffer = (max_range - (Z.max()-Z.min())) / 2.0
    
    ax.auto_scale_xyz([X.min()-X_buffer, X.max()+X_buffer], [Y.min()-Y_buffer, Y.max()+Y_buffer], [Z.min()-Z_buffer, Z.max()+Z_buffer])
    plt.show()       

