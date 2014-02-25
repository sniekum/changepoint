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
    
    #Create test data
    t = []  
    x = np.random.normal(-1.0, 0.1, 200)
    t += x.tolist()
    x = np.random.normal(2.0, 0.3, 40)
    t += x.tolist()
    x = np.random.normal(-2.0, 0.1, 20)
    t += x.tolist()
    x = np.random.normal(-0.5, 0.5, 20)
    t += x.tolist()
    x = np.random.normal(-3.0, 0.7, 20)
    t += x.tolist()
    #plot(t)
    #show()
    #traj = [[x] for x in t]
    
    #Generate rotational data
    traj = []
    
    mean = 0.0
    sigma = 0.05
    radius = 10.0
    for i in range(14,-1,-1):
        pose = [0.0]*7
        angle = (i * 5.0) * 0.0174532925
        pose[0] = radius * cos(angle) + np.random.normal(mean, sigma, 1)[0]
        pose[1] = radius * sin(angle) + np.random.normal(mean, sigma, 1)[0]
        pose[2] = (np.random.normal(mean, sigma, 1))[0]
        pose[3] = 0
        pose[4] = 0
        pose[5] = sin(angle/2.0)
        pose[6] = cos(angle/2.0)
        #traj.append(pose)
    
    for i in xrange(15):
        pose = [0.0]*7
        #pose[0] = (10.0 - (i+1)/2.0) + np.random.normal(mean, sigma, 1)[0]
        pose[0] = (5.0 - (i+1)/5.0) + np.random.normal(mean, sigma, 1)[0]
        pose[1] = np.random.normal(mean, sigma, 1)[0]
        pose[2] = (np.random.normal(mean, sigma, 1))[0]
        pose[3] = 0
        pose[4] = 0
        pose[5] = 0
        pose[6] = 1.0
        traj.append(pose)    
    
    radius = 2.0
    for i in xrange(15):
        pose = [0.0]*7
        angle = (i * 5.0) * 0.0174532925
        pose[0] = radius * cos(angle) + np.random.normal(mean, sigma, 1)[0]
        pose[1] = radius * sin(angle) + np.random.normal(mean, sigma, 1)[0]
        pose[2] = (np.random.normal(mean, sigma, 1))[0]
        pose[3] = 0
        pose[4] = 0
        pose[5] = sin(angle/2.0)
        pose[6] = cos(angle/2.0)
        #traj.append(pose)
        
    sigma = 0.05
    for i in xrange(50):
        pose = [0.0]*7
        pose[0] = 2.0 + np.random.normal(mean, sigma, 1)[0]
        pose[1] = 0.0 + np.random.normal(mean, sigma, 1)[0]
        pose[2] = (np.random.normal(mean, sigma, 1))[0]
        pose[3] = 0
        pose[4] = 0
        pose[5] = 0
        pose[6] = 1.0
        traj.append(pose)    
        
    for t in traj:
        print t
        
    X = np.array([traj[i][0] for i in xrange(len(traj))])
    Y = np.array([traj[i][1] for i in xrange(len(traj))])
    Z = np.array([traj[i][2] for i in xrange(len(traj))])
    
    colors = []
    colors += ['r']*14
    colors += ['g']*26
    #colors += ['b']*15
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('equal')
    ax.scatter(X,Y,Z, c=colors)
    
    # Create cubic bounding box to simulate equal aspect ratio
    max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(X.max()+X.min())
    Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(Y.max()+Y.min())
    Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(Z.max()+Z.min())
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')

    #p = Circle((0,0), 9.424720, facecolor = 'None', edgecolor = 'r', alpha = .6)
    #ax.add_patch(p)
    #pathpatch_2d_to_3d(p, z = 0, normal = (0, 0, 1))
    #pathpatch_translate(p, (0.500496, 0.326766, 0.093434))
    
    #llen = 100
    #px = 9.505870 
    #py = -0.041050
    #pz = 0.006211 
    #sx = -0.999897   
    #sy = 0.013742   
    #sz = 0.004071
    #lx = [px-(sx*llen),px+(sx*llen)]
    #ly = [py-(sy*llen),py+(sy*llen)]
    #lz = [pz-(sz*llen),pz+(sz*llen)]
    #ax.plot(lx,ly,lz, color='g', alpha=0.6)
    
    #p = Circle((0,0), 1.816539, facecolor = 'None', edgecolor = 'b', alpha = .6)
    #ax.add_patch(p)
    #pathpatch_2d_to_3d(p, z = 0, normal = (0, 0, 1))
    #pathpatch_translate(p, (0.185348, 0.124114, -0.244365))
           
    #llen = 100
    #px = 4.760337
    #py = -0.031724
    #pz = 0.004703 
    #sx = -0.999792  
    #sy = 0.018180  
    #sz = -0.009205
    #lx = [px-(sx*llen),px+(sx*llen)]
    #ly = [py-(sy*llen),py+(sy*llen)]
    #lz = [pz-(sz*llen),pz+(sz*llen)]
    #ax.plot(lx,ly,lz, color='r', alpha=0.6)

    
    ax.auto_scale_xyz([0, 5], [-2.5, 2.5], [-2.5, 2.5])
    #plt.show()   
    req = DetectChangepointsRequest()
    req.data = [DataPoint(x) for x in traj]
    resp = makeDetectRequest(req)
    
    print
    for seg in resp.segments:
        print seg
        print

