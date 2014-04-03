#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_unstructured_lfd')

import sys
import rospy
import threading
import generalUtils
#import trajUtils
#import moveUtils
import drawUtils
import arWorldModel
import tf
import geometry_msgs






def drawWedge(tag_id, radius, center, axis, min_q, max_q):

    gen_utils = generalUtils.GeneralUtils()
    draw_utils = drawUtils.DrawUtils()
    wm = arWorldModel.ARWorldModel()
    tformer = tf.TransformerROS(True, rospy.Duration(10.0))
    
    while 1:
        tag_pose = wm.getObjectById(0)
        p0 = [0.0]*3
        p1 = [0.0]*3
        p2 = [0.0]*3
        
        #Calc center point
        for i in xrange(3):
            p0[i] = tag_pose[i] + center[i]
            
        #Calc points on circumference  
        #First, make a frame for the base marker
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = 'torso_lift_link'
        m.child_frame_id = 'base_marker_pose'
        m.transform = gen_utils.vecToRosTransform(tag_pose)
        tformer.setTransform(m)
        
        #Then, add a transform from base marker to axis of rotation
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = 'base_marker_pose'
        m.child_frame_id = 'circ_axis_pose'
        m.transform = gen_utils.vecToRosTransform(center+axis)
        tformer.setTransform(m)
        
        #Now, calc coords in axis frame -- rotation axis is x-forward
        coord1 =  geometry_msgs.msg.PoseStamped()
        circ_axis.header.frame_id =  'circ_axis_pose'
        c1 = [0, radius * -cos(min_q), radius * sin(min_q), 0, 0, 0, 1]
        coord1.pose = gen_utils.vecToRosPose(c1)
        
        coord2 =  geometry_msgs.msg.PoseStamped()
        circ_axis.header.frame_id =  'circ_axis_pose'
        c2 = [0, radius * -cos(min_q), radius * sin(max_q), 0, 0, 0, 1]
        coord2.pose = gen_utils.vecToRosPose(c2)
        
        #Finally, do the transforms to get in robot frame
        try:
            tp1 = tformer.transformPose('torso_lift_link',coord1)
            tp2 = tformer.transformPose('torso_lift_link',coord2)
            p1 = gen_utils.rosPoseToVec(tp1)
            p2 = gen_utils.rosPoseToVec(tp2)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            print "\n drawWedge(): TF transform problem!"
        
        draw_utils.drawTriangle(p0,p1,p2)
        
        rospy.sleep(0.1)

  
if __name__ == '__main__':
    
    rospy.init_node('rvizArtic')
    
    r = 1.0
    c = [0.0, 0.0, 0.0]
    a = [0.0, 0.0, 0.0, 1.0]
    min_q = 0.67
    max_q = 0.79
    drawWedge(0, r, c, a, min_q, max_q)