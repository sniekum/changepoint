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
import numpy as np



def drawWedge(tag_id, radius, center, axis, min_q, max_q):

    gen_utils = generalUtils.GeneralUtils()
    draw_utils = drawUtils.DrawUtils()
    wm = arWorldModel.ARWorldModel()
    tformer = tf.TransformerROS(True, rospy.Duration(10.0))
    
    rospy.sleep(1.0) #Let the world model warm up

    while not rospy.is_shutdown():
        tag_pose = (wm.getPartialWorldState([tag_id]))[tag_id]
        
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
        
        #Calc the center point
        coord0 =  geometry_msgs.msg.PoseStamped()
        coord0.header.frame_id =  'circ_axis_pose'
        c0 = [0, 0, 0, 0, 0, 0, 1]
        coord0.pose = gen_utils.vecToRosPose(c0)
        tp0 = tformer.transformPose('torso_lift_link',coord0)
        p0 = gen_utils.rosPoseToVec(tp0.pose)

        #Now, calc other coords in axis frame -- rotation axis is x-forward
        coord1 =  geometry_msgs.msg.PoseStamped()
        coord1.header.frame_id =  'circ_axis_pose'
        c1 = [radius * np.cos(min_q), radius * -np.sin(min_q), 0, 0, 0, 0, 1]
        coord1.pose = gen_utils.vecToRosPose(c1)
        
        coord2 =  geometry_msgs.msg.PoseStamped()
        coord2.header.frame_id =  'circ_axis_pose'
        c2 = [radius * np.cos(min_q), radius * -np.sin(max_q), 0, 0, 0, 0, 1]
        coord2.pose = gen_utils.vecToRosPose(c2)
        
        #Finally, do the transforms to get in robot frame
        try:
            tp1 = tformer.transformPose('torso_lift_link',coord1)
            tp2 = tformer.transformPose('torso_lift_link',coord2)
            p1 = gen_utils.rosPoseToVec(tp1.pose)
            p2 = gen_utils.rosPoseToVec(tp2.pose)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            print "\n drawWedge(): TF transform problem!"
        
        draw_utils.drawTriangle(p0,p1,p2)
        
        rospy.sleep(0.1)
        
        
       
def drawGraspAxis(tag_id, radius, orient):

    gen_utils = generalUtils.GeneralUtils()
    draw_utils = drawUtils.DrawUtils()
    wm = arWorldModel.ARWorldModel()
    tformer = tf.TransformerROS(True, rospy.Duration(10.0))
    
    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        wmstate = wm.getPartialWorldState([tag_id])
        tag_pose = wmstate[tag_id]
        
        #First, make a frame for the base marker
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = 'torso_lift_link'
        m.child_frame_id = 'base_marker_pose'
        m.transform = gen_utils.vecToRosTransform(tag_pose)
        tformer.setTransform(m)
           
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = 'base_marker_pose'
        m.child_frame_id = 'circ_temp'
        inv = [-orient[0], -orient[1], -orient[2], orient[3]]
        m.transform = gen_utils.vecToRosTransform([0,0,0]+[0,0,0,1])
        #m.transform = gen_utils.vecToRosTransform([0,0,0]+inv)
        tformer.setTransform(m)
        
        #Now, calc coords in axis frame -- rotation axis is x-forward
        coord1 =  geometry_msgs.msg.PoseStamped()
        coord1.header.frame_id =  'circ_temp'
        rotq = [0, .7071, 0, -.7071] 
        c1 = [radius, 0, -.1] + rotq
        coord1.pose = gen_utils.vecToRosPose(c1)
        
        #Finally, do the transforms to get in robot frame
        try:
            tp1 = tformer.transformPose('torso_lift_link',coord1)
            grasp_axis = gen_utils.rosPoseToVec(tp1.pose)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            print "\n drawWedge(): TF transform problem!"
        
        draw_utils.drawArrow(grasp_axis, [0.2, 0.015, 0.015], [0,1,0,1])
        
        rospy.sleep(0.1)  
        
        
        
#traj is the (non-difference) traj of the marker of interest
# tag_id is the tag to draw on
def drawRigidDirections(tag_id, traj):
    
    gen_utils = generalUtils.GeneralUtils()
    draw_utils = drawUtils.DrawUtils()
    wm = arWorldModel.ARWorldModel()
    tformer = tf.TransformerROS(True, rospy.Duration(10.0))            

    #Just check and see if the observed movement in each DOF is greater than some epsilon
    eps = 0.05 #meters  #FOR NOW, DON'T WORRY ABOUT ROTATIONAL DOFs
    
    traj_x = [traj[i][0] for i in xrange(len(traj))]
    traj_y = [traj[i][1] for i in xrange(len(traj))]
    traj_z = [traj[i][2] for i in xrange(len(traj))]

    draw_x = draw_y = draw_z = False
    if max(traj_x) - min(traj_x) > eps:
        draw_x = True
    if max(traj_y) - min(traj_y) > eps:
        draw_y = True
    if max(traj_z) - min(traj_z) > eps:
        draw_z = True
    
    print "DRAW"
    print draw_x, draw_y, draw_z    
    print max(traj_x) - min(traj_x)
    print max(traj_y) - min(traj_y)
    print max(traj_z) - min(traj_z)

    rospy.sleep(1.0) #Let the world model warm up

    while not rospy.is_shutdown():
        tag_pose = (wm.getPartialWorldState([tag_id]))[tag_id]
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = 'torso_lift_link'
        m.child_frame_id = 'base_marker_pose'
        m.transform = gen_utils.vecToRosTransform(tag_pose)
        tformer.setTransform(m)
        
        if draw_x:
            rotate180z = [0, 0, 1, 0] 
            coord2 =  geometry_msgs.msg.PoseStamped()
            coord2.header.frame_id =  'base_marker_pose' 
            c2 = [0, 0, 0] + rotate180z
            coord2.pose = gen_utils.vecToRosPose(c2)

            try:
                tp2 = tformer.transformPose('torso_lift_link',coord2)
                pose2 = gen_utils.rosPoseToVec(tp2.pose)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                print "\n drawRigidDirections(): TF transform problem!"
            
            draw_utils.drawArrow(tag_pose, [0.2, 0.015, 0.015], [.5,0,.5,1], 1)
            draw_utils.drawArrow(pose2, [0.2, 0.015, 0.015], [.5,0,.5,1], 2)
            
        if draw_y:
            rotate90zpos = [0, 0, .7071, .7071] 
            rotate90zneg = [0, 0, .7071, -.7071] 
            
            coord1 =  geometry_msgs.msg.PoseStamped()
            coord1.header.frame_id =  'base_marker_pose' 
            c1 = [0, 0, 0] + rotate90zpos
            coord1.pose = gen_utils.vecToRosPose(c1)

            try:
                tp1 = tformer.transformPose('torso_lift_link',coord1)
                pose1 = gen_utils.rosPoseToVec(tp1.pose)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                print "\n drawRigidDirections(): TF transform problem!"

            coord2 =  geometry_msgs.msg.PoseStamped()
            coord2.header.frame_id =  'base_marker_pose' 
            c2 = [0, 0, 0] + rotate90zneg
            coord2.pose = gen_utils.vecToRosPose(c2)

            try:
                tp2 = tformer.transformPose('torso_lift_link',coord2)
                pose2 = gen_utils.rosPoseToVec(tp2.pose)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                print "\n drawRigidDirections(): TF transform problem!"

            draw_utils.drawArrow(pose1, [0.2, 0.015, 0.015], [.5,0,.5,1], 3)
            draw_utils.drawArrow(pose2, [0.2, 0.015, 0.015], [.5,0,.5,1], 4)
            
        if draw_z:
            rotate90ypos = [0, .7071, 0, .7071] 
            rotate90yneg = [0, .7071, 0, -.7071] 
            
            coord1 =  geometry_msgs.msg.PoseStamped()
            coord1.header.frame_id =  'base_marker_pose'
            c1 = [0, 0, 0] + rotate90ypos
            coord1.pose = gen_utils.vecToRosPose(c1)

            try:
                tp1 = tformer.transformPose('torso_lift_link',coord1)
                pose1 = gen_utils.rosPoseToVec(tp1.pose)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                print "\n drawRigidDirections(): TF transform problem!"

            coord2 =  geometry_msgs.msg.PoseStamped()
            coord2.header.frame_id =  'base_marker_pose'
            c2 = [0, 0, 0] + rotate90yneg
            coord2.pose = gen_utils.vecToRosPose(c2)

            try:
                tp2 = tformer.transformPose('torso_lift_link',coord2)
                pose2 = gen_utils.rosPoseToVec(tp2.pose)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                print "\n drawRigidDirections(): TF transform problem!"

            draw_utils.drawArrow(pose1, [0.2, 0.015, 0.015], [.5,0,.5,1], 5)
            draw_utils.drawArrow(pose2, [0.2, 0.015, 0.015], [.5,0,.5,1], 6)
            
        rospy.sleep(0.1) 
        
        
  
if __name__ == '__main__':
    
    rospy.init_node('rvizArtic')
    
    #r = 0.1024
    #c = [-0.02215, -0.02017, -0.0014]
    #a = [-0.0302, -0,0407, 0.99867, -0.00899]
    #min_q = -2.755 
    #max_q = -2.5 
    #drawWedge(1, r, c, a, min_q, max_q)


    r =  0.12634677978
    o = [0.0768123576463,-0.00533408064445,0.667130229488,0.74095119016]
    drawGraspAxis(0,r,o)
