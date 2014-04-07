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
        
        
       
def drawGraspAxis(tag_id, center, axis):

    gen_utils = generalUtils.GeneralUtils()
    draw_utils = drawUtils.DrawUtils()
    wm = arWorldModel.ARWorldModel()
    tformer = tf.TransformerROS(True, rospy.Duration(10.0))
    
    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        tag_pose = (wm.getPartialWorldState([tag_id]))[tag_id]

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
        coord1.header.frame_id =  'circ_axis_pose'
        c1 = [0, 0, 0, 0, 0, 0, 1]
        coord1.pose = gen_utils.vecToRosPose(c1)
        
        #Finally, do the transforms to get in robot frame
        try:
            tp1 = tformer.transformPose('torso_lift_link',coord1)
            grasp_axis = gen_utils.rosPoseToVec(tp1.pose)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            print "\n drawWedge(): TF transform problem!"
        
        draw_utils.drawArrow(grasp_axis, [0.1, 0.01, 0.01], [0,1,0,1])
        
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
    traj_y = [traj[i][0] for i in xrange(len(traj))]
    traj_z = [traj[i][0] for i in xrange(len(traj))]
    
    draw_x = draw_y = draw_z = False
    if max(traj_x) - min(traj_x) > eps:
        draw_x = True
    if max(traj_y) - min(traj_y) > eps:
        draw_y = True
    if max(traj_z) - min(traj_z) > eps:
        draw_z = True
    
    while not rospy.is_shutdown():
        tag_pose = (wm.getPartialWorldState([tag_id]))[tag_id]
        tpq = tf.Quaternion(tag_pose[3:])  
        
        if draw_x:
            rotate180z = tf.createQuaternionFromRPY(0,0,np.pi)
            q2 = tpq * rotate180z
            pose2 = tag_pos[0:3] + [q2.x, q2.y, q2.z, q2.w]
            draw_utils.drawArrow(tag_pose)
            draw_utils.drawArrow(pose2)
            
        if draw_y:
            rotate90zpos = tf.createQuaternionFromRPY(0,0,np.pi/2.0)
            rotate90zneg = tf.createQuaternionFromRPY(0,0,-np.pi/2.0)
            q1 = tpq * rotate90zpos
            q2 = tpq * rotate90zneg
            pose1 = tag_pos[0:3] + [q1.x, q1.y, q1.z, q1.w]
            pose2 = tag_pos[0:3] + [q2.x, q2.y, q2.z, q2.w]
            draw_utils.drawArrow(pose1)
            draw_utils.drawArrow(pose2)
            
        if draw_z:
            rotate90ypos = tf.createQuaternionFromRPY(0.0, np.pi/2.0, 0.0)
            rotate90yneg = tf.createQuaternionFromRPY(0,0, -np.pi/2.0, 0,0)
            q1 = tpq * rotate90ypos
            q2 = tpq * rotate90yneg
            pose1 = tag_pos[0:3] + [q1.x, q1.y, q1.z, q1.w]
            pose2 = tag_pos[0:3] + [q2.x, q2.y, q2.z, q2.w]
            draw_utils.drawArrow(pose1)
            draw_utils.drawArrow(pose2)
            
        rospy.sleep(0.1) 
        
        
  
if __name__ == '__main__':
    
    rospy.init_node('rvizArtic')
    
    #r = 0.1024
    #c = [-0.02215, -0.02017, -0.0014]
    #a = [-0.0302, -0,0407, 0.99867, -0.00899]
    #min_q = -2.755 
    #max_q = -2.5 
    #drawWedge(1, r, c, a, min_q, max_q)

    #prismatic_dir.x : -0.215005492967
    #prismatic_dir.y : 0.925764394093
    #prismatic_dir.z : 0.311019170829
    #rigid_position.x : -0.0225000537285
    #rigid_position.y : -0.109253380959
    #rigid_position.z : -0.0523021798595

    c = [-0.00438106158762,0.0219812866424,-0.0291447669762]
    a = [0.154436288188,0.134253967621,-0.377609576905,0.903070491433]
    drawGraspAxis(0,c,a)
