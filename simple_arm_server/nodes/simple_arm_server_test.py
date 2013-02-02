#!/usr/bin/env python

"""
  simple_arm_server_test.py - tests the simple_arm_server.py program
  Copyright (c) 2011 Michael Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holders nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

usage_= "usage: simple_arm_server_test.py x y z wrist_pitch [wrist_roll=0.0 wrist_yaw=0.0 frame_id='base_link' duration=5.0]"

import roslib; roslib.load_manifest('simple_arm_server')
import rospy, actionlib
import sys

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from simple_arm_server.msg import *

if __name__ == '__main__':
    if len(sys.argv) > 4:     
        rospy.init_node('simple_arm_server_test')

        client = actionlib.SimpleActionClient('move_arm', MoveArmAction)
        client.wait_for_server()

        goal = MoveArmGoal()
        goal.header.frame_id = "base_link"
        if len(sys.argv) > 7:
            goal.header.frame_id = sys.argv[7]

        action = ArmAction()
        action.type = ArmAction.MOVE_ARM
        action.goal.position.x = float(sys.argv[1])
        action.goal.position.y = float(sys.argv[2])
        action.goal.position.z = float(sys.argv[3])

        roll = 0.0
        if len(sys.argv) > 5:
            roll = float(sys.argv[5])
        yaw = 0.0
        if len(sys.argv) > 6:
            yaw = float(sys.argv[6])
        q = quaternion_from_euler(roll, float(sys.argv[4]), yaw, 'sxyz')
        action.goal.orientation.x = q[0]
        action.goal.orientation.y = q[1]
        action.goal.orientation.z = q[2]
        action.goal.orientation.w = q[3]

        if len(sys.argv) > 8:
            action.move_time = rospy.Duration(float(sys.argv[8]))

        try:
            goal.motions.append(action)
            client.send_goal(goal)
            client.wait_for_result()   
            r = client.get_result()
            print r
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)
    else:
        print usage_

