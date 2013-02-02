#!/usr/bin/env python

"""
  simple_arm_server.py - this is a simple way to move an arm
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

import roslib; roslib.load_manifest('simple_arm_server')
import rospy, actionlib

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import *

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from actionlib_msgs.msg import GoalStatus
from kinematics_msgs.msg import KinematicSolverInfo, PositionIKRequest
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from simple_arm_server.msg import *
from simple_arm_server.srv import *

from math import *

class SimpleArmServer:
    """ 
        A class to move a 4/5/6DOF arm to a designated location (x,y,z), 
        respecting wrist rotation (roll) and gripper rotation (pitch). 
    """

    def __init__(self):
        rospy.init_node("simple_arm_server")        
        
        # configuration
        self.dof = rospy.get_param("~arm_dof", 5)
        self.root = rospy.get_param("~root_name","base_link")
        self.tip = rospy.get_param("~tip_name","gripper_link")
        self.timeout = rospy.get_param("~timeout",5.0)
        self.offset = rospy.get_param("~offset",0.0)

        # connect to arm kinematics
        rospy.wait_for_service('arm_kinematics/get_ik')
        rospy.wait_for_service('arm_kinematics/get_ik_solver_info')
        self._get_ik_proxy = rospy.ServiceProxy('arm_kinematics/get_ik', GetPositionIK, persistent=True)
        self._get_ik_solver_info_proxy = rospy.ServiceProxy('arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
    
        # setup tf for translating poses
        self._listener = tf.TransformListener()

        # a publisher for arm movement
        self._client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        self._client.wait_for_server()
        self._gripper = rospy.Publisher('gripper_controller/command', Float64)

        # listen to joint states
        self.servos = dict()
        rospy.Subscriber('joint_states', JointState, self.stateCb)

        # load action for moving arm
        self.server = actionlib.SimpleActionServer("move_arm", MoveArmAction, execute_cb=self.actionCb, auto_start=False)
        self.server.start()

        # service for checking trajectory
        rospy.Service('~get_trajectory_validity', GetTrajectoryValidity, self.validityCb)

        rospy.loginfo('simple_arm_server node started')
        rospy.spin()


    def validityCb(self, req):
        """ Check a trajectory for validity. """
        for action in req.motions:
            if action.type == ArmAction.MOVE_ARM:
                if self.motionToTrajectory(action, req.header.frame_id) == None:
                    return GetTrajectoryValidityResponse(False)
        return GetTrajectoryValidityResponse(True)


    def stateCb(self, msg):
        """ Update our known location of joints, for interpolation. """
        for i in range(len(msg.name)):
            joint = msg.name[i]
            self.servos[joint] = msg.position[i]


    def actionCb(self, req):
        """ Given pose to move gripper to, do it. """ 
        self.arm_solver_info = self._get_ik_solver_info_proxy() 
        computed_actions = list()       

        # compute trajectories
        for action in req.motions:
            if self.server.is_preempt_requested():
                self.server.set_preempted( MoveArmResult(False) )
                return
            if action.type == ArmAction.MOVE_ARM:
                msg = self.motionToTrajectory(action, req.header.frame_id)
                if msg != None:
                    computed_actions.append( [ 'move', msg, msg.points[0].time_from_start ] )
                else:
                    self.server.set_aborted( MoveArmResult(False) )
                    return
            else:
                computed_actions.append( [ 'grip', Float64(action.command), action.move_time ] )

        # smooth and execute trajectories
        traj = None
        for action in computed_actions:
            msg = action[1]
            time = action[2]
            if action[0] == 'move':
                # build trajectory
                if traj == None:
                    traj = msg
                else:
                    msg.points[0].time_from_start += traj.points[-1].time_from_start
                    traj.points.append(msg.points[0])
            else: 
                # move arm
                if traj != None:
                    if not self.executeTrajectory(traj):
                        return
                    traj = None
                # move gripper
                rospy.loginfo("Move gripper to " + str(msg.data))
                self._gripper.publish( msg )
                rospy.sleep( time )
        if traj != None:
            print traj
            if not self.executeTrajectory(traj):
                return

        self.server.set_succeeded( MoveArmResult(True) )
        return


    def executeTrajectory(self, traj):
        """ Execute a trajectory. """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        traj.header.stamp = rospy.Time.now()
        rospy.loginfo("Sending action with " + str(len(traj.points)) + " points.")
        self._client.send_goal(goal)
        while self._client.get_state() != GoalStatus.SUCCEEDED:
            if self.server.is_preempt_requested():
                self._client.cancel_goal()
                self.server.set_preempted( MoveArmResult(False) )
                return False
        rospy.loginfo("Action succeeded: " + str(self._client.wait_for_result()))   
        return True
   

    def motionToTrajectory(self, action, frame_id):
        """ Convert an arm movement action into a trajectory. """
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose = action.goal
        pose = self._listener.transformPose(self.root, ps)
        rospy.loginfo("Parsing move to:\n" + str(pose))
        
        # create IK request
        request = GetPositionIKRequest()
        request.timeout = rospy.Duration(self.timeout)

        request.ik_request.pose_stamped.header.frame_id = self.root;
        request.ik_request.ik_link_name = self.tip;
        request.ik_request.pose_stamped.pose.position.x = pose.pose.position.x
        request.ik_request.pose_stamped.pose.position.y = pose.pose.position.y
        request.ik_request.pose_stamped.pose.position.z = pose.pose.position.z

        e = euler_from_quaternion([pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w])
        e = [i for i in e]
        if self.dof < 6:
            # 5DOF, so yaw angle = atan2(Y,X-shoulder offset)
            e[2] = atan2(pose.pose.position.y, pose.pose.position.x-self.offset)
        if self.dof < 5:
            # 4 DOF, so yaw as above AND no roll
            e[0] = 0
        q =  quaternion_from_euler(e[0], e[1], e[2])

        request.ik_request.pose_stamped.pose.orientation.x = q[0]
        request.ik_request.pose_stamped.pose.orientation.y = q[1]
        request.ik_request.pose_stamped.pose.orientation.z = q[2]
        request.ik_request.pose_stamped.pose.orientation.w = q[3]

        request.ik_request.ik_seed_state.joint_state.name = self.arm_solver_info.kinematic_solver_info.joint_names
        request.ik_request.ik_seed_state.joint_state.position = [self.servos[joint] for joint in request.ik_request.ik_seed_state.joint_state.name]

        # get IK, wiggle if needed
        tries = 0
        pitch = e[1]
        print "roll", e[0]
        while tries < 80:
            try:
                response = self._get_ik_proxy(request)
                if response.error_code.val == response.error_code.SUCCESS:
                    break
                else:
                    tries += 1
                    # wiggle gripper
                    pitch = e[1] + ((-1)**tries)*((tries+1)/2)*0.025
                    # update quaternion
                    q = quaternion_from_euler(e[0], pitch, e[2])
                    request.ik_request.pose_stamped.pose.orientation.x = q[0]
                    request.ik_request.pose_stamped.pose.orientation.y = q[1]
                    request.ik_request.pose_stamped.pose.orientation.z = q[2]
                    request.ik_request.pose_stamped.pose.orientation.w = q[3]
            except rospy.ServiceException, e:
                print "Service did not process request: %s"%str(e)

        print response
        if response.error_code.val == response.error_code.SUCCESS:
            arm_solver_info = self._get_ik_solver_info_proxy()     
            msg = JointTrajectory()
            msg.joint_names = arm_solver_info.kinematic_solver_info.joint_names
            msg.points = list()
            point = JointTrajectoryPoint()
            point.positions = [ 0.0 for servo in msg.joint_names ]
            point.velocities = [ 0.0 for servo in msg.joint_names ]
            for joint in request.ik_request.ik_seed_state.joint_state.name:
                i = msg.joint_names.index(joint)
                point.positions[i] = response.solution.joint_state.position[response.solution.joint_state.name.index(joint)] 
            if action.move_time > rospy.Duration(0.0):
                point.time_from_start = action.move_time
            else:
                point.time_from_start = rospy.Duration(5.0)
            msg.points.append(point)
            msg.header.stamp = rospy.Time.now() + rospy.Duration(0.01)

            return msg
        else:
            return None


if __name__ == '__main__':
    try:
        SimpleArmServer()
    except rospy.ROSInterruptException:
        rospy.loginfo("And that's all folks...")

