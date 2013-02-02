/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Vanadium Labs LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Vanadium Labs LLC nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * author: Michael Ferguson
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <simple_arm_server/MoveArmAction.h>
#include <std_msgs/Float64.h>
#include <math.h>

class SimpleArmNavigationServer
{
  private:
    ros::NodeHandle nh;
    tf::TransformListener listener;

    // ROS interface
    actionlib::SimpleActionServer<simple_arm_server::MoveArmAction> server;
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> client;
    ros::Publisher gripper;

    simple_arm_server::MoveArmFeedback     feedback_;
    simple_arm_server::MoveArmResult       result_;
    simple_arm_server::MoveArmGoalConstPtr goal_;

    // parameters
    int arm_dof_;
    std::string root_name_;
    std::string tip_name_;
    std::string pan_link_;
    double timeout_;
    double x_offset_;
    int max_tries_;
    std::vector<double> step_list_;

  public:
    
    SimpleArmNavigationServer() : nh("~"), listener(nh), server(ros::NodeHandle(), "simple_move_arm", false), client("move_arm", true)
    {
        ROS_INFO("Starting simple_move_arm_server.");
        // configuration
        nh.param<int>("arm_dof", arm_dof_, 5);
        nh.param<std::string>("root_name", root_name_, "arm_link");
        nh.param<std::string>("tip_name", tip_name_, "gripper_link");
        nh.param<double>("timeout", timeout_, 5.0);
        nh.param<int>("iterations", max_tries_, 30);
        if(nh.hasParam("step_list")){
            XmlRpc::XmlRpcValue step_params;
            nh.getParam("step_list", step_params);
            for(int i=0; i<step_params.size(); ++i)
            {
                step_list_.push_back( static_cast<double>(step_params[i]) );
            }
        }else{
            step_list_.push_back(-0.05);
            step_list_.push_back( 0.05);
        }

        // lookup X offset of pan_link from root_name
        nh.param<std::string>("pan_link", pan_link_, "arm_link");
        if( !listener.waitForTransform(root_name_, pan_link_, ros::Time(0), ros::Duration(10.0)) ){
            ROS_ERROR("Cannot lookup transfrom from %s to %s", pan_link_.c_str(), root_name_.c_str());
        }
        tf::StampedTransform transform;
        try{
            listener.lookupTransform(root_name_, pan_link_, ros::Time(0), transform);
        }catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        x_offset_ = transform.getOrigin().x();
        ROS_INFO("X offset %f", x_offset_);

        // output for arm movement
        client.waitForServer();
        gripper = nh.advertise<std_msgs::Float64>("/gripper_controller/command", 1, false);
        ROS_INFO("Connected to move_arm server");

        // input for moving arm
        server.registerGoalCallback(boost::bind(&SimpleArmNavigationServer::goalCB, this));
        server.registerPreemptCallback(boost::bind(&SimpleArmNavigationServer::preemptCB, this));
        server.start();
        
        ROS_INFO("simple_move_arm_server node started");
    }

    void goalCB()
    {
        // accept the new goal
        goal_ = server.acceptNewGoal();
        
        for(size_t i = 0; i < goal_->motions.size(); i++){
            simple_arm_server::ArmAction action = goal_->motions[i];

            if( action.type == simple_arm_server::ArmAction::MOVE_ARM )
            {
                // transform goal
                geometry_msgs::PoseStamped in, pose;
                in.header = goal_->header;
                in.pose = action.goal;
                try{
                    listener.transformPose(root_name_, in, pose);
                }catch(tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                }
                // our hacks for low DOF
                double roll, pitch, yaw;
                btQuaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
                btMatrix3x3(q).getRPY(roll, pitch, yaw);
                if( arm_dof_ < 6 ){
                    // 5DOF, so yaw angle = atan2(Y,X-shoulder offset)
                    yaw = atan2(pose.pose.position.y, pose.pose.position.x-x_offset_);
                    if( arm_dof_ < 5 ){
                        // 4 DOF, so yaw as above AND no roll
                        roll = 0.0;
                    }
                }

                // wiggle if needed
                int tries;
                bool goal_reached = false;
                for( tries = 0; tries <= max_tries_; tries++ ){
                    for(size_t i=0; i<step_list_.size(); i++){
                        // construct the representation of a pose goal (define the position of the end effector)
                        arm_navigation_msgs::MoveArmGoal goal;
                        
                        goal.motion_plan_request.group_name = "arm";
                        goal.motion_plan_request.num_planning_attempts = 1;
                        goal.motion_plan_request.planner_id = std::string("");
                        goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
                        goal.motion_plan_request.allowed_planning_time = ros::Duration(timeout_);

                        arm_navigation_msgs::SimplePoseConstraint desired_pose;
                        desired_pose.absolute_position_tolerance.x = 0.005;
                        desired_pose.absolute_position_tolerance.y = 0.005;
                        desired_pose.absolute_position_tolerance.z = 0.005;
                        desired_pose.absolute_roll_tolerance = 0.25;
                        desired_pose.absolute_pitch_tolerance = 0.05;
                        desired_pose.absolute_yaw_tolerance = 0.5;

                        desired_pose.header.frame_id = pose.header.frame_id;
                        desired_pose.link_name = tip_name_;

                        desired_pose.pose.position.x = pose.pose.position.x;
                        desired_pose.pose.position.y = pose.pose.position.y;
                        desired_pose.pose.position.z = pose.pose.position.z;

                        double attempt = pitch + (tries * step_list_[i]);
                        q.setRPY(roll, attempt, yaw);

                        desired_pose.pose.orientation.x = (double) q.getX();
                        desired_pose.pose.orientation.y = (double) q.getY();
                        desired_pose.pose.orientation.z = (double) q.getZ();
                        desired_pose.pose.orientation.w = (double) q.getW();
                        ROS_INFO("%d: (%f, %f, %f)", tries, roll, attempt, yaw);

                        arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose, goal);

                        // send request
                        client.sendGoal(goal);
                        bool finished_within_time = client.waitForResult(ros::Duration(300.0));
                        if( !finished_within_time )
                        {
                            client.cancelGoal();
                            ROS_INFO("Continue: goal timed out");
                        }
                        else
                        {
                            actionlib::SimpleClientGoalState state = client.getState();
                            arm_navigation_msgs::MoveArmResultConstPtr result = client.getResult();
                            //if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
                            if(result->error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS ){
                                ROS_INFO("Continue: goal succeeded.");
                                goal_reached = true;
                            }else if( result->error_code.val == int(arm_navigation_msgs::ArmNavigationErrorCodes::START_STATE_IN_COLLISION) ){
                                ROS_INFO("Abort: start state in collision.");
                                result_.success = false;
                                server.setAborted(result_);
                                return;
                            }else if( (result->error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::GOAL_CONSTRAINTS_VIOLATED) ||
                                      (result->error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::PATH_CONSTRAINTS_VIOLATED) ) {
                                ROS_INFO("Continue: goal constraints violated.");
                                goal_reached = true;
                            }else{
                                ROS_INFO("Continue: path failed, trying again");
                            }
                        }
                        if(tries==0) break;
	                    if(goal_reached) break;
                    } // end of steps
	                if(goal_reached) break;
                } // end of tries                   
                if( tries > max_tries_ ){
                    result_.success = false;
                    server.setAborted(result_);
                    return;                    
                }
            }
            else if( action.type == simple_arm_server::ArmAction::MOVE_GRIPPER )
            {
                ROS_INFO("Move gripper to %f.", action.command);
                std_msgs::Float64 msg;
                msg.data = action.command;
                gripper.publish( msg );
                ros::Duration(action.move_time).sleep();
            }
        } // for each action

        result_.success = true;
        server.setSucceeded(result_);
    }

    void preemptCB()
    {
        ROS_INFO("simple_move_arm preempted");
        server.setPreempted();
    }

};

int main(int argc, char **argv)
{
    ros::init (argc, argv, "simple_move_arm_server");
    
    SimpleArmNavigationServer server;
    ros::spin();
    
    return 0;
}

