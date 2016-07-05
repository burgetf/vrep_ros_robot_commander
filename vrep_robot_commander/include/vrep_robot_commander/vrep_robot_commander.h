/*
 * vrep_robot_commander.h
 *
 *  Created on: July 05, 2015
 *      Author: Felix Burget
 */


// --- Includes -- 
#include <ros/ros.h>

// Used data structures:
#include <vrep_robot_commander/v_repConst.h>

//#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/simRosSetObjectPose.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosGetJointState.h"
#include "vrep_common/simRosSetRobotPose.h"
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosSetObjectIntParameter.h"


// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"


#ifndef ROBOT_COMMANDER_VREP_H
#define ROBOT_COMMANDER_VREP_H

// --Namespaces --
using namespace std;


namespace vrep{


class RobotCommander
{
	public:
    RobotCommander();
    ~RobotCommander();


	//Set initial Robot Pose (Start Config)
    void setInitialRobotPose();

    //Set the joint control modes
    bool setJointControlModes(vector<int> joint_control_modes);

    //Compute and execute required joint velocities
    void setJointSpeedsManipulator(vector<double> joint_velocities);

    //Stop motion of entire manipulator
    bool stopManipulatorMotion();

    //Stop motion of single manipulator joint
    bool stopManipulatorJointMotion(int joint_num);

    
    private:

    //ROS Node Handle
    ros::NodeHandle m_nh_;

    // -------- VREP Stuff

    //Vrep motor handle ID's
    vector<int> m_motor_handles;
    
    //Flag indicating whether motor handles are available
    bool m_motor_handles_available;

    //Subcriber for motor handles
    ros::Subscriber m_subMotorHandles;
    
    //Motor Handles callback (triggered by vrep_ros_communication node)
    void motorhandlesCallback(const std_msgs::Int32MultiArray::ConstPtr& handles_array);


    //Control modes for the robot joints
    vector<int> m_joint_control_modes;
    
    //Set initial Control Modes for Joints
    void setInitialControlModes();
    

	//Subscriber for V-REP's info stream (that stream is the only one enabled by default,
    // and the only one that can run while no simulation is running):
    ros::Subscriber m_subInfo;
    
    //Callback for VREP info topic
    void infoCallback(const vrep_common::VrepInfo::ConstPtr& info);

    // Global variables (modified by topic subscribers):
    bool m_simulationRunning;
    float m_simulationTime;


    //Publisher for omnirob and robotino wheel speeds / LBR joints motor speeds:
    ros::Publisher m_lbr_motor_speedPub;
    
    // Service client to get current joint positions of manipulator
    ros::ServiceClient client_joint_config;


    
    
};


}//end of namespace

#endif // ROBOT_COMMANDER_VREP_H

