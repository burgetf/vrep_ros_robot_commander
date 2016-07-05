/*
 *robot_commander_example.cpp
 *
 *  Created on: July 05, 2015
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <vrep_robot_commander/vrep_robot_commander.h>


using namespace std;

int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "robot_commander_example");

    //Node Handle
    ros::NodeHandle nh;

    //Robot Motion Commander (starts simulation in V-Rep)
    vrep::RobotCommander vrep_robo_com;

    //Define joint control modes (velocity mode = 0 or position mode = 1)
    vector<int> joint_control_modes(7);
    for(int i = 0 ; i <joint_control_modes.size() ; i++)
        joint_control_modes[i] = 0;
	
    //Send joint control modes to V-Rep
    vrep_robo_com.setJointControlModes(joint_control_modes);


    //Define joint velocities
    vector<double> joint_vels(7);
    joint_vels[0] = 0.1;
    joint_vels[1] = 0.1;
    joint_vels[2] = 0.1;
    joint_vels[3] = 0.1;
    joint_vels[4] = 0.1;
    joint_vels[5] = 0.1;
    joint_vels[6] = 0.1;
	
    while (ros::ok()){

        //Send joint velocity
        vrep_robo_com.setJointSpeedsManipulator(joint_vels);
    }


    ros::shutdown();

    return 0;
}


