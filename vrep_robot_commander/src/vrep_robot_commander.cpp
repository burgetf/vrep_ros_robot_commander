#include <vrep_robot_commander/vrep_robot_commander.h>

namespace vrep{

//Constructor
RobotCommander::RobotCommander()
{
       
    //Control modes for the robot joints
    m_joint_control_modes.resize(7);

    // Global variables (modified by topic subscribers):
    m_simulationRunning=true;
    m_simulationTime=0.0f;
    m_motor_handles_available = false;

    // 1. Let's subscribe to V-REP's info stream (that stream is the only one enabled by default,
    // and the only one that can run while no simulation is running):
    m_subInfo = m_nh_.subscribe("/vrep/info",1,&RobotCommander::infoCallback,this);

    //Prepare Subcriber for motor handles:
    m_subMotorHandles = m_nh_.subscribe("/vrep_ros_communication/motor_handles",1,&RobotCommander::motorhandlesCallback,this);

    //Prepare Subcriber and Publisher
    m_lbr_motor_speedPub = m_nh_.advertise<vrep_common::JointSetStateData>("/vrep_ros_communication/lbr_joints",1);   

    // Service client to get current joint positions of manipulator from Vrep
    client_joint_config = m_nh_.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState");


    //Start the simulation mode in VREP
    ros::ServiceClient client_startSimulation=m_nh_.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srv_startSimulation;
    client_startSimulation.call(srv_startSimulation);
    

    //Wait until motor handles become available (published by vrep_ros_communication node)
    while(m_motor_handles_available == false)
    {
        ROS_INFO("Waiting for motor handles callback. Make sure to activate simulation in V-REP!!!");

        // handle ROS messages:
        ros::spinOnce();

        // sleep a bit:
        usleep(5000);
    }
    ROS_INFO("Robot motor handles available!");
    
    
    //Set the joint control modes (position or velocity mode)
    setInitialControlModes();
   

}

//Destructor
RobotCommander::~RobotCommander()
{
	//Nothing to do yet
}



// Topic subscriber callbacks:
void RobotCommander::infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
    m_simulationTime=info->simulationTime.data;
    m_simulationRunning=(info->simulatorState.data&1)!=0;
}


void RobotCommander::motorhandlesCallback(const std_msgs::Int32MultiArray::ConstPtr& handles_array)
{
    //Set motor handle vector size
    m_motor_handles.resize(handles_array->data.size());

    //Get the motor handle ID's from the message
    int i = 0;
    for(std::vector<int>::const_iterator it = handles_array->data.begin(); it != handles_array->data.end(); ++it)
        {
            m_motor_handles[i] = *it;
            i++;
        }

    //Motor Handles are now set
    m_motor_handles_available = true;

}



//Set the joint control modes
bool RobotCommander::setJointControlModes(vector<int> joint_control_modes)
{
    bool modes_set = false;

    //Publish control modes
    ros::ServiceClient client_setControlModes=m_nh_.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter setControlModes;
    //Set streamCmd
    setControlModes.request.parameter = sim_jointintparam_ctrl_enabled;

    //Set Motor control mode for Robot Joints (last handle is for the entire robot -> used to get the current robot pose)
    for (int i = 0; i < m_motor_handles.size()-1; i++)
    {
        setControlModes.request.handle = m_motor_handles[i];
        setControlModes.request.parameterValue = joint_control_modes[i];
        //Set motor control modes for current joint
        modes_set = client_setControlModes.call(setControlModes);

        //cout<<m_motor_handles[i] <<" : "<<joint_control_modes[i];
    }

	// handle ROS messages:
    ros::spinOnce();
    
    return modes_set;
}



//Set initial Robot Pose (Start Config)
void RobotCommander::setInitialRobotPose(){

	//Service to set robot pose
    ros::ServiceClient client_initialRobotPose=m_nh_.serviceClient<vrep_common::simRosSetRobotPose>("/vrep/simRosSetInitialRobotPose");
    vrep_common::simRosSetRobotPose robot_pose;
   
   
    //Initial configuration for LBR Joints -> angle in [rad]
    robot_pose.request.position.push_back(0.0);
    robot_pose.request.position.push_back(0.0);
    robot_pose.request.position.push_back(0.0);
    robot_pose.request.position.push_back(0.0);
    robot_pose.request.position.push_back(0.0);
    robot_pose.request.position.push_back(0.0);
    robot_pose.request.position.push_back(0.0);

    //Set initial pose for robot
    client_initialRobotPose.call(robot_pose);

    cout<<"Set initial robot pose..."<<endl;

    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(2000);
}


//Set initial Control Modes for Joints
void RobotCommander::setInitialControlModes(){

    //Set Control Modes
    //vector<int> control_modes_values; //position control enabled -> 0 no (i.e. velocity control is used), 1 yes
    
    //LBR Joints (velocity mode = 0 or position mode = 1)
    m_joint_control_modes[0] = 0;
    m_joint_control_modes[1] = 0;
    m_joint_control_modes[2] = 0;
    m_joint_control_modes[3] = 0;
    m_joint_control_modes[4] = 0;
    m_joint_control_modes[5] = 0;
    m_joint_control_modes[6] = 0;

     
    //Set the joint control modes
    setJointControlModes(m_joint_control_modes);

    // handle ROS messages:
    ros::spinOnce();
}



//Compute and execute required joint velocities
void RobotCommander::setJointSpeedsManipulator(vector<double> joint_velocities){

    //Kuka LBR Manipulator
    float desired_LBR_joint1;
    float desired_LBR_joint2;
    float desired_LBR_joint3;
    float desired_LBR_joint4;
    float desired_LBR_joint5;
    float desired_LBR_joint6;
    float desired_LBR_joint7;

    desired_LBR_joint1 = joint_velocities[0];
    desired_LBR_joint2 = joint_velocities[1];
    desired_LBR_joint3 = joint_velocities[2];
    desired_LBR_joint4 = joint_velocities[3];
    desired_LBR_joint5 = joint_velocities[4];
    desired_LBR_joint6 = joint_velocities[5];
    desired_LBR_joint7 = joint_velocities[6];

    vrep_common::JointSetStateData lbr_motor_speeds;

    //Publish the motor speeds for the joints of the Kuka LBR:
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[0]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[1]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[2]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[3]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[4]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[5]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[6]);


    //Modes (set Joint Dynamic Properties in V-Rep accordingly)
    // 0: sets the position
    // 1: sets the target position (when joint is dynamically enabled and in position control)
    // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
    // 3: sets the maximum force/torque that the joint can exert
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);

    lbr_motor_speeds.values.data.push_back(desired_LBR_joint1);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint2);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint3);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint4);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint5);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint6);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint7);

    m_lbr_motor_speedPub.publish(lbr_motor_speeds);
    
    // handle ROS messages:
    ros::spinOnce();
}



bool RobotCommander::stopManipulatorMotion(){

    vrep_common::JointSetStateData lbr_motor_speeds;

    //Kuka LBR Manipulator
    float desired_LBR_joint1;
    float desired_LBR_joint2;
    float desired_LBR_joint3;
    float desired_LBR_joint4;
    float desired_LBR_joint5;
    float desired_LBR_joint6;
    float desired_LBR_joint7;

    desired_LBR_joint1 = 0.0;
    desired_LBR_joint2 = 0.0;
    desired_LBR_joint3 = 0.0;
    desired_LBR_joint4 = 0.0;
    desired_LBR_joint5 = 0.0;
    desired_LBR_joint6 = 0.0;
    desired_LBR_joint7 = 0.0;


    //Publish the motor speeds for the joints of the Kuka LBR:
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[0]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[1]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[2]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[3]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[4]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[5]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[6]);


    //Modes (set Joint Dynamic Properties in V-Rep accordingly)
    // 0: sets the position
    // 1: sets the target position (when joint is dynamically enabled and in position control)
    // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
    // 3: sets the maximum force/torque that the joint can exert
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);

    lbr_motor_speeds.values.data.push_back(desired_LBR_joint1);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint2);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint3);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint4);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint5);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint6);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint7);

    //Publish motor speeds
    m_lbr_motor_speedPub.publish(lbr_motor_speeds);


    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(2000); //20ms -> 50Hz loop

    cout<<"Manipulator motion stopped!"<<endl;

    return true;

}


bool RobotCommander::stopManipulatorJointMotion(int joint_num)
{
    vrep_common::JointSetStateData lbr_motor_speeds;

    //Set desired joint velocity to zero
    float desired_LBR_joint = 0.0;

    //Publish the motor speeds for the joints of the Kuka LBR:
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[joint_num]);

    //Modes (set Joint Dynamic Properties in V-Rep accordingly)
    // 0: sets the position
    // 1: sets the target position (when joint is dynamically enabled and in position control)
    // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
    // 3: sets the maximum force/torque that the joint can exert
    lbr_motor_speeds.setModes.data.push_back(2);

    //Set speed in message
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint);

    //Publish motor speeds
    m_lbr_motor_speedPub.publish(lbr_motor_speeds);

    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(2000); //20ms -> 50Hz loop

    cout<<"Motion of joint: "<<joint_num<<" stopped!"<<endl;

    return true;
}



} //end of namespace


