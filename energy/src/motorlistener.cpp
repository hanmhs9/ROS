#include "ros/ros.h"
#include "std_msgs/String.h"
#include "energy/realVal.h"

void commandCallback(const energy::realVal::ConstPtr& msg)
{
    ROS_INFO("\nMotor1 Velocity (rpm): %d\nMotor2 Position (qc): %.1f\nMotor3 Current (mA): %.3f\nMotor1 Voltage (V) : %.3f Current (mA) : %.3f\nMotor2 Voltage (V) : %.3f Current (mA): %.3f\n Motor1 Torque (Nm) : %.3f\n", msg->realVel, msg->realPos, msg->realCur, msg->volt1, msg->cur_node1, msg->volt2, msg->cur_node2, msg->torque);
//    ROS_INFO("\nMotor1 Velocity (rpm): %d\nMotor2 Position (qc): %d\nMotor3 Current (mA): %f\nMotor1 Voltage (V): %f\nMotor2 Voltage (V): %f\n", msg->realVel, msg->realPos, msg->realCur, msg->volt1, msg->volt2);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motorlistener");

    ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("measure", 1000, commandCallback);
	
	ros::Rate loop_rate(100);
	while(ros::ok())
	{	
	    ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}
