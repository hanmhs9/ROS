#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "energy/realVal.h"
#include "energy/Val.h"
#include <termios.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>

float gear_ratio = 1/4.8 * 10/22 * 19/43; //0.04184
float J_motor = 79*pow(10,-7);
float r_range[563] = {0.0563};
float actual_r = 0.0563;
int prev_qc = 0;
bool fold = 0; //if 1, gear touching right, if 0, gear touching left

float R = 0.582; 
float Km = 29.2; 
float Kn = 328;
float Io = 0.128; 
int no = 7750;
float m = 4.5;
float m1 = 1.8; 
float mur = 0.01;
float g = 9.81; 
float rho = 1.225; 
float As = (0.27*0.19 + 2*(0.1*0.045))* 0.05;
float Cd = 0.5;

//float maxEff[60001] = {0};


void commandCallback(const energy::realVal::ConstPtr& msg)
{
    //ROS_INFO("Motor1 Velocity (rpm): %d\nMotor2 Position (qc): %.4f\nMotor3 Current (mA): %f", msg->realVel, msg->realPos, msg->realCur);
	int qc;
	qc = msg->realPos;
	int updated_qc;
	if(prev_qc < qc) 
		fold = 0;
	else if(prev_qc >= qc) 
		fold = 1;
	
	if(fold==0) {
		updated_qc = qc + 45000;
		if(updated_qc > 300000)
			updated_qc = 300000;	
		actual_r = updated_qc / 300000 * 0.0563 + 0.0563;
		prev_qc = qc;
	}
	else { //fold == 1
		updated_qc = qc - 45000; 
		if(updated_qc < 0) 
			updated_qc = 0;
		actual_r = updated_qc / 300000 * 0.0563 + 0.0563;
		prev_qc = qc; 
	}
}

float * getValue(float a, float v, int slope)
{
	float theta = slope * M_PI/180;
	float Fds = 0.5*rho*pow((v*gear_ratio),2) * Cd * As; 
	float M[563] = {0};
	float Td = Fds * (0.19/2) * 0.05;
	float eff[563];
	float maxEff = 0;
	int index = 0;

	for(int i=0;i<563;i++){
		M[i] = (mur*m*g*r_range[i]*cos(theta) + m*g*r_range[i]*sin(theta) + Td + a*gear_ratio/r_range[i] * (0.5*m1*pow(r_range[i],2) + J_motor/(gear_ratio*gear_ratio) + (85.714*pow(10,-9))/((gear_ratio*4.8)*(gear_ratio*4.8)) + m*pow(r_range[i],2)))*gear_ratio*1000 + Io*Km;
		if(M[i] < Io*Km)
			M[i]=fabs(Io*Km - M[i] + Io*Km);
		eff[i] = fabs(1-((Kn*Km*Km*M_PI*r_range[i]/(v*30*Km*Km+30000*R*M[i]*r_range[i])) * (M[i]*R/Km-Io*R)+Io*Km/M[i])); 
		if(eff[i] > maxEff){ 
			maxEff = eff[i];
			index = i;
		}
	}
	//std::cout<<maxEff<<" , "<<index<<" , " << r_range[index]<<std::endl;
	static float effnR[2]; 
	effnR[0] = maxEff; //efficiency_max
	effnR[1] = r_range[index]; //desired Radius
	return effnR;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motortalker");

    ros::NodeHandle n;
	for(int i=1;i<563;i++){
		r_range[i]=r_range[i-1]+0.0001;
	}	
    ros::Publisher command_pub = n.advertise<energy::Val>("command", 1000);
	ros::Subscriber sub = n.subscribe("measure", 1000, commandCallback);

    int arraySize = 0;
	std::string s;
    std::cout << "Motor Talker running \n";
	//std::ifstream velText("velocities.txt", std::ios_base::ate);
	std::ifstream velText("velocities.txt");	
	if(velText.is_open()) {
		while(getline(velText, s))
			arraySize ++;
		velText.clear();
		velText.seekg(0, velText.beg); //go back to the beginning of the file
		std::cout << "Array Size : " << arraySize << "\n";
		//velText.seekg(0, std::ios::beg);

			
	} else {
		std::cout<<"Cannot open file \n";
	}

	float velocities[arraySize] = {0};
	float acceleration[arraySize] = {0}; 
   	int rpm_s[arraySize] = {0};
	int rpm_l[arraySize] = {0};
	float loadM[arraySize] = {0};
	float current[arraySize] = {0};
	int scaledRPM_s[arraySize] = {0};
	int scaledRPM_l[arraySize] = {0};
	int max_s = 0;
	int max_l = 0;
	float dt = 0.01;
	float m3Io = 0.130; //A
	int slopeAngle = 5.491;
//	velText.clear();
//	velText.seekg(0, std::ios::beg);
	for(int i=0; i<arraySize; i++){ 
		velText >> velocities[i];
	}
	
	std::cout << "text file reading done.. closing file stream \n";
	velText.close();

	for(int i=1; i<arraySize; i++) {
		acceleration[i] = (velocities[i] - velocities[i-1]) / dt;
	}	

	std::ifstream load("load0rmin.txt");
//	std::ifstream load("load0rmax.txt");
//	std::ifstream load("load25rmin.txt");
//	std::ifstream load("load-20rmin.txt");
	if(load.is_open()) {
		for(int i=0; i<arraySize; i++) {
			load >> loadM[i];
		}
	}
	else 
		std::cout <<"Cannot open file \n";
	load.close();
	
    for(int i =0; i < arraySize; i++) {
       rpm_s[i] = velocities[i] / 0.0563 * 30 / 3.141592;
	   rpm_l[i] = velocities[i] / 0.1050 * 30 / 3.141592;
       if(rpm_s[i]>max_s) 
			max_s = rpm_s[i];
	   if(rpm_l[i]>max_l)
			max_l = rpm_l[i];
		
		current[i] = -(loadM[i]*1000/86/12.9 + m3Io)*1000;
      // usleep(1000);
    }
	for(int i=0; i < arraySize; i++)
	{
		scaledRPM_s[i] = rpm_s[i]*(max_s/2)/max_s;
		scaledRPM_l[i] = rpm_l[i]*(max_l/2)/max_l;
	}

	energy::Val msg;
    int count = 0;
	int j = 0;
	float radius;
	ros::Rate loop_rate(100);
	
	int expNum = 1; //1~12
	

    while (ros::ok())
    {
		while(j<arraySize){
			if(expNum == 1) { //Flat Ground, Small Wheel 
     			msg.velocity = scaledRPM_s[j] / gear_ratio;
		        ROS_INFO("Desired RPM: %d rpm", msg.velocity);
				msg.position = 0;
				//radius = (msg.position)/2121728.0*360/28*0.0563 + 0.0563;
				radius = 56.3;				
				ROS_INFO("Radius : %.1f mm\n", radius);
				//msg.current = current[j];
				msg.current = -(loadM[j]*1000/86/12.9 + m3Io) * 1000;		
				ROS_INFO("Desired Current: %.1f mA",msg.current);
				ROS_INFO("j : %d", j);
				
				//cur_eff = getCurrentEfficiency(acceleration[j]/gear_ratio, velocities[j]/gear_ratio, slopeAngle, cur_radius);
			}
			else if(expNum == 2) { //Flat Ground, Large Wheel 
				msg.velocity = scaledRPM_l[j] / gear_ratio; 
				ROS_INFO("Desired RPM: %d rpm", msg.velocity); 
				msg.position = 300000;
				radius = 112.5;
				ROS_INFO("Radius : %.1f mm", radius);
				msg.current = -(loadM[j]*1000/86/12.9 + m3Io) * 1000; 
				ROS_INFO("Desired Current: %.1f mA", msg.current);
				ROS_INFO("j : %d", j); 
			}
			else if(expNum == 3) { //Flat Ground, Transformable Wheel
				float *effNr; 
				effNr = getValue(acceleration[j]/gear_ratio, velocities[j]/gear_ratio, slopeAngle); // returns effd & rd		
				if(fold == 1) 
					msg.position = int((effNr[1] - 0.0563)/0.0563 * 255000 - 45000); 
				
				else 
					msg.position = int((effNr[1] - 0.0563)/0.0563 * 255000 + 45000);
					
				msg.velocity = velocities[j]/effNr[1] *30/3.141592; 
				ROS_INFO("Eff = %.3f", effNr[0]);
				ROS_INFO("Desired RPM: %d rpm", msg.velocity); 
				ROS_INFO("Desired Pos: %d qc(r=%.4f mm)", msg.position, effNr[1]);
				msg.current = -(loadM[j]*1000/86/12.9 + m3Io) * 1000; //mA
				ROS_INFO("Desired Cur: %.1f mA (load=%.3f mNm)",msg.current, loadM[j]);	
				ROS_INFO("j: %d",j);	
				
			}
			else if(expNum = 4) { //Upper Slope, Small Wheel
				msg.velocity = scaledRPM_s[j] / gear_ratio;
				ROS_INFO("Desired RPM: %d rpm", msg.velocity);
				msg.position = 0;
				//radius = (msg.position)/2121728.0*360/28*0.0563 + 0.0563;
				radius = 56.3;				
				ROS_INFO("Radius : %.1f mm\n", radius);
				//msg.current = current[j];
				msg.current = -(loadM[j]*1000/86/12.9 + m3Io) * 1000;		
				ROS_INFO("Desired Current: %.1f mA",msg.current);
				ROS_INFO("j : %d", j);
			}
			else if(expNum = 5) { //Upper Slope, Large Wheel 
				msg.velocity = scaledRPM_l[j] / gear_ratio; 
				ROS_INFO("Desired RPM: %d rpm", msg.velocity); 
				msg.position = 300000;
				radius = 112.5;
				ROS_INFO("Radius : %.1f mm", radius);
				msg.current = -(loadM[j]*1000/86/12.9 + m3Io) * 1000; 
				ROS_INFO("Desired Current: %.1f mA", msg.current);
				ROS_INFO("j : %d", j); 
			}
			else if(expNum = 6) { //Upper Slope, Transforming Wheel
				float *effNr; 
				effNr = getValue(acceleration[j]/gear_ratio, velocities[j]/gear_ratio, slopeAngle); // returns effd & rd		
				if(fold == 1) 
					msg.position = int((effNr[1] - 0.0563)/0.0563 * 255000 - 45000); 
				
				else 
					msg.position = int((effNr[1] - 0.0563)/0.0563 * 255000 + 45000);
					
				msg.velocity = velocities[j]/effNr[1] *30/3.141592; 
				ROS_INFO("Eff = %.3f", effNr[0]);
				ROS_INFO("Desired RPM: %d rpm", msg.velocity); 
				ROS_INFO("Desired Pos: %d qc(r=%.4f mm)", msg.position, effNr[1]);
				msg.current = -(loadM[j]*1000/86/12.9 + m3Io) * 1000; //mA
				ROS_INFO("Desired Cur: %.1f mA (load=%.3f mNm)",msg.current, loadM[j]);	
				ROS_INFO("j: %d",j);	
			}
			else if(expNum = 7) { //Down Slope, Small Wheel
			}
			else if(expNum = 8) { //Down Slope, Large Wheel 
			}
			else if(expNum = 9) { //Down Slope, Transforming Wheel
			}
		//std::cin >> numba.data;		
		//msg.position = numba.data;
        //ROS_INFO("%d", numba.data);
		//std::cin >> numba2.data;		
		//msg.current = numba2.data;
        //ROS_INFO("%f", numba2.data);

   	    command_pub.publish(msg);			
        	
		// msg.data = ss.str();

		
        ros::spinOnce();

        //loop_rate.sleep();
        ++count;
		j++;
		loop_rate.sleep();
		}
    }
	
    return 0;
}
