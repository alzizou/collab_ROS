#include "ros/ros.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "payload_obj.h"
#include "general_param.h"
#include "inverse.h"
#include "pinv.h"
#include "AMFC.h"

payload_obj paylod_obj_inst;
AMFC amfc_lin_load;
AMFC amfc_ang_load;
AMFC_gains amfc_lin_load_gains;
AMFC_gains amfc_ang_load_gains;
pinv pinv_load;

int main(int argc, char **argv){

    ros::init(argc,argv,"payload");
    ros::NodeHandle n;
    ros::Rate loop_rate(100.0);
    ros::Subscriber sub_load_pos = n.subscribe("payload_pos",callback_pos);
    ros::Subscriber sub_load_vel = n.subscribe("payload_vel",callback_vel);
    ros::Subscriber sub_load_des_pos = n.subscribe("payload_des_pos",callback_des_pos);
    ros::Publisher pub_des_u_mag = n.advertise<std::vector<desired_force>("desired_forces",1);
    ros::Time time_stamp = ros::Time::now();
    paylod_obj_inst.Init();
    amfc_lin_load.Init(3,amfc_lin_load_gains);
    amfc_ang_load.Init(3,amfc_ang_load_gains);
    pinv.Init();

    while (ros::ok()){
	//---------------------------------------------------------------------------------------
	//(START) measuring the frequency of the node
	float period = (ros::Time::now() - time_stmp).toSec();
	time_stamp = ros::Time::now();
	float frequency = 1.0/period;
	std::cout<< "frequency of payload node:" << " " << frequency << std::endl;
	//(END) measuring the frequency of the node
	//---------------------------------------------------------------------------------------
	//(START) determining the desired velocities of the payload
	for (int i=0;i<3;i++){
	    paylod_obj_inst.des_vel[i] = k_lin_load * (paylod_obj_inst.des_pos[i] - paylod_obj_inst.pos[i]);
	    paylod_obj_inst.des_omg[i] = k_ang_load * (paylod_obj_inst.des_Eul[i] - paylod_obj_inst.Eulers[i]);
	}
	//(END) determining the desired velocities of the payload
	//---------------------------------------------------------------------------------------
	//(START) AMFCs for linear and angular velocities of the payload
	amfc_lin_load.main(paylod_obj_inst.vel, paylod_obj_inst.des_vel, period);
	paylod_obj_inst.des_acc.assign(amfc_lin_load.u.begin(),amfc_lin_load.u.end());

	amfc_ang_load.main(paylod_obj_inst.omg, paylod_obj_inst.des_omg, period);
	paylod_obj_inst.des_omg_dot.assign(amfc_ang_load.u.begin(),amfc_ang_load.u.end());
	//(END) AMFCs for linear and angular velocities of the payload
	//---------------------------------------------------------------------------------------
	//(START) Psudo-Inverse module for determining the desired forces on the payload by each link
	pinv_load.main(paylod_obj_inst);
	//(END) Psudo-Inverse module for determining the desired forces on the payload by each link
	//---------------------------------------------------------------------------------------
	//(START) publishing the desired forces
	pub_des_u_mag.publish(pinv_load.desired_force_val);
	//(END) publishing the desired forces
	//---------------------------------------------------------------------------------------
	ros::spinOnce();
	loop_rate.sleep();
    }

}


