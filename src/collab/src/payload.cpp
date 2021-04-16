#include "ros/ros.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <collab/des_force.h>
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

float k_lin_load = 1.0;
float k_ang_load = 1.0;

collab::des_force drn1_des_forc;
collab::des_force drn2_des_forc;
collab::des_force drn3_des_forc;
collab::des_force drn4_des_forc;


void callback_pos(const geometry_msgs::Pose& msg){
    paylod_obj_inst.pos[0] = msg.position.x;
    paylod_obj_inst.pos[1] = msg.position.y;
    paylod_obj_inst.pos[2] = msg.position.z;
}


void callback_des_pos(const geometry_msgs::Pose& msg){
    paylod_obj_inst.des_pos[0] = msg.position.x;
    paylod_obj_inst.des_pos[1] = msg.position.y;
    paylod_obj_inst.des_pos[2] = msg.position.z;
}


void callback_vel(const geometry_msgs::Twist& msg){
    paylod_obj_inst.vel[0] = msg.linear.x;
    paylod_obj_inst.vel[1] = msg.linear.y;
    paylod_obj_inst.vel[2] = msg.linear.z;
}


int main(int argc, char **argv){

    ros::init(argc,argv,"payload");
    ros::NodeHandle n;
    ros::Rate loop_rate(100.0);
    ros::Subscriber sub_load_pos = n.subscribe("payload_pos",1,callback_pos); // this is going to be a geometry_msgs/pose
    ros::Subscriber sub_load_vel = n.subscribe("payload_vel",1,callback_vel); // this is going to be a geometry_msgs/twist
    ros::Subscriber sub_load_des_pos = n.subscribe("payload_des_pos",1,callback_des_pos); // this is going to be a geometry_msgs/pose
    ros::Publisher pub_des_u1_mag = n.advertise<collab::des_force>("drn1_desired_forces",1);
    ros::Publisher pub_des_u2_mag = n.advertise<collab::des_force>("drn2_desired_forces",1);
    ros::Publisher pub_des_u3_mag = n.advertise<collab::des_force>("drn3_desired_forces",1);
    ros::Publisher pub_des_u4_mag = n.advertise<collab::des_force>("drn4_desired_forces",1);
    ros::Time time_stamp = ros::Time::now();
    paylod_obj_inst.Init();
    amfc_lin_load.Init(3,amfc_lin_load_gains);
    amfc_ang_load.Init(3,amfc_ang_load_gains);
    pinv_load.Init();
    ros::Time time_stmp = ros::Time::now();

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

	drn1_des_forc.mag = pinv_load.desired_force_val[0].magnitude;
	drn1_des_forc.unit_vec_1 = pinv_load.desired_force_val[0].unit_vector[0];
	drn1_des_forc.unit_vec_2 = pinv_load.desired_force_val[0].unit_vector[1];
	drn1_des_forc.unit_vec_3 = pinv_load.desired_force_val[0].unit_vector[2];

	drn2_des_forc.mag = pinv_load.desired_force_val[1].magnitude;
	drn2_des_forc.unit_vec_1 = pinv_load.desired_force_val[1].unit_vector[0];
	drn2_des_forc.unit_vec_2 = pinv_load.desired_force_val[1].unit_vector[1];
	drn2_des_forc.unit_vec_3 = pinv_load.desired_force_val[1].unit_vector[2];

	drn3_des_forc.mag = pinv_load.desired_force_val[2].magnitude;
	drn3_des_forc.unit_vec_1 = pinv_load.desired_force_val[2].unit_vector[0];
	drn3_des_forc.unit_vec_2 = pinv_load.desired_force_val[2].unit_vector[1];
	drn3_des_forc.unit_vec_3 = pinv_load.desired_force_val[2].unit_vector[2];

	drn4_des_forc.mag = pinv_load.desired_force_val[3].magnitude;
	drn4_des_forc.unit_vec_1 = pinv_load.desired_force_val[3].unit_vector[0];
	drn4_des_forc.unit_vec_2 = pinv_load.desired_force_val[3].unit_vector[1];
	drn4_des_forc.unit_vec_3 = pinv_load.desired_force_val[3].unit_vector[2];

	//(END) Psudo-Inverse module for determining the desired forces on the payload by each link
	//---------------------------------------------------------------------------------------
	//(START) publishing the desired forces
	pub_des_u1_mag.publish(drn1_des_forc);
	pub_des_u2_mag.publish(drn2_des_forc);
	pub_des_u3_mag.publish(drn3_des_forc);
	pub_des_u4_mag.publish(drn4_des_forc);
	//(END) publishing the desired forces
	//---------------------------------------------------------------------------------------
	ros::spinOnce();
	loop_rate.sleep();
    }

}


