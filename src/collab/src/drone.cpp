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
#include "observer.h"
#include "AMFC.h"


payload_obj paylod_obj_dron;
observer observer_dron;
AMFC amfc_dron;
AMFC_gains amfc_dron_gains;
Rel_Drn_load rel_drn_load_inst;
link_obj link_inst;
desired_force drn_des_forc;

geometry_msgs::Vector3 drn_att_stp;
std_msgs::Float32 drn_thrust_stp;

int drone_ID = 1;
float Q_val = 1.0;
float R_Val = 1.0;
float k_link = 10.0;


void callback_quat(const geometry_msgs::Pose& msg){
    paylod_obj_dron.quat[0] = msg.orientation.w;
    paylod_obj_dron.quat[1] = msg.orientation.x;
    paylod_obj_dron.quat[2] = msg.orientation.y;
    paylod_obj_dron.quat[3] = msg.orientation.z;
}


void callback_des_force(const collab::des_force& msg){
    drn_des_forc.magnitude = msg.mag;
    drn_des_forc.unit_vector[0] = msg.unit_vec_1;
    drn_des_forc.unit_vector[1] = msg.unit_vec_2;
    drn_des_forc.unit_vector[2] = msg.unit_vec_3;
}


int main(int argc, char **argv){
    ros::init(argc,argv,"drone");
    ros::NodeHandle n;
    ros::Rate loop_rate(100.0);
    ros::Subscriber sub_drn_quat = n.subscribe("payload_quat",1,callback_quat); // this is going to be a geometry_msgs/pose
    if (drone_ID == 1){
	ros::Subscriber sub_des_forc = n.subscribe("drn1_desired_forces",1,callback_des_force); // custom message
    }
    if (drone_ID == 2){
	ros::Subscriber sub_des_forc = n.subscribe("drn2_desired_forces",1,callback_des_force); // custom message
    }
    if (drone_ID == 3){
	ros::Subscriber sub_des_forc = n.subscribe("drn3_desired_forces",1,callback_des_force); // custom message
    }
    if (drone_ID == 4){
	ros::Subscriber sub_des_forc = n.subscribe("drn4_desired_forces",1,callback_des_force); // custom message
    }
    ros::Publisher pub_des_att = n.advertise<geometry_msgs::Vector3>("drone_Des_att",1);
    ros::Publisher pub_des_thrust = n.advertise<std_msgs::Float32>("drone_Des_thrust",1);
    ros::Time time_stamp = ros::Time::now();
    paylod_obj_dron.Init();
    observer_dron.Init(drone_ID, 6, 3, 3, Q_val, R_Val, paylod_obj_dron.link_lenght, paylod_obj_dron.rho_payload);
    amfc_dron.Init(3,amfc_dron_gains);
    ros::Time time_stmp = ros::Time::now();

    while (ros::ok()){
	//-------------------------------------------------------------------------------
	//(START) measuring the frequensy of the node
	float period = (ros::Time::now() - time_stmp).toSec();
	time_stmp = ros::Time::now();
	float frequency = 1.0/period;
	std::cout << "frequency of the node:" << " " << frequency << std::endl;
	//(END)
	//-------------------------------------------------------------------------------
	//(START) estimating the unit vector and its rate of change for the link using the observer
	observer_dron.main(period, rel_drn_load_inst.rel_pos_local, paylod_obj_dron.quat,\
	                   rel_drn_load_inst.rel_acc_local, paylod_obj_dron.omg);
	for (int i=0;i<3;i++){
	    link_inst.q[i] = observer_dron.x_hat[i];
	    link_inst.q_dot[i] = observer_dron.x_hat[i+3];
	}
	//(END)
	//-------------------------------------------------------------------------------
	//(START) determining the desired rate of change for the unit vector of the link
	link_inst.des_mag = drn_des_forc.magnitude;
	link_inst.des_q[0] = drn_des_forc.unit_vector[0];
	link_inst.des_q[1] = drn_des_forc.unit_vector[1];
	link_inst.des_q[2] = drn_des_forc.unit_vector[2];
	for (int i=0;i<3;i++){
	    link_inst.des_q_dot[i] = k_link * (link_inst.des_q[i] - link_inst.q[i]);
	}
	//(END)
	//-------------------------------------------------------------------------------
	//(START) AMFC for the rate of change of the unit vector of the link
	amfc_dron.main(link_inst.q_dot, link_inst.des_q_dot, period);
	link_inst.u_parall.assign(amfc_dron.u.begin(),amfc_dron.u.end());
	//(END)
	//-------------------------------------------------------------------------------
	//(START) computing the total demanded forces on the drone in the inertial frame
	for (int i=0;i<3;i++){
	    link_inst.u_perpnd[i] = link_inst.des_mag * link_inst.des_q[i];
	    link_inst.u_total[i] = link_inst.u_parall[i] + link_inst.u_perpnd[i];
	}
	//(END)
	//-------------------------------------------------------------------------------
	//(START) computing the desired thrust force and attitude set-points for the drone
	float mag1 = pow(link_inst.u_total[0],2) + pow(link_inst.u_total[1],2) + pow(link_inst.u_total[2],2);
	float mag2 = pow(mag1,0.5);
	drn_thrust_stp.data = mag2; // set-point for the thrust
	drn_att_stp.x = atan2(link_inst.u_total[1], mag2); // set-point for \phi (pitch)
	drn_att_stp.y = atan2(link_inst.u_total[0], link_inst.u_total[2]); // set-point for \theta (roll)
	drn_att_stp.z = 0.0; //set-point for \psi (yaw)
	//(END)
	//-------------------------------------------------------------------------------
	//(START) publishing the command variables
	pub_des_att.publish(drn_att_stp);
	pub_des_thrust.publish(drn_thrust_stp);
	//(END)
	//-------------------------------------------------------------------------------
	ros::spinOnce();
	loop_rate.sleep();

    }

}


