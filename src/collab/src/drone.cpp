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
#include "observer.h"
#include "AMFC.h"


payload_obj paylod_obj_dron;
observer observer_dron;
AMFC amfc_dron;
AMFC_gains amfc_dron_gains;
Rel_Drn_load rel_drn_load_inst;
link_obj link_inst;

std::vector<float> drn_att_stp = {0.0,0.0,0.0};
float drn_thrust_stp = 0.0;


int main(int argc, char **argv){
    ros::init(argc,argv,"drone");
    ros::NodeHandle n;
    ros::Rate loop_rate(100.0);
    ros::Subscriber sub_drn_quat = n.subscriber("drone_quat",callback_quat);
    ros::Subscriber sub_des_forc = n.subscriber("drone_des_force",callback_des_force);
    ros::Publisher pub_des_att = n.advertise<std::vector<float>>("drone_Des_att",1);
    ros::Publisher pub_des_thrust = n.advertise<float>("drone_Des_thrust",1);
    ros::Time time_stamp = ros::Time::now();
    paylod_obj_dron.Init();
    observer_dron.Init(drone_ID, 6, 3, 3, Q_val, R_Val, paylod_obj_dron.link_lenght, paylod_obj_dron.rho_payload);
    amfc_dron.Init(3,amfc_dron_gains);

    while (ros::ok()){
	//-------------------------------------------------------------------------------
	//(START) measuring the frequensy of the node
	float period = (ros::Time::now() - time_stamp).toSec();
	time_stamp = ros::Time::now();
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
	for (int i=0;i<3;i++){
	    link_inst.des_q_dot = k_link * (link_inst.des_q[i] - link_inst.q[i]);
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
	drn_thrust_stp = pow(mag1,0.5); // set-point for the thrust
	drn_att_stp[0] = atan2(link_inst.u_total[1], drn_thrust_stp); // set-point for \phi (pitch)
	drn_att_stp[1] = atan2(link_inst.u_total[0], link_inst.u_total[2]); // set-point for \theta (roll)
	drn_att_stp[2] = 0.0; //set-point for \psi (yaw)
	//(END)
	//-------------------------------------------------------------------------------
	//(START) publishing the command variables
	pub_des_att.publish(drn_att_stp);
	pub_des_thrust.publish(drn_thrust_stp);
	//(END)
	//-------------------------------------------------------------------------------
	ros::spinOnce();
	loop_rate.Sleep();

    }

}


