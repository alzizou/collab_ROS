#ifndef PAYLOAD_OBJ_H
#define PAYLOAD_OBJ_H

#endif // PAYLOAD_OBJ_H

class payload_obj{
    public:
	std::vector<float> Eulers;
	std::vector<float> quat; //quat.w=quat[0] , quat.x=quat[1] , quat.y=quat[2], quat.z=quat[3]
	std::vector<float> pos;
	std::vector<float> vel;
	std::vector<float> acc;
	std::vector<float> omg;
	std::vector<float> omg_dot;
	std::vector<std::vector<float>> rot_mat;
	std::vector<float> des_pos;
	std::vector<float> des_vel;
	std::vector<float> des_Eul;
	std::vector<float> des_omg;
	std::vector<float> des_acc;
	std::vector<float> des_omg_dot;

	float link_lenght = 2.0; //length of each link of the payload (m)
	float dim1_load = 0.5; //dimension-1 of the payload (m)
	float dim2_load = 0.3; //dimension-2 of the payload (m)
	float mass_load = 5.0; //mass of the payload (kg)
	std::vector<std::vector<float>> inertia_load; //moment of inertia of the payload (1/(kg.m2))
	std::vector<std::vector<float>> inertia_inv_load; //inverse matrix of moment of inertia of the payload (1/(kg.m2))
	std::vector<std::vector<float>> rho_payload; //matrix representing the local positions of the joints on the payload

	void Init(void);
	void Construct_Rotation_Matrix(void);
	void Get_quat(std::vector<float>);

};


void payload_obj::Init(void){

    inertia_load.resize(3,std::vector<float>(3,0.0));
    inertia_load[0][0] = 1.25*0.001;
    inertia_load[1][1] = 1.25*0.001;
    inertia_load[2][2] = 2*1.25*0.001;

    inertia_inv_load.resize(3,std::vector<float>(3,0.0));
    inertia_inv_load[0][0] = 1.0 / (1.25*0.001);
    inertia_inv_load[1][1] = 1.0 / (1.25*0.001);
    inertia_inv_load[2][2] = 1.0 / (2*1.25*0.001);

    rho_payload.resize(4,std::vector<float>(3,0.0));
    rho_payload[0][0] = dim1_load;
    rho_payload[1][0] = -dim1_load;
    rho_payload[2][1] = dim2_load;
    rho_payload[3][1] = -dim2_load;
}


void payload_obj::Construct_Rotation_Matrix(void){

    rot_mat[0][0] = 1.0 - 2.0*(pow(quat[2],2)) - 2.0*(pow(quat[3],2));
    rot_mat[0][1] = 2.0*quat[1]*quat[2] - 2.0*quat[3]*quat[2];
    rot_mat[0][2] = 2.0*quat[1]*quat[3] + 2.0*quat[2]*quat[0];

    rot_mat[1][0] = 2.0*quat[1]*quat[2] + 2.0*quat[3]*quat[0];
    rot_mat[1][1] = 1.0 - 2.0*(pow(quat[1],2)) - 2.0*(pow(quat[3],2));
    rot_mat[1][2] = 2.0*quat[2]*quat[3] - 2.0*quat[1]*quat[0];

    rot_mat[2][0] = 2.0*quat[1]*quat[3] - 2.0*quat[2]*quat[0];
    rot_mat[2][1] = 2.0*quat[2]*quat[3] + 2.0*quat[1]*quat[0];
    rot_mat[2][2] = 1.0 - 2.0*(pow(quat[1],2)) - 2.0*(pow(quat[2],2));

}

void payload_obj::Get_quat(std::vector<float> inp_quat){
    quat.assign(inp_quat.begin(),inp_quat.end());
}


