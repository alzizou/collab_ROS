#ifndef PINV_H
#define PINV_H

#endif // PINV_H

class pinv{
    public:
	std::vector<std::vector<float>> B_ext;
	std::vector<float> Z_ext;
	std::vector<float> des_acc; // desired linear and angular acceleration of the payload
	std::vector<float> mue;

	std::vector<desired_force> desired_force_val;

	int n = 10; //number of measured variables
	int m = 12; //number of estimated variables

	float des_rx;
	float des_ry;

	payload_obj load_obj;

	void Init(void);
	void Construct_B(void);
	void Construct_Sub_B(int);
	void Construct_Z(void);
	void Construct_mu(void);
	void main(void);
	void updating_load(payload_obj);
	void determining_desired_forces(void);

};


void pinv::Init(void){

    B_ext.resize(n,std::vector<float>(m,0.0));
    Z_ext.resize(n,0.0);
    mu.resize(m,0.0);
    des_acc.resize(6,0.0);

    load_obj.Init();

}

void pinv::Construct_B(void){

    for (int i=0;i<3;i++){
	for (int j=0;j<m;j++){
	    B_ext[i][i+(3*j)] = 1.0 / load_obj.mass_load;
	}
    }

    for (int i=0;i<4;i++){
	Construct_Sub_B(i);
    }

    //-----------------------------------------------
    // adding the extension elements

    std::vector<float> bx;
    bx.resize(3,0.0);
    bx[0] = 1.0;
    std::vector<float> by;
    by.resize(3,0.0);
    by[1] = 0.0;

    std::vector<std::vector<float>> rot_mat_trn;
    rot_mat_trn.resize(3,std::vector<float>(3,0.0));
    mat_trn(load_obj.rot_mat, rot_mat_trn, 3, 3);

    mat_vec_dot(rot_mat_trn, bx, bx, 3, 3);
    mat_vec_dot(rot_mat_trn, by, by, 3, 3);

    for (int i=0;i<3;i++){
	B_ext[6][i] = bx[i];
	B_ext[6][i+3] = bx[i];
	B_ext[7][i+6] = by[i];
	B_ext[7][i+9] = by[i];
	B_ext[8][i] = bx[i];
	B_ext[9][i+6] = by[i];
    }

}

void pinv::Construct_Sub_B(int inp_i){

    std::vector<std::vector<float>> rho_skew;
    rho_skew.resize(3,std::vector<float>(3,0.0));
    construct_skew(load_obj.rho_payload[inp_i], rho_skew);

    std::vector<std::vector<float>> rot_mat_trn;
    rot_mat_trn.resize(3,std::vector<float>(3,0.0));
    mat_trn(load_obj.rot_mat, rot_mat_trn, 3, 3);

    std::vector<std::vector<float>> sub_var1;
    sub_var1.resize(3,std::vector<float>(3,0.0));
    std::vector<std::vector<float>> sub_var2;
    sub_var2.resize(3,std::vector<float>(3,0.0));

    mat_dot(load_obj.inertia_inv_load, rho_skew, sub_var1, 3, 3, 3);
    mat_dot(sub_var1, rot_mat_trn, sub_var2, 3, 3, 3);

    for (int i=0;i<3;i++){
	for (int j=0;j<3;j++){
	    B_ext[3+i][(3*inp_i)+j] = sub_var2[i][j];
	}
    }

}


void pinv::Construct_Z(void){

    for (int i=0;i<6;i++){
	Z_ext[i] = des_acc[i];
    }
    Z_ext[2] = Z_ext[2] - 9.81;

    std::vector<std::vector<float>> omg_skew;
    omg_skew.resize(3,std::vector<float>(3,0.0));
    construct_skew(load_obj.omg, omg_skew);

    std::vector<std::vector<float>> Z_var1;
    Z_var1.resize(3,std::vector<float>(3,0.0));
    std::vector<float> Z_var2;
    Z_var2.resize(3,0.0);
    std::vector<float> Z_var3;
    Z_var3.resize(3,0.0);

    mat_dot(omg_skew, load_obj.inertia_load, Z_var1, 3, 3, 3);
    mat_vec_dot(Z_var1, load_obj.omg, Z_var2, 3, 3);
    mat_vec_dot(load_obj.inertia_inv_load, Z_var2, Z_var3, 3, 3);

    for (int i=3;i<6;i++){
	Z_ext[i] = Z_ext[i] + Z_var3[i-3];
    }

    //----------------------------------
    // adding the extention elements

    std::vector<std::vector<float>> rot_mat_trn;
    rot_mat_trn.resize(3,std::vector<float>(3,0.0));
    mat_trn(load_obj.rot_mat, rot_mat_trn, 3, 3);

    std::vector<float> Z0;
    Z0.resize(3,0.0);
    std::vector<float> Z0_var1;
    Z0_var1.resize(3,0.0);
    for (int i=0;i<3;i++){
	Z0_var1[i] = Z_ext[i];
    }
    mat_vec_dot(rot_mat_trn, Z0_var1, Z0, 3, 3);
    for (int i=0;i<3;i++){
	Z0[i] = load_obj.mass_load * Z_ext[i];
    }

    float h1 = (des_rx - (2.0*load_obj.dim1_load)) / (2.0*load_obj.link_lenght);
    float h2 = (des_ry - (2.0*load_obj.dim2_load)) / (2.0*load_obj.link_lenght);

    float k1 = (-1.0) * pow( ( ((pow((Z0(2)/2.0),2.0)) + (pow((Z0(3)/4.0),2.0))) / ((1.0/(pow(h1,2))) - 1) ),0.5 );
    float k2 = (-1.0) * pow( ( ((pow((Z0(1)/2.0),2.0)) + (pow((Z0(3)/4.0),2.0))) / ((1.0/(pow(h2,2))) - 1) ),0.5 );

    Z_ext[6] = 0.0;
    Z_ext[7] = 0.0;

    Z_ext[8] = k1;
    Z_ext[9] = k2;

}


void pinv::Construct_mu(void){

    std::vector<std::vector<float>> mu_var1;
    mu_var1.resize(n,std::vector<float>(n,0.0));
    std::vector<std::vector<float>> mu_var2;
    mu_var2.resize(n,std::vector<float>(n,0.0));
    std::vector<std::vector<float>> mu_var3;
    mu_var3.resize(m,std::vector<float>(n,0.0));

    std::vector<std::vector<float>> B_ext_trn;
    B_ext_trn.resize(m,std::vector<float>(n,0.0));
    mat_trn(B_ext, B_ext_trn, n, m);

    mat_dot(B_ext, B_Ext_trn, mu_var1, n, m, n);

    inverse B_inverse;
    B_inverse.Init(mu_var1,m);
    B_inverse.Construct_Inv();

    mat_dot(B_Ext_trn, B_inverse.A_Inv, mu_var3, m, n, n);
    mat_vec_dot(mu_var3, Z_ext, mu, m, n);

}

void pinv::main(payload_obj inp_load){

    updating_load(inp_load);

    Construct_B();
    Construct_Z();
    Construct_mu();

    determining_desired_forces();

}


void pinv::updating_load(payload_obj inp_load){

    for (int i=0;i<3;i++){
	for (int j=0;j<3;j++){
	    load_obj.rot_mat[i][j] = inp_load.rot_mat[i][j];
	}
	load_obj.omg[i] = inp_load.omg[i];
    }

}


void pinv::determining_desired_forces(void){

    for (int i=0;i<4;i++){
	float mag1 = pow(mu[3*i+0],2) + pow(mu[3*i+1],2) + pow(mu[3*i+2],2);
	float mag = pow(mag1,0.5);
	desired_force_val[i].magnitude = mag;
	desired_force_val[i].unit_vector[0] = -1.0 * mu[3*i+0] / mag;
	desired_force_val[i].unit_vector[1] = -1.0 * mu[3*i+1] / mag;
	desired_force_val[i].unit_vector[2] = -1.0 * mu[3*i+2] / mag;
    }

}
