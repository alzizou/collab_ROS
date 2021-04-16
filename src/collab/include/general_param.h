#ifndef GENERAL_PARAM_H
#define GENERAL_PARAM_H

#endif // GENERAL_PARAM_H

float abs_ali(float inp){
    float out = inp;
    if (inp < 0.0){
	out = -1.0 * out;
    }
    return out;
}

float sgn_ali(float inp){
    float out = 0.0;
    if (inp < 0.0){
	out = -1.0;
    }
    if (inp > 0.0){
	out = +1.0;
    }
    return out;
}

void mat_dot(std::vector<std::vector<float>> inp1, std::vector<std::vector<float>> inp2, \
             std::vector<std::vector<float>> &out, int n_inp, int m_inp, int p_inp){
    for (int i=0;i<n_inp;i++){
	for (int j=0;j<p_inp;j++){
	    out[i][j] = 0.0;
	    for (int k=0;k<m_inp;k++){
		out[i][j] = out[i][j] + (inp1[i][k] * inp2[k][j]);
	    }
	}
    }
}

void mat_vec_dot(std::vector<std::vector<float>> inp1, std::vector<float> inp2, \
             std::vector<float> &out, int n_inp, int m_inp){
    for (int i=0;i<n_inp;i++){
	out[i] = 0.0;
	for (int k=0;k<m_inp;k++){
	    out[i] = out[i] + (inp1[i][k] * inp2[k]);
	}
    }
}

void mat_add(std::vector<std::vector<float>> inp1, std::vector<std::vector<float>> inp2, \
             std::vector<std::vector<float>> &out, int n_inp, int m_inp, float sgn_inp){
    for (int i=0;i<n_inp;i++){
	for (int j=0;j<m_inp;j++){
	    out[i][j] = inp1[i][j] + (sgn_inp * inp2[i][j]);
	}
    }
}

void mat_trn(std::vector<std::vector<float>> inp1, std::vector<std::vector<float>> &out, \
             int n_inp, int m_inp){
    for (int i=0;i<n_inp;i++){
	for (int j=0;j<m_inp;j++){
	    out[j][i] = inp1[i][j];
	}
    }
}

void vec_add(std::vector<float> inp1, std::vector<float> inp2, \
             std::vector<float> &out, int n_inp, float sgn_inp){
    for (int i=0;i<n_inp;i++){
	out[i] = inp1[i] + (sgn_inp * inp2[i]);
    }
}

void construct_skew(std::vector<float> inp_vec, std::vector<std::vector<float>> &out_mat){
    out_mat[0][1] = -inp_vec[2];
    out_mat[1][0] = +inp_vec[2];
    out_mat[1][2] = -inp_vec[0];
    out_mat[2][1] = +inp_vec[0];
    out_mat[0][2] = +inp_vec[1];
    out_mat[2][0] = -inp_vec[1];
}

struct Rel_Drn_load{
    std::vector<float> rel_pos_global; //global in the inertial frame
    std::vector<float> rel_pos_local; //local in the drone frame
    std::vector<float> rel_acc_global;
    std::vector<float> rel_acc_local;
};

struct slid_diff_var{
    float diff_val = 0.0;
    float variable1 = 0.0;
    float variable2 = 0.0;
};

struct slid_gains{
    float k1 = 1.0;
    float k2 = 0.1;
};

struct desired_force{
    float magnitude = 0.0;
    std::vector<float> unit_vector = {0.0,0.0,0.0};
};

struct link_obj{
    std::vector<float> q = {0.0,0.0,0.0};
    std::vector<float> q_dot = {0.0,0.0,0.0};

    float des_mag = 0.0;
    std::vector<float> des_q = {0.0,0.0,0.0};
    std::vector<float> des_q_dot = {0.0,0.0,0.0};

    std::vector<float> u_parall = {0.0,0.0,0.0};
    std::vector<float> u_perpnd = {0.0,0.0,0.0};
    std::vector<float> u_total = {0.0,0.0,0.0};
};
