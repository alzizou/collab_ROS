#ifndef OBSERVER_H
#define OBSERVER_H

#endif // OBSERVER_H

class observer{
    public:
	int Drn_ID;
	std::vector<std::vector<float>> A;
	std::vector<std::vector<float>> A_trn;
	std::vector<std::vector<float>> B;
	std::vector<std::vector<float>> C;
	std::vector<std::vector<float>> C_trn;
	std::vector<std::vector<float>> Q;
	std::vector<std::vector<float>> R;
	std::vector<std::vector<float>> P;
	std::vector<std::vector<float>> Pbar;
	std::vector<std::vector<float>> K;
	std::vector<std::vector<float>> I;
	std::vector<float> x_hat;
	std::vector<float> y;
	std::vector<float> u;
	float q;
	float r;
	float n; //number of system states
	float m; //number of system inputs
	float k; //number of system outputs
	float dt; //time step
	float l_link;
	std::vector<std::vector<float>> rho_load;
	std::vector<slid_diff_var> omg_diffs;

	payload_obj load_obj;
	Rel_Drn_load rel_obj;

	void Init(int, int, int, int, float, float, float, std::vector<std::vector<float>>);
	void Construct_Pbar(void);
	void Construct_K(void);
	void Construct_P(void);
	void Construct_measurements(void);
	void Construct_xhat(void);
	void main(float, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>);
	void Construct_Rotation_Matrix(void);
	void slid_diff_method(int);

};


void observer::Init(int inp_ID, int inp_n, int inp_m, int inp_k, float inp_q, float inp_r,\
                    float inp_l, std::vector<std::vector<float>> inp_rho){
    n = inp_n;
    m = inp_m;
    k = inp_k;
    q = inp_q;
    r = inp_r;
    Drn_ID = inp_ID;
    l_link = inp_l;
    for (int i=0;i<4;i++){
	rho_load[i].assign(inp_rho[i].begin(),inp_rho[i].end);
    }

    A.resize(n,std::vector<float>(n,0.0));
    A_trn.resize(n,std::vector<float>(n,0.0));
    B.resize(n,std::vector<float>(m,0.0));
    C.resize(k,std::vector<float>(n,0.0));
    C_trn.resize(n,std::vector<float>(k,0.0));
    Q.resize(n,std::vector<float>(n,0.0));
    R.resize(m,std::vector<float>(m,0.0));
    P.resize(n,std::vector<float>(n,0.0));
    K.resize(n,std::vector<float>(k,0.0));
    I.resize(n,std::vector<float>(n,0.0));

    x_hat.resize(n,0.0);
    u.resize(m,0.0);
    y.resize(k,0.0);
    omg_diffs.resize(3,0.0);

    for (int i=0;i<n;i++){
	Q[i][i] = q;
	P[i][i] = 1.0;
	I[i][i] = 1.0;
    }

    for (int j=0;j<m;j++){
	R[j][j] = r;
    }

    A[0][0] = 1.0;
    A[1][1] = 1.0;
    A[2][2] = 1.0;

    B[3][0] = 1.0;
    B[4][1] = 1.0;
    B[5][2] = 1.0;

    C[0][0] = 1.0;
    C[1][1] = 1.0;
    C[2][2] = 1.0;

    mat_trn(A, A_trn, n, n);
    mat_trn(C, C_trn, k, n);

    load_obj.Init();
}


void observer::Construct_Pbar(void){

    std::vector<std::vector<float>> Pbar_var1;
    Pbar_var1.resize(n,std::vector<float>(n,0.0));
    std::vector<std::vector<float>> Pbar_var2;
    Pbar_var2.resize(n,std::vector<float>(n,0.0));

    mat_dot(A, P, Pbar_var1, n, n, n);
    mat_dot(Pbar_var1, A_trn, Pbar_var2, n, n, n);
    mat_add(Pbar_var2, Q, Pbar, n, n, +1.0);

}


void observer::Construct_K(void){

    std::vector<std::vector<float>> K_var1;
    K_var1.resize(n,std::vector<float>(k,0.0));
    std::vector<std::vector<float>> K_var2;
    K_var2.resize(k,std::vector<float>(n,0.0));
    std::vector<std::vector<float>> K_var3;
    K_var3.resize(k,std::vector<float>(k,0.0));
    std::vector<std::vector<float>> K_var4;
    K_var4.resize(k,std::vector<float>(k,0.0));

    mat_dot(Pbar, C_trn, K_var1, n, n, n);
    mat_dot(C, Pbar, K_var2, k, n, n);
    mat_dot(K_var2, C_trn, K_var3, n, n, n);
    mat_add(K_var3, R, K_var4, k, k, +1.0);

    inverse K_inverse;
    K_inverse.Init(K_var4,k);
    K_inverse.Construct_Inv();

    mat_dot(K_var1, K_inverse.A_Inv, K, n, k, k);

}

void observer::Construct_P(void){

    std::vector<std::vector<float>> P_var1;
    P_var1.resize(n,std::vector<float>(n,0.0));
    std::vector<std::vector<float>> P_var2;
    P_var2.resize(n,std::vector<float>(n,0.0));

    mat_dot(K, C, P_var1, n, k, n);
    mat_add(I, P_var1, P_var2, n, n, -1.0);
    mat_dot(P_var2, Pbar, P, n, n, n);

}

void observer::Construct_measurements(){

    std::vector<float> y_var1;

    vec_add(rel_obj.rel_pos_local, rho_load[Drn_ID], y_var1, 3, +1.0);
    mat_vec_dot(load_obj.rot_mat, y_var1, y, 3, 3);
    for (int i=0;i<3;i++){
	y[i] = y[i] / l_link;
    }

    //--------------------------------------------------

    std::vector<float> u_var1;
    std::vector<float> u_var2;
    std::vector<float> u_var3;
    std::vector<float> u_var4;

    std::vector<std::vector<float>> omg_skew;
    omg_skew.resize(3,std::vector<float>(3,0.0));
    construct_skew(load_obj.omg, omg_skew);

    for (int j=0;j<3;j++){
	slid_diff_method(j);
    }

    std::vector<std::vector<float>> omg_dot_skew;
    omg_dot_skew.resize(3,std::vector<float>(3,0.0));
    construct_skew(load_obj.omg_dot, omg_dot_skew);

    mat_dot(omg_skew, omg_skew, u_var1, 3, 3, 3);
    mat_add(u_var1, omg_dot_skew, u_var2, 3, 3, +1.0);
    mat_vec_dot(u_var2, rho_load[Drn_ID], u_var3, 3, 3);
    vec_add(rel_obj.rel_acc_local, u_var3, u_var4, 3, +1.0);
    mat_vec_dot(load_obj.rot_mat, u_var4, u, 3, 3);
    for (int i=0;i<3;i++){
	u[i] = u[i] / l_link;
    }

}


void observer::Construct_xhat(void){

    std::vector<float> x_var1;
    x_var1.resize(n,0.0);
    std::vector<float> x_var2;
    x_var2.resize(n,0.0);
    std::vector<float> x_var3;
    x_var3.resize(n,0.0);
    std::vector<float> x_var4;
    x_var4.resize(k,0.0);
    std::vector<float> x_var5;
    x_var5.resize(k,0.0);
    std::vector<float> x_var6;
    x_var6.resize(n,0.0);

    mat_vec_dot(A, x, x_var1, n, n);
    mat_vec_dot(B, u, x_var2, n, m);
    vec_add(x_var1, x_var2, x_var3, n, +1.0);
    mat_vec_dot(C, x_var3, x_var4, k, n);
    vec_add(y, x_var4, x_var5, k, -1.0);
    mat_vec_dot(K, x_var5, x_var6, n, k);
    vec_add(x_var3, x_var5, x_hat, n, +1.0);

}

void observer::main(float inp_dt, std::vector<float> inp_rel_pos_local, std::vector<float> inp_quat_load,\
               std::vector<float> inp_rel_acc_local, std::vector<float> inp_omg_load){

    dt = inp_dt;

    A[0][3] = dt;
    A[1][4] = dt;
    A[2][5] = dt;

    mat_trn(A, A_trn, n, n);

    load_obj.quat.assign(inp_quat_load.begin(),inp_quat_load.end());
    load_obj.omg.assign(inp_omg_load.begin(),inp_omg_load.end());

    rel_obj.rel_pos_local.assign(inp_rel_pos_local.begin(),inp_rel_pos_local.end());
    rel_obj.rel_acc_local.assign(inp_rel_acc_local.begin(),inp_rel_acc_local.end());

    Construct_Pbar();
    Construct_K();
    Construct_P();

    Construct_measurements();
    Construct_xhat();

}

void observer::slid_diff_method(int inp_i){
    float eta = omg_diffs[inp_i].variable1 - load_obj.omg[inp_i];
    omg_diffs[inp_i].diff_val = (-1.0 * gains.k1 * pow(abs_ali(eta),0.5) * sgn_ali(eta) ) + omg_diffs[inp_i].variable2;
    float D_variable1 = omg_diffs[inp_i].diff_val;
    float D_variable2 = -1.0 * gains.k2 * sgn_ali(eta);
    omg_diffs[inp_i].variable1 = omg_diffs[inp_i].variable1 + (D_variable1 * dt);
    omg_diffs[inp_i].variable2 = omg_diffs[inp_i].variable2 + (D_variable2 * dt);
}

