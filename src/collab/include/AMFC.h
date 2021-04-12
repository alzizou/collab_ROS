#ifndef AMFC_H
#define AMFC_H

#endif // AMFC_H

struct AMFC_gains{
    float gamma1 = 1.0e2;
    float gamma2 = 1.0e0;
    float rho1 = 1.0;
    float rho2 = 1.0;
    float q = 1.0;
    float k1 = 1.0;
    float k2 = 0.1;
};

struct DRE_var{
    float int_delta = 0.0;
    float W = 0.0;
};

class AMFC{
    public:
	float dt;
	int num_states;
	std::vector<float> A_hat;
	std::vector<float> P;
	std::vector<float> g_hat;
	std::vector<float> A_hat_dot;
	std::vector<float> P_dot;
	std::vector<float> g_hat_dot;
	std::vector<float> e;
	std::vector<float> int_e;
	std::vector<float> sigma;
	std::vector<float> x;
	std::vector<float> des_x;
	std::vector<float> des_x_dot;
	std::vector<float> u;
	AMFC_gains gains;
	std::vector<slid_diff_var> des_diffs;
	std::vector<DRE_var> dre_vars;

	void Init(int, AMFC_gains);
	void main(std::vector<float>, std::vector<float>, float);
	void slid_diff_method(int);
	void DRE_sol(int);
};


void AMFC::Init(int inp_num_states, AMFC_gains inp_AMFC_gains){

    num_states = inp_num_states;
    A_hat.resize(num_states);
    A_hat_dot.resize(num_states);
    g_hat.resize(num_states);
    g_hat_dot.resize(num_states);
    P.resize(num_states);
    P_dot.resize(num_states);
    e.resize(num_states);
    int_e.resize(num_states);
    sigma.resize(num_states);
    x.resize(num_states);
    des_x.resize(num_states);
    des_x_dot.resize(num_states);
    u.resize(num_states);
    des_diffs.resize(num_states);
    dre_vars.resize(num_states);

    for (int i=0;i<num_states;i++){
	A_hat[i] = 0.0;
	A_hat_dot[i] = 0.0;
	g_hat[i] = 0.0;
	g_hat_dot[i] = 0.0;
	P[i] = 1.0;
	P_dot[i] = 0.0;
	e[i] = 0.0;
	int_e[i] = 0.0;
	sigma[i] = 0.0;
	x[i] = 0.0;
	des_x[i] = 0.0;
	des_x_dot[i] = 0.0;
	u[i] = 0.0;
    }

    gains.gamma1 = inp_AMFC_gains.gamma1;
    gains.gamma2 = inp_AMFC_gains.gamma2;
    gains.rho1 = inp_AMFC_gains.rho1;
    gains.rho2 = inp_AMFC_gains.rho2;
    gains.q = inp_AMFC_gains.q;
    gains.k1 = inp_AMFC_gains.k1;
    gains.k2 = inp_AMFC_gains.k2;

}


void AMFC::main(std::vector<float> inp_x, std::vector<float> inp_des_x, float inp_dt){
    dt = inp_dt;
    for (int i=0;i<num_states;i++){
	des_x[i] = inp_des_x[i];
	x[i] = inp_x[i];
	e[i] = des_x[i] - x[i];
	slid_diff_method(i);
	des_x_dot[i] = des_diffs[i].diff_val;
	int_e[i] = int_e[i] + (e[i] * dt);
	sigma[i] = int_e[i] + e[i];
	u[i] = des_x_dot[i] - (A_hat[i] * x[i]) - g_hat[i] - int_e[i] + \
	        ((1.0 + (2.0*gains.q/P[i]) + A_hat[i])*sigma[i]) - ( 0.25 * P[i] * sigma[i]);
	g_hat_dot[i] = -(gains.gamma1 * P[i] * sigma[i]) - (gains.rho1 * gains.gamma1 * g_hat[i]);
	A_hat_dot[i] = -(gains.gamma2 * P[i] * sigma[i] * (x[i] - sigma[i])) - (gains.rho2 * gains.gamma2 * A_hat[i]);
	DRE_sol(i);
	g_hat[i] = g_hat[i] + (g_hat_dot[i] * dt);
	A_hat[i] = A_hat[i] + (A_hat_dot[i] * dt);
	P[i] = P[i] + (P_dot[i] * dt);
    }
}


void AMFC::slid_diff_method(int inp_i){
    float eta = des_diffs[inp_i].variable1 - des_x[inp_i];
    des_diffs[inp_i].diff_val = (-1.0 * gains.k1 * pow(abs_ali(eta),0.5) * sgn_ali(eta) ) + des_diffs[inp_i].variable2;
    float D_variable1 = des_diffs[inp_i].diff_val;
    float D_variable2 = -1.0 * gains.k2 * sgn_ali(eta);
    des_diffs[inp_i].variable1 = des_diffs[inp_i].variable1 + (D_variable1 * dt);
    des_diffs[inp_i].variable2 = des_diffs[inp_i].variable2 + (D_variable2 * dt);
}


void AMFC::DRE_sol(int inp_i){
    float delta = (A_hat[inp_i] * A_hat[inp_i]) + (2.0*gains.q);
    float P0 = A_hat[inp_i] - pow(delta,0.5);
    float W0 = (1.0 / (1.0 - P0));
    dre_vars[inp_i].int_delta = dre_vars[inp_i].int_delta + (pow(delta,0.5) * dt);
    dre_vars[inp_i].W = (1.0/(2.0*pow(delta,0.5))) + ( (W0 - (1.0/(2.0*pow(delta,0.5)))) *  exp(-2.0*dre_vars[inp_i].int_delta) );
    P_dot[inp_i] = P0 + (1.0/dre_vars[inp_i].W);
}

