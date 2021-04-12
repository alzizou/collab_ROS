#ifndef INVERSE_H
#define INVERSE_H

#endif // INVERSE_H

class determinant{
    public:
	std::vector<std::vector<float>> A;
	std::vector<std::vector<int>> Perms;
	std::vector<int> Perms_sgn;

	float det_val = 0.0;
	int n; //number of rows and columns in A
	int n_fact = 1; //factoril of n

	void Init(std::vector<std::vector<float>>, int);
	void Construct_Perm(void);
	void Construct_det(void);
};

void determinant::Init(std::vector<std::vector<float>> inp_A, int inp_n){
    n = inp_n;
    A.resize(n, std::vector<float>(n));
    for (int i=0;i<n;i++){
	for (int j=0;j<n;j++){
	    A[i][j] = inp_A[i][j];
	}
	n_fact = n_fact * (n+1);
    }
    Perms.resize(n_fact, std::vector<int>(n));
    Perms_sgn.resize(n_fact);

}

void determinant::Construct_Perm(void){
    for (int i=0;i<n;i++){
	Perms[0][i] = i+1;
	Perms_sgn[0] = +1;
    }
    int num = 1;
    while (num<n_fact){
	for (int k=0;k<num;k++){
	    for (int i=0;i<n;i++){
		for (int j=0;j<n;j++){
		    std::vector<int> new_perm = Perms[k];
		    new_perm[i] = Perms[k][j];
		    new_perm[j] = Perms[k][i];
		    for (int u=0;u<num;u++){
			if (new_perm != Perms[u]){
			    num++;
			    Perms.push_back(new_perm);
			    Perms_sgn.push_back(-1*Perms_sgn[k]);
			}
		    }
		}
	    }
	}
    }
}


void determinant::Construct_det(void){
    for (int i=0;i<n_fact;i++){
	float prod = 1.0;
	for (int j=0;j<n;j++){
	    prod = prod * A[j][Perms[i][j]];
	}
	det_val = det_val + (Perms_sgn[i] * prod);
    }
}


class inverse{
    public:
	std::vector<std::vector<float>> A;
	std::vector<std::vector<float>> A_Inv;
	std::vector<std::vector<float>> A_Adj;
	std::vector<std::vector<float>> Cofac;

	determinant A_det;

	int n; //number of rows and columns in A
	int n_fact = 1; //factoril of n

	void Init(std::vector<std::vector<float>>, int);
	void Construct_Adj(void);
	void Construct_Inv(void);

};


void inverse::Init(std::vector<std::vector<float>> inp_A, int inp_n){
    n = inp_n;
    A.resize(n, std::vector<float>(n));
    A_Inv.resize(n, std::vector<float>(n));
    A_Adj.resize(n, std::vector<float>(n));
    Cofac.resize(n, std::vector<float>(n));

    for (int i=0;i<n;i++){
	for (int j=0;j<n;j++){
	    A[i][j] = inp_A[i][j];
	    A_Inv[i][j] = 0.0;
	    A_Adj[i][j] = 0.0;
	    Cofac[i][j] = 0.0;
	}
    }

    A_det.Init(A,n);
    A_det.Construct_Perm();
    A_det.Construct_det();
}

void inverse::Construct_Adj(void){
    for (int i=0;i<n;i++){
	for (int j=0;j<n;j++){
	    std::vector<std::vector<float>> minor_mat = A;
	    minor_mat.erase(minor_mat.begin()+i);
	    for (int k=0;k<(n-1);k++){
		minor_mat[k].erase(minor_mat[k].begin()+j);
	    }
	    determinant det_obj_minor;
	    det_obj_minor.Init(minor_mat,n-1);
	    det_obj_minor.Construct_Perm();
	    det_obj_minor.Construct_det();
	    Cofac[i][j] = pow(-1.0,(i+j)) * det_obj_minor.det_val;
	    A_Adj[j][i] = Cofac[i][j];
	}
    }
}


void inverse::Construct_Inv(void){
    Construct_Adj();
    for (int i=0;i<n;i++){
	for (int j=0;j<n;j++){
	    A_Inv[i][j] = (1.0/A_det.det_val) * A_Adj[i][j];
	}
    }
}
