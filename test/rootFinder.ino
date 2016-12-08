#include <math.h>

const double AB = 1;
const double BC = 1;
const double AC = 2;

h1 = bufs[0].get_h_angle()
v1 = bufs[0].get_v_angle()

h2 = bufs[1].get_h_angle()
v2 = bufs[1].get_v_angle()

h3 = bufs[2].get_h_angle()
v3 = bufs[2].get_v_angle()

double cAB, cBC, cAC;
cAB = sin(v1)*cos(h1)*sin(v2)*cos(h2) + sin(v1)*sin(h1)*sin(v2)*sin(h2) + cos(v1)*cos(v2);
cBC = sin(v2)*cos(h2)*sin(v3)*cos(h3) + sin(v2)*sin(h2)*sin(v3)*sin(h3) + cos(v2)*cos(v3);
cAC = sin(v1)*cos(h1)*sin(v3)*cos(h3) + sin(v1)*sin(h1)*sin(v3)*sin(h3) + cos(v1)*cos(v3);


// double x[3];
double x0[] = {50, 51, 52};

// double eqs[3];
// eqs[0] = pow(x[0],2) + pow(x[1],2) - 2*x[0]*x[1]*cAB - pow(AB, 2);
// eqs[1] = pow(x[1],2) + pow(x[2],2) - 2*x[1]*x[2]*cBC - pow(BC, 2);
// eqs[2] = pow(x[0],2) + pow(x[2],2) - 2*x[0]*x[2]*cAC - pow(AC, 2);

double* f(double* x) {
	double eqs[3];
	eqs[0] = pow(x[0],2) + pow(x[1],2) - 2*x[0]*x[1]*cAB - pow(AB, 2);
	eqs[1] = pow(x[1],2) + pow(x[2],2) - 2*x[1]*x[2]*cBC - pow(BC, 2);
	eqs[2] = pow(x[0],2) + pow(x[2],2) - 2*x[0]*x[2]*cAC - pow(AC, 2);
	return eqs;
}

double** jac(double* x) {
	double jacobian[3][3];
	
	jacobian[0][0] = 2*x[0] - 2*x[1]*cAB;
	jacobian[0][1] = 2*x[1] - 2*x[0]*cAB;
	jacobian[0][2] = 0;
	
	jacobian[1][0] = 0;
	jacobian[1][1] = 2*x[1] - 2*x[2]*cBC;
	jacobian[1][2] = 2*x[2] - 2*x[1]*cBC;
	
	jacobian[2][0] = 2*x[0] - 2*x[2]*cAC;
	jacobian[2][1] = 0;
	jacobian[2][2] = 2*x[2] - 2*x[0]*cAC;

	return jacobian;
}

int gelim(double **a, double *b, double *x, int n)
{
    double tmp,  , *t;
    int i, j, k, itmp;

    for (i = 0; i < n; i++) {             // outer loop on rows
        pvt = a[i][i];              // get pivot value
        if (!pvt) {
            for (j = i+1; j < n; j++) {
                if((pvt = a[j][i]) != 0.0) break;
            }
            if (!pvt) return 1;     // nowhere to run!
            t = a[j];                 // swap matrix rows...
            a[j] = a[i];
            a[i] = t;
            tmp = b[j];               // ...and result vector
            b[j] = b[i];
            b[i] = tmp;        
        }
		// (virtual) Gaussian elimination of column
        for (k = i+1; k < n; k++) {       // alt: for (k=n-1;k>i;k--)
            tmp = a[k][i]/pvt;
            for (j = i+1; j<n; j++) {   // alt: for (j=n-1;j>i;j--)
                a[k][j] -= tmp*a[i][j];
            }
            b[k] -= tmp*b[i];
        }
	}
	// Do back substitution
	for (i = n-1; i >= 0; i--) {
		x[i] = b[i];
		for (j = n-1; j > i; j--) {
			x[i] -= a[i][j]*x[j];
		}
		x[i] /= a[i][i];
	}
    return 0;
}

void newtonOpt(double* x, uint16_t* maxiter, double eps) {
	// double xval[3];
	double fval[3];
	double jval[3][3];
	double xdel[3];
	double tmp;

	// xval[0] = x[0];
	// xval[1] = x[1];
	// xval[2] = x[2];

	uint16_t k;
	for (k = 0; k < maxiter; k++)
	{
		fval = f(x);					// get function value (vector) for current value of 'x'
		jval = jac(x);					// get Jacobian matrix
		gelim(jval, fval, xdel, 3);		// store solution of J*xdel=fval (like Ax=b)

		// update x and check for convergence
		tmp = 0.0;
		for (uint8_t i = 0; i < 3; i++)
		{
			tmp += fabs(xdel[i]);
			x[i] = x[i] - xdel[i];
		}
		if (tmp < 1e-4) break;
	}
	*maxiter = k;	// store the number of iterations it took
}