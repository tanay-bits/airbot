/* This file is part of the AirBot firmware */

#include <math.h>
#include <stdlib.h>

// Gaussian elimination to solve for x in ax=b; n is the length of vector b.
// Taken from http://www.crbond.com/nonlinear.htm 
int gelim(float **a,float *b,float *x, int n)
{
	float tmp,pvt,*t;
	int i,j,k;

	for (i=0;i<n;i++) {             // outer loop on rows
		pvt = a[i][i];              // get pivot value
		if (!pvt) {
			for (j=i+1;j<n;j++) {
				if((pvt = a[j][i]) != 0.0) break;
			}
			if (!pvt) return 1;     // nowhere to run!
			t=a[j];                 // swap matrix rows...
			a[j]=a[i];
			a[i]=t;
			tmp=b[j];               // ...and result vector
			b[j]=b[i];
			b[i]=tmp;        
		}
		// (virtual) Gaussian elimination of column
		for (k=i+1;k<n;k++) {       // alt: for (k=n-1;k>i;k--)
			tmp = a[k][i]/pvt;
			for (j=i+1;j<n;j++) {   // alt: for (j=n-1;j>i;j--)
				a[k][j] -= tmp*a[i][j];
			}
			b[k] -= tmp*b[i];
		}
	}
	// Do back substitution
	for (i=n-1;i>=0;i--) {
		x[i]=b[i];
		for (j=n-1;j>i;j--) {
			x[i] -= a[i][j]*x[j];
		}
		x[i] /= a[i][i];
	}
	return 0;
}

// vector function representing the set of nonlinear equations to solve, of the form f(x)=0
void f(float* x, float* eqs) {
	eqs[0] = pow(x[0],2) + pow(x[1],2) - 2*x[0]*x[1]*cAB - pow(AB, 2);
	eqs[1] = pow(x[1],2) + pow(x[2],2) - 2*x[1]*x[2]*cBC - pow(BC, 2);
	eqs[2] = pow(x[0],2) + pow(x[2],2) - 2*x[0]*x[2]*cAC - pow(AC, 2);
}

// Jacobian of the above function
void jac(float* x, float **jacobian) {
	jacobian[0][0] = 2*x[0] - 2*x[1]*cAB;
	jacobian[0][1] = 2*x[1] - 2*x[0]*cAB;
	jacobian[0][2] = 0;
	
	jacobian[1][0] = 0;
	jacobian[1][1] = 2*x[1] - 2*x[2]*cBC;
	jacobian[1][2] = 2*x[2] - 2*x[1]*cBC;
	
	jacobian[2][0] = 2*x[0] - 2*x[2]*cAC;
	jacobian[2][1] = 0;
	jacobian[2][2] = 2*x[2] - 2*x[0]*cAC;
}

// Newton-Raphson root finding routine
void newtonOpt(float* x, int* maxiter, float* eqs, float **jacobian) {
	float xdel[3];
	float tmp;

	int k;
	for (k = 0; k < *maxiter; k++)
	{
		f(x, eqs);						// update function values (eqs) for current value of 'x'
		jac(x, jacobian);				// update Jacobian matrix
		gelim(jacobian, eqs, xdel, 3);	// store solution of J*xdel=fval (like Ax=b)

		// update x and check for convergence
		tmp = 0.0;
		int i;
		for (i = 0; i < 3; i++)
		{
			tmp += fabs(xdel[i]);
			x[i] = x[i] - xdel[i];
		}
		if (tmp < 1e-3) break;
	}
	*maxiter = k;	// store the number of iterations it took
}