#include <math.h>
#include <stdlib.h>
#include <stdio.h>

const float AB = 2;
const float BC = 2;
const float AC = 4;
float cAB, cBC, cAC;

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

void f(float* x, float* eqs) {
	eqs[0] = pow(x[0],2) + pow(x[1],2) - 2*x[0]*x[1]*cAB - pow(AB, 2);
	eqs[1] = pow(x[1],2) + pow(x[2],2) - 2*x[1]*x[2]*cBC - pow(BC, 2);
	eqs[2] = pow(x[0],2) + pow(x[2],2) - 2*x[0]*x[2]*cAC - pow(AC, 2);
}

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


int main() {	
	float h1 = 101.0 * M_PI/180.0;
	float v1 = 65.0 * M_PI/180.0;

	float h2 = 102.0 * M_PI/180.0;
	float v2 = 66.0 * M_PI/180.0;

	float h3 = 103.0 * M_PI/180.0;
	float v3 = 67.0 * M_PI/180.0;

	float eqsVec[3];

	// float **jacMat = (float **)malloc(3*(sizeof(float *)));
	// jacMat[0] = (float *)malloc(sizeof(float) * 3 * 3);
	// int i;
	// for(i=0;i<3;i++)
	// 	jacMat[i] = *jacMat + 3 * i;

	float **jacMat=(float**)malloc(3*(sizeof(float*)));
	int i;
	for(i=0;i<3;i++)
		*(jacMat+i)=(float*)malloc(sizeof(float)*3);

	cAB = sin(v1)*cos(h1)*sin(v2)*cos(h2) + sin(v1)*sin(h1)*sin(v2)*sin(h2) + cos(v1)*cos(v2);
	cBC = sin(v2)*cos(h2)*sin(v3)*cos(h3) + sin(v2)*sin(h2)*sin(v3)*sin(h3) + cos(v2)*cos(v3);
	cAC = sin(v1)*cos(h1)*sin(v3)*cos(h3) + sin(v1)*sin(h1)*sin(v3)*sin(h3) + cos(v1)*cos(v3);

	float x0[3] = {50, 51, 52};
	int* iter;
	int ival = 50000;
	iter = &ival;

	newtonOpt(x0, iter, eqsVec, jacMat);

	float Xa = x0[0] * sin(v1) * cos(h1);
	float Ya = x0[0] * sin(v1) * sin(h1);
	float Za = x0[0] * cos(v1);

	float Xb = x0[1] * sin(v2) * cos(h2);
	float Yb = x0[1] * sin(v2) * sin(h2);
	float Zb = x0[1] * cos(v2);

	float Xc = x0[2] * sin(v3) * cos(h3);
	float Yc = x0[2] * sin(v3) * sin(h3);
	float Zc = x0[2] * cos(v3);

	printf("Ranges: %f %f %f\n", x0[0], x0[1], x0[2]);
	printf("F vals: %f %f %f\n", eqsVec[0], eqsVec[1], eqsVec[2]);
	printf("Iterations: %d\n\n", *iter);
	printf("A: %f %f %f\n", Xa, Ya, Za);
	printf("B: %f %f %f\n", Xb, Yb, Zb);
	printf("C: %f %f %f\n", Xc, Yc, Zc);

	return 0;
}