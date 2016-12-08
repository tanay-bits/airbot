#include <math.h>
#include <stdio.h>

const double AB = 1;
const double BC = 1;
const double AC = 2;
double cAB, cBC, cAC;

int gelimd(double **a,double *b,double *x, int n)
{
    double tmp,pvt,*t;
    int i,j,k,itmp;

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

void f()
void nlnewt(void (*f)(double *x,double *fv,int n),double *x,
    double *fv,double **jac,double *p,int n,double eps)
{
    double tmp,delta;
    int i,j;

    
    f(x,fv,n);                  // get residuals for current value of 'x'

// Compute Jacobian matrix
    for (i=0;i<n;i++) {
        tmp = x[i];
        delta = (tmp > 1.0) ? eps*tmp : eps;
        x[i] = tmp+delta;       // bump this element
        delta = x[i] - tmp;     // try this to reduce error (from Schnabel)
        f(x,p,n);
        x[i] = tmp;             // restore original value
        for (j=0;j<n;j++) {
            jac[j][i] = (p[j]-fv[j])/delta;
        }
    }
// Update residuals
    for (i=0;i<n;i++) {
        tmp = 0.0;
        for (j=0;j<n;j++) {
            tmp += jac[i][j]*x[j];
        }
        p[i] = tmp - fv[i];
    }
// Update solution vector
    gelimd(jac,p,x,n);
}


// void f(double* x, double* eqs) {
// 	eqs[0] = pow(x[0],2) + pow(x[1],2) - 2*x[0]*x[1]*cAB - pow(AB, 2);
// 	eqs[1] = pow(x[1],2) + pow(x[2],2) - 2*x[1]*x[2]*cBC - pow(BC, 2);
// 	eqs[2] = pow(x[0],2) + pow(x[2],2) - 2*x[0]*x[2]*cAC - pow(AC, 2);
// }

// void jac(double* x, double **jacobian) {
// 	jacobian[0][0] = 2*x[0] - 2*x[1]*cAB;
// 	jacobian[0][1] = 2*x[1] - 2*x[0]*cAB;
// 	jacobian[0][2] = 0;
	
// 	jacobian[1][0] = 0;
// 	jacobian[1][1] = 2*x[1] - 2*x[2]*cBC;
// 	jacobian[1][2] = 2*x[2] - 2*x[1]*cBC;
	
// 	jacobian[2][0] = 2*x[0] - 2*x[2]*cAC;
// 	jacobian[2][1] = 0;
// 	jacobian[2][2] = 2*x[2] - 2*x[0]*cAC;
// }

// int gelim(double **a, double *b, double *x, int n)
// {
//     double tmp, pvt, *t;
//     int i, j, k, itmp;

//     for (i = 0; i < n; i++) {             // outer loop on rows
//         pvt = a[i][i];              // get pivot value
//         if (!pvt) {
//             for (j = i+1; j < n; j++) {
//                 if((pvt = a[j][i]) != 0.0) break;
//             }
//             if (!pvt) return 1;     // nowhere to run!
//             t = a[j];                 // swap matrix rows...
//             a[j] = a[i];
//             a[i] = t;
//             tmp = b[j];               // ...and result vector
//             b[j] = b[i];
//             b[i] = tmp;        
//         }
// 		// (virtual) Gaussian elimination of column
//         for (k = i+1; k < n; k++) {       // alt: for (k=n-1;k>i;k--)
//             tmp = a[k][i]/pvt;
//             for (j = i+1; j<n; j++) {   // alt: for (j=n-1;j>i;j--)
//                 a[k][j] -= tmp*a[i][j];
//             }
//             b[k] -= tmp*b[i];
//         }
// 	}
// 	// Do back substitution
// 	for (i = n-1; i >= 0; i--) {
// 		x[i] = b[i];
// 		for (j = n-1; j > i; j--) {
// 			x[i] -= a[i][j]*x[j];
// 		}
// 		x[i] /= a[i][i];
// 	}
//     return 0;
// }

// void newtonOpt(double* x, int* maxiter, double* eqs, double **jacobian) {
// 	double xdel[3];
// 	double tmp;

// 	int k;
// 	for (k = 0; k < *maxiter; k++)
// 	{
// 		f(x, eqs);						// update function values (eqs) for current value of 'x'
// 		jac(x, jacobian);				// update Jacobian matrix
// 		gelim(jacobian, eqs, xdel, 3);	// store solution of J*xdel=fval (like Ax=b)

// 		// update x and check for convergence
// 		tmp = 0.0;
// 		int i;
// 		for (i = 0; i < 3; i++)
// 		{
// 			tmp += fabs(xdel[i]);
// 			x[i] = x[i] - xdel[i];
// 		}
// 		if (tmp < 1e-4) break;
// 	}
// 	*maxiter = k;	// store the number of iterations it took
// }


int main() {
	
	float h1 = 101.0;
	float v1 = 65.0;

	float h2 = 102.0;
	float v2 = 66.0;

	float h3 = 103.0;
	float v3 = 67.0;

	double eqsVec[3];
	double **jacMat;
	jacMat[0][0] = 0; 
	jacMat[0][1] = 0; 
	jacMat[0][2] = 0; 
	jacMat[1][0] = 0; 
	jacMat[1][1] = 0; 
	jacMat[1][2] = 0; 
	jacMat[2][0] = 0; 
	jacMat[2][1] = 0; 
	jacMat[2][2] = 0; 
	
	cAB = sin(v1)*cos(h1)*sin(v2)*cos(h2) + sin(v1)*sin(h1)*sin(v2)*sin(h2) + cos(v1)*cos(v2);
	cBC = sin(v2)*cos(h2)*sin(v3)*cos(h3) + sin(v2)*sin(h2)*sin(v3)*sin(h3) + cos(v2)*cos(v3);
	cAC = sin(v1)*cos(h1)*sin(v3)*cos(h3) + sin(v1)*sin(h1)*sin(v3)*sin(h3) + cos(v1)*cos(v3);

	double x0[3] = {50, 51, 52};
	int* iter;
	*iter = 5;

	newtonOpt(x0, iter, eqsVec, jacMat);

	printf("%f\n", x0[0]);

	return 0;
}