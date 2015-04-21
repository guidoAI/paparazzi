#ifndef _NR_UTILS_H_
#define _NR_UTILS_H_
/*
static float sqrarg;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)
static float dsqrarg;
#define DSQR(a) ((dsqrarg=(a)) == 0.0 ? 0.0 : dsqrarg*dsqrarg)
static float dmaxarg1,dmaxarg2;
#define DMAX(a,b) (dmaxarg1=(a),dmaxarg2=(b),(dmaxarg1) > (dmaxarg2) ?\
        (dmaxarg1) : (dmaxarg2))
static float dminarg1,dminarg2;
#define DMIN(a,b) (dminarg1=(a),dminarg2=(b),(dminarg1) < (dminarg2) ?\
        (dminarg1) : (dminarg2))
static float maxarg1,maxarg2;
#define FMAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ?\
        (maxarg1) : (maxarg2))
static float minarg1,minarg2;
#define FMIN(a,b) (minarg1=(a),minarg2=(b),(minarg1) < (minarg2) ?\
        (minarg1) : (minarg2))
static int lmaxarg1,lmaxarg2;
#define LMAX(a,b) (lmaxarg1=(a),lmaxarg2=(b),(lmaxarg1) > (lmaxarg2) ?\
        (lmaxarg1) : (lmaxarg2))
static int lminarg1,lminarg2;
#define LMIN(a,b) (lminarg1=(a),lminarg2=(b),(lminarg1) < (lminarg2) ?\
        (lminarg1) : (lminarg2))
static int imaxarg1,imaxarg2;
#define IMAX(a,b) (imaxarg1=(a),imaxarg2=(b),(imaxarg1) > (imaxarg2) ?\
        (imaxarg1) : (imaxarg2))
static int iminarg1,iminarg2;
#define IMIN(a,b) (iminarg1=(a),iminarg2=(b),(iminarg1) < (iminarg2) ?\
        (iminarg1) : (iminarg2))
*/
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

#if defined(__STDC__) || defined(ANSI) || defined(NRANSI) /* ANSI */

void nrerror(char error_text[]);
float *vector(int nl, int nh);
int *ivector(int nl, int nh);
unsigned char *cvector(int nl, int nh);
unsigned int *lvector(int nl, int nh);
float *dvector(int nl, int nh);
float **matrix(int nrl, int nrh, int ncl, int nch);
float **dmatrix(int nrl, int nrh, int ncl, int nch);
int **imatrix(int nrl, int nrh, int ncl, int nch);
float **submatrix(float **a, int oldrl, int oldrh, int oldcl, int oldch,
	int newrl, int newcl);
float **convert_matrix(float *a, int nrl, int nrh, int ncl, int nch);
float ***f3tensor(int nrl, int nrh, int ncl, int nch, int ndl, int ndh);
void free_vector(float *v, int nl, int nh);
void free_ivector(int *v, int nl, int nh);
void free_cvector(unsigned char *v, int nl, int nh);
void free_lvector(unsigned int *v, int nl, int nh);
void free_dvector(float *v, int nl, int nh);
void free_matrix(float **m, int nrl, int nrh, int ncl, int nch);
void free_dmatrix(float **m, int nrl, int nrh, int ncl, int nch);
void free_imatrix(int **m, int nrl, int nrh, int ncl, int nch);
void free_submatrix(float **b, int nrl, int nrh, int ncl, int nch);
void free_convert_matrix(float **b, int nrl, int nrh, int ncl, int nch);
void free_f3tensor(float ***t, int nrl, int nrh, int ncl, int nch,
	int ndl, int ndh);

#else /* ANSI */
/* traditional - K&R */

void nrerror();
float *vector();
float **matrix();
float **submatrix();
float **convert_matrix();
float ***f3tensor();
float *dvector();
float **dmatrix();
int *ivector();
int **imatrix();
unsigned char *cvector();
unsigned int *lvector();
void free_vector();
void free_dvector();
void free_ivector();
void free_cvector();
void free_lvector();
void free_matrix();
void free_submatrix();
void free_convert_matrix();
void free_dmatrix();
void free_imatrix();
void free_f3tensor();

#endif /* ANSI */

#endif /* _NR_UTILS_H_ */
