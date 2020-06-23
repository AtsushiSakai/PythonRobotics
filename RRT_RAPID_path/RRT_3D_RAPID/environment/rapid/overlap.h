#ifndef OVERLAP_H
#define OVERLAP_H

int 
tri_contact (double *P1, double *P2, double *P3,
	     double *Q1, double *Q2, double *Q3);

int
obb_disjoint(double B[3][3], double T[3], double a[3], double b[3]);

#endif
