/*************************************************************************\

  Copyright 1995 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.


  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Gottschalk
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:              {gottscha}@cs.unc.edu


\**************************************************************************/



#ifndef MATVEC_H
#define MATVEC_H

#include <math.h>
#include <stdio.h>

#ifdef gnu
#include "zzzz.h"

#ifdef hppa
#define myfabs(x) \
 ({double __value, __arg = (x); \
   asm("fabs,dbl %1, %0": "=f" (__value): "f" (__arg)); \
   __value; \
});
#endif

#ifdef mips
#define myfabs(x) \
 ({double __value, __arg = (x); \
   asm("abs.d %0, %1": "=f" (__value): "f" (__arg)); \
   __value; \
});
#endif

#else  

#define myfabs(x) ((x < 0) ? -x : x)

#endif


inline
void
Mprintg(double M[3][3])
{
  printf("%g %g %g\n%g %g %g\n%g %g %g\n",
	 M[0][0], M[0][1], M[0][2],
	 M[1][0], M[1][1], M[1][2],
	 M[2][0], M[2][1], M[2][2]);
}


inline
void
Mfprint(FILE *f, double M[3][3])
{
  fprintf(f, "%g %g %g\n%g %g %g\n%g %g %g\n",
	 M[0][0], M[0][1], M[0][2],
	 M[1][0], M[1][1], M[1][2],
	 M[2][0], M[2][1], M[2][2]);
}

inline
void
Mprint(double M[3][3])
{
  printf("%g %g %g\n%g %g %g\n%g %g %g\n",
	 M[0][0], M[0][1], M[0][2],
	 M[1][0], M[1][1], M[1][2],
	 M[2][0], M[2][1], M[2][2]);
}

inline
void
Vprintg(double V[3])
{
  printf("%g %g %g\n", V[0], V[1], V[2]);
}

inline
void
Vfprint(FILE *f, double V[3])
{
  fprintf(f, "%g %g %g\n", V[0], V[1], V[2]);
}

inline
void
Vprint(double V[3])
{
  printf("%g %g %g\n", V[0], V[1], V[2]);
}

inline
void
Midentity(double M[3][3])
{
  M[0][0] = M[1][1] = M[2][2] = 1.0;
  M[0][1] = M[1][2] = M[2][0] = 0.0;
  M[0][2] = M[1][0] = M[2][1] = 0.0;
}

inline
void
McM(double Mr[3][3], double M[3][3])
{
  Mr[0][0] = M[0][0];  Mr[0][1] = M[0][1];  Mr[0][2] = M[0][2];
  Mr[1][0] = M[1][0];  Mr[1][1] = M[1][1];  Mr[1][2] = M[1][2];
  Mr[2][0] = M[2][0];  Mr[2][1] = M[2][1];  Mr[2][2] = M[2][2];
}

inline
void
VcV(double Vr[3], double V[3])
{
  Vr[0] = V[0];  Vr[1] = V[1];  Vr[2] = V[2];
}

inline
void
McolcV(double Vr[3], double M[3][3], int c)
{
  Vr[0] = M[0][c];
  Vr[1] = M[1][c];
  Vr[2] = M[2][c];
}

inline
void
McolcMcol(double Mr[3][3], int cr, double M[3][3], int c)
{
  Mr[0][cr] = M[0][c];
  Mr[1][cr] = M[1][c];
  Mr[2][cr] = M[2][c];
}

inline
void
MxMpV(double Mr[3][3], double M1[3][3], double M2[3][3], double T[3])
{
  Mr[0][0] = (M1[0][0] * M2[0][0] +
	      M1[0][1] * M2[1][0] +
	      M1[0][2] * M2[2][0] +
	      T[0]);
  Mr[1][0] = (M1[1][0] * M2[0][0] +
	      M1[1][1] * M2[1][0] +
	      M1[1][2] * M2[2][0] +
	      T[1]);
  Mr[2][0] = (M1[2][0] * M2[0][0] +
	      M1[2][1] * M2[1][0] +
	      M1[2][2] * M2[2][0] +
	      T[2]);
  Mr[0][1] = (M1[0][0] * M2[0][1] +
	      M1[0][1] * M2[1][1] +
	      M1[0][2] * M2[2][1] +
	      T[0]);
  Mr[1][1] = (M1[1][0] * M2[0][1] +
	      M1[1][1] * M2[1][1] +
 	      M1[1][2] * M2[2][1] +
	      T[1]);
  Mr[2][1] = (M1[2][0] * M2[0][1] +
	      M1[2][1] * M2[1][1] +
	      M1[2][2] * M2[2][1] +
	      T[2]);
  Mr[0][2] = (M1[0][0] * M2[0][2] +
	      M1[0][1] * M2[1][2] +
	      M1[0][2] * M2[2][2] +
	      T[0]);
  Mr[1][2] = (M1[1][0] * M2[0][2] +
	      M1[1][1] * M2[1][2] +
	      M1[1][2] * M2[2][2] +
	      T[1]);
  Mr[2][2] = (M1[2][0] * M2[0][2] +
	      M1[2][1] * M2[1][2] +
	      M1[2][2] * M2[2][2] +
	      T[2]);
}

inline
void
MxM(double Mr[3][3], double M1[3][3], double M2[3][3])
{
  Mr[0][0] = (M1[0][0] * M2[0][0] +
	      M1[0][1] * M2[1][0] +
	      M1[0][2] * M2[2][0]);
  Mr[1][0] = (M1[1][0] * M2[0][0] +
	      M1[1][1] * M2[1][0] +
	      M1[1][2] * M2[2][0]);
  Mr[2][0] = (M1[2][0] * M2[0][0] +
	      M1[2][1] * M2[1][0] +
	      M1[2][2] * M2[2][0]);
  Mr[0][1] = (M1[0][0] * M2[0][1] +
	      M1[0][1] * M2[1][1] +
	      M1[0][2] * M2[2][1]);
  Mr[1][1] = (M1[1][0] * M2[0][1] +
	      M1[1][1] * M2[1][1] +
 	      M1[1][2] * M2[2][1]);
  Mr[2][1] = (M1[2][0] * M2[0][1] +
	      M1[2][1] * M2[1][1] +
	      M1[2][2] * M2[2][1]);
  Mr[0][2] = (M1[0][0] * M2[0][2] +
	      M1[0][1] * M2[1][2] +
	      M1[0][2] * M2[2][2]);
  Mr[1][2] = (M1[1][0] * M2[0][2] +
	      M1[1][1] * M2[1][2] +
	      M1[1][2] * M2[2][2]);
  Mr[2][2] = (M1[2][0] * M2[0][2] +
	      M1[2][1] * M2[1][2] +
	      M1[2][2] * M2[2][2]);
}


inline
void
MxMT(double Mr[3][3], double M1[3][3], double M2[3][3])
{
  Mr[0][0] = (M1[0][0] * M2[0][0] +
	      M1[0][1] * M2[0][1] +
	      M1[0][2] * M2[0][2]);
  Mr[1][0] = (M1[1][0] * M2[0][0] +
	      M1[1][1] * M2[0][1] +
	      M1[1][2] * M2[0][2]);
  Mr[2][0] = (M1[2][0] * M2[0][0] +
	      M1[2][1] * M2[0][1] +
	      M1[2][2] * M2[0][2]);
  Mr[0][1] = (M1[0][0] * M2[1][0] +
	      M1[0][1] * M2[1][1] +
	      M1[0][2] * M2[1][2]);
  Mr[1][1] = (M1[1][0] * M2[1][0] +
	      M1[1][1] * M2[1][1] +
 	      M1[1][2] * M2[1][2]);
  Mr[2][1] = (M1[2][0] * M2[1][0] +
	      M1[2][1] * M2[1][1] +
	      M1[2][2] * M2[1][2]);
  Mr[0][2] = (M1[0][0] * M2[2][0] +
	      M1[0][1] * M2[2][1] +
	      M1[0][2] * M2[2][2]);
  Mr[1][2] = (M1[1][0] * M2[2][0] +
	      M1[1][1] * M2[2][1] +
	      M1[1][2] * M2[2][2]);
  Mr[2][2] = (M1[2][0] * M2[2][0] +
	      M1[2][1] * M2[2][1] +
	      M1[2][2] * M2[2][2]);
}

inline
void
MTxM(double Mr[3][3], double M1[3][3], double M2[3][3])
{
  Mr[0][0] = (M1[0][0] * M2[0][0] +
	      M1[1][0] * M2[1][0] +
	      M1[2][0] * M2[2][0]);
  Mr[1][0] = (M1[0][1] * M2[0][0] +
	      M1[1][1] * M2[1][0] +
	      M1[2][1] * M2[2][0]);
  Mr[2][0] = (M1[0][2] * M2[0][0] +
	      M1[1][2] * M2[1][0] +
	      M1[2][2] * M2[2][0]);
  Mr[0][1] = (M1[0][0] * M2[0][1] +
	      M1[1][0] * M2[1][1] +
	      M1[2][0] * M2[2][1]);
  Mr[1][1] = (M1[0][1] * M2[0][1] +
	      M1[1][1] * M2[1][1] +
 	      M1[2][1] * M2[2][1]);
  Mr[2][1] = (M1[0][2] * M2[0][1] +
	      M1[1][2] * M2[1][1] +
	      M1[2][2] * M2[2][1]);
  Mr[0][2] = (M1[0][0] * M2[0][2] +
	      M1[1][0] * M2[1][2] +
	      M1[2][0] * M2[2][2]);
  Mr[1][2] = (M1[0][1] * M2[0][2] +
	      M1[1][1] * M2[1][2] +
	      M1[2][1] * M2[2][2]);
  Mr[2][2] = (M1[0][2] * M2[0][2] +
	      M1[1][2] * M2[1][2] +
	      M1[2][2] * M2[2][2]);
}

inline
void
MxV(double Vr[3], double M1[3][3], double V1[3])
{
  Vr[0] = (M1[0][0] * V1[0] +
	   M1[0][1] * V1[1] + 
	   M1[0][2] * V1[2]);
  Vr[1] = (M1[1][0] * V1[0] +
	   M1[1][1] * V1[1] + 
	   M1[1][2] * V1[2]);
  Vr[2] = (M1[2][0] * V1[0] +
	   M1[2][1] * V1[1] + 
	   M1[2][2] * V1[2]);
}


inline
void
MxVpV(double Vr[3], double M1[3][3], double V1[3], double V2[3])
{
  Vr[0] = (M1[0][0] * V1[0] +
	   M1[0][1] * V1[1] + 
	   M1[0][2] * V1[2] + 
	   V2[0]);
  Vr[1] = (M1[1][0] * V1[0] +
	   M1[1][1] * V1[1] + 
	   M1[1][2] * V1[2] + 
	   V2[1]);
  Vr[2] = (M1[2][0] * V1[0] +
	   M1[2][1] * V1[1] + 
	   M1[2][2] * V1[2] + 
	   V2[2]);
}

inline
void
sMxVpV(double Vr[3], double s1, double M1[3][3], double V1[3], double V2[3])
{
  Vr[0] = s1 * (M1[0][0] * V1[0] +
		M1[0][1] * V1[1] + 
		M1[0][2] * V1[2]) +
		V2[0];
  Vr[1] = s1 * (M1[1][0] * V1[0] +
		M1[1][1] * V1[1] + 
		M1[1][2] * V1[2]) + 
		V2[1];
  Vr[2] = s1 * (M1[2][0] * V1[0] +
		M1[2][1] * V1[1] + 
		M1[2][2] * V1[2]) + 
		V2[2];
}

inline
void
MTxV(double Vr[3], double M1[3][3], double V1[3])
{
  Vr[0] = (M1[0][0] * V1[0] +
	   M1[1][0] * V1[1] + 
	   M1[2][0] * V1[2]); 
  Vr[1] = (M1[0][1] * V1[0] +
	   M1[1][1] * V1[1] + 
	   M1[2][1] * V1[2]);
  Vr[2] = (M1[0][2] * V1[0] +
	   M1[1][2] * V1[1] + 
	   M1[2][2] * V1[2]); 
}

inline
void
sMTxV(double Vr[3], double s1, double M1[3][3], double V1[3])
{
  Vr[0] = s1*(M1[0][0] * V1[0] +
	      M1[1][0] * V1[1] + 
	      M1[2][0] * V1[2]); 
  Vr[1] = s1*(M1[0][1] * V1[0] +
	      M1[1][1] * V1[1] + 
	      M1[2][1] * V1[2]);
  Vr[2] = s1*(M1[0][2] * V1[0] +
	      M1[1][2] * V1[1] + 
	      M1[2][2] * V1[2]); 
}


inline
void
VmV(double Vr[3], const double V1[3], const double V2[3])
{
  Vr[0] = V1[0] - V2[0];
  Vr[1] = V1[1] - V2[1];
  Vr[2] = V1[2] - V2[2];
}

inline
void
VpV(double Vr[3], double V1[3], double V2[3])
{
  Vr[0] = V1[0] + V2[0];
  Vr[1] = V1[1] + V2[1];
  Vr[2] = V1[2] + V2[2];
}

inline
void
VpVxS(double Vr[3], double V1[3], double V2[3], double s)
{
  Vr[0] = V1[0] + V2[0] * s;
  Vr[1] = V1[1] + V2[1] * s;
  Vr[2] = V1[2] + V2[2] * s;
}

inline 
void
MskewV(double M[3][3], const double v[3])
{
  M[0][0] = M[1][1] = M[2][2] = 0.0;
  M[1][0] = v[2];
  M[0][1] = -v[2];
  M[0][2] = v[1];
  M[2][0] = -v[1];
  M[1][2] = -v[0];
  M[2][1] = v[0];
}


inline
void
VcrossV(double Vr[3], const double V1[3], const double V2[3])
{
  Vr[0] = V1[1]*V2[2] - V1[2]*V2[1];
  Vr[1] = V1[2]*V2[0] - V1[0]*V2[2];
  Vr[2] = V1[0]*V2[1] - V1[1]*V2[0];
}


inline
double
Vlength(double V[3])
{
  return sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);
}

inline
void
Vnormalize(double V[3])
{
  double d = 1.0 / sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);
  V[0] *= d;
  V[1] *= d;
  V[2] *= d;
}


inline
double
VdotV(double V1[3], double V2[3])
{
  return (V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2]);
}

inline
void
VxS(double Vr[3], double V[3], double s)
{
  Vr[0] = V[0] * s;
  Vr[1] = V[1] * s;
  Vr[2] = V[2] * s;
}

inline
void
Mqinverse(double Mr[3][3], double m[3][3])
{
  int i,j;

  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      {
	int i1 = (i+1)%3;
	int i2 = (i+2)%3;
	int j1 = (j+1)%3;
	int j2 = (j+2)%3;
	Mr[i][j] = (m[j1][i1]*m[j2][i2] - m[j1][i2]*m[j2][i1]);
      }
}


#define rfabs(x) ((x < 0) ? -x : x)

#define ROT(a,i,j,k,l) g=a[i][j]; h=a[k][l]; a[i][j]=g-s*(h+g*tau); a[k][l]=h+s*(g-h*tau);

int
inline
Meigen(double vout[3][3], double dout[3], double a[3][3])
{
  int i;
  double tresh,theta,tau,t,sm,s,h,g,c;
  int nrot;
  double b[3];
  double z[3];
  double v[3][3];
  double d[3];
  
  v[0][0] = v[1][1] = v[2][2] = 1.0;
  v[0][1] = v[1][2] = v[2][0] = 0.0;
  v[0][2] = v[1][0] = v[2][1] = 0.0;
  
  b[0] = a[0][0]; d[0] = a[0][0]; z[0] = 0.0;
  b[1] = a[1][1]; d[1] = a[1][1]; z[1] = 0.0;
  b[2] = a[2][2]; d[2] = a[2][2]; z[2] = 0.0;

  nrot = 0;
  
  for(i=0; i<50; i++)
    {

      sm=0.0; sm+=fabs(a[0][1]); sm+=fabs(a[0][2]); sm+=fabs(a[1][2]);
      if (sm == 0.0) { McM(vout,v); VcV(dout,d); return i; }
      
      if (i < 3) tresh=0.2*sm/(3*3); else tresh=0.0;
      
      {
	g = 100.0*rfabs(a[0][1]);  
	if (i>3 && rfabs(d[0])+g==rfabs(d[0]) && rfabs(d[1])+g==rfabs(d[1]))
	  a[0][1]=0.0;
	else if (rfabs(a[0][1])>tresh)
	  {
	    h = d[1]-d[0];
	    if (rfabs(h)+g == rfabs(h)) t=(a[0][1])/h;
	    else
	      {
		theta=0.5*h/(a[0][1]);
		t=1.0/(rfabs(theta)+sqrt(1.0+theta*theta));
		if (theta < 0.0) t = -t;
	      }
	    c=1.0/sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a[0][1];
	    z[0] -= h; z[1] += h; d[0] -= h; d[1] += h;
	    a[0][1]=0.0;
	    ROT(a,0,2,1,2); ROT(v,0,0,0,1); ROT(v,1,0,1,1); ROT(v,2,0,2,1); 
	    nrot++;
	  }
      }

      {
	g = 100.0*rfabs(a[0][2]);
	if (i>3 && rfabs(d[0])+g==rfabs(d[0]) && rfabs(d[2])+g==rfabs(d[2]))
	  a[0][2]=0.0;
	else if (rfabs(a[0][2])>tresh)
	  {
	    h = d[2]-d[0];
	    if (rfabs(h)+g == rfabs(h)) t=(a[0][2])/h;
	    else
	      {
		theta=0.5*h/(a[0][2]);
		t=1.0/(rfabs(theta)+sqrt(1.0+theta*theta));
		if (theta < 0.0) t = -t;
	      }
	    c=1.0/sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a[0][2];
	    z[0] -= h; z[2] += h; d[0] -= h; d[2] += h;
	    a[0][2]=0.0;
	    ROT(a,0,1,1,2); ROT(v,0,0,0,2); ROT(v,1,0,1,2); ROT(v,2,0,2,2); 
	    nrot++;
	  }
      }


      {
	g = 100.0*rfabs(a[1][2]);
	if (i>3 && rfabs(d[1])+g==rfabs(d[1]) && rfabs(d[2])+g==rfabs(d[2]))
	  a[1][2]=0.0;
	else if (rfabs(a[1][2])>tresh)
	  {
	    h = d[2]-d[1];
	    if (rfabs(h)+g == rfabs(h)) t=(a[1][2])/h;
	    else
	      {
		theta=0.5*h/(a[1][2]);
		t=1.0/(rfabs(theta)+sqrt(1.0+theta*theta));
		if (theta < 0.0) t = -t;
	      }
	    c=1.0/sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a[1][2];
	    z[1] -= h; z[2] += h; d[1] -= h; d[2] += h;
	    a[1][2]=0.0;
	    ROT(a,0,1,0,2); ROT(v,0,1,0,2); ROT(v,1,1,1,2); ROT(v,2,1,2,2); 
	    nrot++;
	  }
      }

      b[0] += z[0]; d[0] = b[0]; z[0] = 0.0;
      b[1] += z[1]; d[1] = b[1]; z[1] = 0.0;
      b[2] += z[2]; d[2] = b[2]; z[2] = 0.0;
      
    }

  fprintf(stderr, "eigen: too many iterations in Jacobi transform (%d).\n", i);

  return i;
}


#endif
