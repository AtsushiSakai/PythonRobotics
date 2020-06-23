#ifndef MOMENTS_H
#define MOMENTS_H

#include "matvec.h"
#include "obb.h"

struct moment
{
  double A;  
  double m[3];
  double s[3][3];
};

struct accum
{
  double A;
  double m[3];
  double s[3][3];
};

inline
void
print_moment(moment &M)
{
  fprintf(stderr, "A: %g, m: %g %g %g, s: %g %g %g %g %g %g\n",
	  M.A,
	  M.m[0], M.m[1], M.m[2],
	  M.s[0][0], M.s[0][1], M.s[0][2], M.s[1][1], M.s[1][2], M.s[2][2]);
}


inline
void
clear_accum(accum &a)
{
  a.m[0] = a.m[1] = a.m[2] = 0.0;
  a.s[0][0] = a.s[0][1] = a.s[0][2] = 0.0;
  a.s[1][0] = a.s[1][1] = a.s[1][2] = 0.0;
  a.s[2][0] = a.s[2][1] = a.s[2][2] = 0.0;
  a.A = 0.0;
}

inline
void
accum_moment(accum &a, moment &b)
{
  a.m[0] += b.m[0] * b.A;
  a.m[1] += b.m[1] * b.A;
  a.m[2] += b.m[2] * b.A;
  
  a.s[0][0] += b.s[0][0];
  a.s[0][1] += b.s[0][1];
  a.s[0][2] += b.s[0][2];
  a.s[1][0] += b.s[1][0];
  a.s[1][1] += b.s[1][1];
  a.s[1][2] += b.s[1][2];
  a.s[2][0] += b.s[2][0];
  a.s[2][1] += b.s[2][1];
  a.s[2][2] += b.s[2][2];

  a.A += b.A;
}

inline
void
mean_from_moment(double M[3], moment &m)
{
  M[0] = m.m[0];
  M[1] = m.m[1];
  M[2] = m.m[2];
}

inline
void
mean_from_accum(double M[3], accum &a)
{
  M[0] = a.m[0] / a.A;
  M[1] = a.m[1] / a.A;
  M[2] = a.m[2] / a.A;
}

inline
void
covariance_from_accum(double C[3][3], accum &a)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      C[i][j] = a.s[i][j] - a.m[i]*a.m[j]/a.A;
}



inline
void
compute_moment(moment &M, double p[3], double q[3], double r[3])
{
  double u[3], v[3], w[3];

  // compute the area of the triangle
  VmV(u, q, p);
  VmV(v, r, p);
  VcrossV(w, u, v);
  M.A = 0.5 * Vlength(w);

  if (M.A == 0.0)
    {
      // This triangle has zero area.  The second order components
      // would be eliminated with the usual formula, so, for the 
      // sake of robustness we use an alternative form.  These are the 
      // centroid and second-order components of the triangle's vertices.

      // centroid
      M.m[0] = (p[0] + q[0] + r[0]) /3;
      M.m[1] = (p[1] + q[1] + r[1]) /3;
      M.m[2] = (p[2] + q[2] + r[2]) /3;

      // second-order components
      M.s[0][0] = (p[0]*p[0] + q[0]*q[0] + r[0]*r[0]);
      M.s[0][1] = (p[0]*p[1] + q[0]*q[1] + r[0]*r[1]);
      M.s[0][2] = (p[0]*p[2] + q[0]*q[2] + r[0]*r[2]);
      M.s[1][1] = (p[1]*p[1] + q[1]*q[1] + r[1]*r[1]);
      M.s[1][2] = (p[1]*p[2] + q[1]*q[2] + r[1]*r[2]);
      M.s[2][2] = (p[2]*p[2] + q[2]*q[2] + r[2]*r[2]);      
      M.s[2][1] = M.s[1][2];
      M.s[1][0] = M.s[0][1];
      M.s[2][0] = M.s[0][2];

      return;
    }

  // get the centroid
  M.m[0] = (p[0] + q[0] + r[0])/3;
  M.m[1] = (p[1] + q[1] + r[1])/3;
  M.m[2] = (p[2] + q[2] + r[2])/3;

  // get the second order components -- note the weighting by the area
  M.s[0][0] = M.A*(9*M.m[0]*M.m[0]+p[0]*p[0]+q[0]*q[0]+r[0]*r[0])/12;
  M.s[0][1] = M.A*(9*M.m[0]*M.m[1]+p[0]*p[1]+q[0]*q[1]+r[0]*r[1])/12;
  M.s[1][1] = M.A*(9*M.m[1]*M.m[1]+p[1]*p[1]+q[1]*q[1]+r[1]*r[1])/12;
  M.s[0][2] = M.A*(9*M.m[0]*M.m[2]+p[0]*p[2]+q[0]*q[2]+r[0]*r[2])/12;
  M.s[1][2] = M.A*(9*M.m[1]*M.m[2]+p[1]*p[2]+q[1]*q[2]+r[1]*r[2])/12;
  M.s[2][2] = M.A*(9*M.m[2]*M.m[2]+p[2]*p[2]+q[2]*q[2]+r[2]*r[2])/12;
  M.s[2][1] = M.s[1][2];
  M.s[1][0] = M.s[0][1];
  M.s[2][0] = M.s[0][2];
}

inline
void
compute_moments(moment *M, tri *tris, int num_tris)
{
  int i;

  // first collect all the moments, and obtain the area of the 
  // smallest nonzero area triangle.

  double Amin = 0.0;
  int zero = 0;
  int nonzero = 0;
  for(i=0; i<num_tris; i++)
    {
      compute_moment(M[i], 
		     tris[i].p1,
		     tris[i].p2, 
		     tris[i].p3);  
      if (M[i].A == 0.0)
	{
	  zero = 1;
	}
      else
	{
	  nonzero = 1;
	  if (Amin == 0.0) Amin = M[i].A;
	  else if (M[i].A < Amin) Amin = M[i].A;
	}
    }

  if (zero)
    {
      fprintf(stderr, "----\n");
      fprintf(stderr, "Warning!  Some triangles have zero area!\n");
      fprintf(stderr, "----\n");

      // if there are any zero area triangles, go back and set their area
      
      // if ALL the triangles have zero area, then set the area thingy
      // to some arbitrary value.
      if (Amin == 0.0) Amin = 1.0;

      for(i=0; i<num_tris; i++)
	{
	  if (M[i].A == 0.0) M[i].A = Amin;
	}
      
    }
}

#endif
