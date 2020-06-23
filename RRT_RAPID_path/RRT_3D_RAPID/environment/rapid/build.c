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


#include "rapid_version.h"

static char rapidtag_data[] = "RAPIDTAG  file: "__FILE__"    date: "__DATE__"    time: "__TIME__;

// to silence the compiler's complaints about unreferenced identifiers.
static void r1(char *f){  r1(f);  r1(rapidtag_data);  r1(rapid_version);}


#include "rapid.h"
#include "matvec.h"
#include "moments.h"
#include "obb.h"

extern int RAPID_initialized;
void RAPID_initialize();

int
RAPID_model::BeginModel()
{
  int bs = build_state;
  
  if (!RAPID_initialized) RAPID_initialize();
  
  // free whatever storage we had.  Remember, it's okay to delete null
  // pointers in C++, so we don't have to check them first.
  delete [] b;   b = 0;
  num_boxes_alloced = 0;
  delete [] tris;   tris = 0;
  num_tris = 0;
  num_tris_alloced = 0;

  build_state = RAPID_BUILD_STATE_BEGIN;
  
  if (bs == RAPID_BUILD_STATE_CONST) return RAPID_OK;

  if (bs == RAPID_BUILD_STATE_PROCESSED) return RAPID_OK;

  if (bs == RAPID_BUILD_STATE_BEGIN) 
    return RAPID_ERR_BUILD_OUT_OF_SEQUENCE;
  
  if (bs == RAPID_BUILD_STATE_ADDTRI) 
    return RAPID_ERR_BUILD_OUT_OF_SEQUENCE;

  return RAPID_OK;
}

int
RAPID_model::EndModel()
{
  if (!RAPID_initialized) RAPID_initialize();

  if (num_tris == 0)
    {
      return RAPID_ERR_BUILD_EMPTY_MODEL;
    }

  int myrc = build_hierarchy();
  if (myrc == RAPID_OK)
    {
      // only change to processed state if successful.
      build_state = RAPID_BUILD_STATE_PROCESSED;
      return RAPID_OK;
    }
  else return myrc;

}

int
RAPID_model::AddTri(const double *p1, const double *p2, const double *p3, int id)
{
  if (!RAPID_initialized) RAPID_initialize();

  int myrc = RAPID_OK; // we'll return this unless a problem is found

  if (build_state == RAPID_BUILD_STATE_PROCESSED)
    {
      // client forgot to call BeginModel() before calling AddTri().
      return RAPID_ERR_BUILD_OUT_OF_SEQUENCE;
    }
  
  // first make sure that we haven't filled up our allocation.
  // if we have, allocate a new array of twice the size, and copy
  // the old data to it.

  if (num_tris == num_tris_alloced)
    {
      // decide on new size -- accounting for first time, where none are 
      // allocated
      int n = num_tris_alloced*2;
      if (n == 0) n = 1;

      // make new array, and copy the old one to it
      tri *t = new tri[n];

      // if we can't get any more space, return an error
      if (!t)
	{
	  // we are leaving the model unchanged.
	  return RAPID_ERR_MODEL_OUT_OF_MEMORY;
	}
      
      int i;
      for(i=0; i<num_tris; i++) t[i] = tris[i]; 

      // free the old array and reassign.  
      delete [] tris;
      tris = t;
      
      // update the allocation counter.
      num_tris_alloced = n;
    }

  // now copy the new tri into the array
  tris[num_tris].p1[0] = p1[0];
  tris[num_tris].p1[1] = p1[1];
  tris[num_tris].p1[2] = p1[2];
  tris[num_tris].p2[0] = p2[0];
  tris[num_tris].p2[1] = p2[1];
  tris[num_tris].p2[2] = p2[2];
  tris[num_tris].p3[0] = p3[0];
  tris[num_tris].p3[1] = p3[1];
  tris[num_tris].p3[2] = p3[2];
  tris[num_tris].id = id;

  // update the counter
  num_tris++;

  return myrc;
}



// not a full sort -- just makes column 1 the largest
int
eigen_and_sort1(double evecs[3][3], double cov[3][3])
{
  double t;
  double evals[3];
  int n;

  n = Meigen(evecs, evals, cov);
  
  if (evals[2] > evals[0])
    {
      if (evals[2] > evals[1])
	{
	  // 2 is largest, swap with column 0
	  t = evecs[0][2]; 
	  evecs[0][2] = evecs[0][0]; 
	  evecs[0][0] = t;
	  t = evecs[1][2]; 
	  evecs[1][2] = evecs[1][0]; 
	  evecs[1][0] = t;
	  t = evecs[2][2]; 
	  evecs[2][2] = evecs[2][0]; 
	  evecs[2][0] = t;
	}
      else
	{
	  // 1 is largest, swap with column 0
	  t = evecs[0][1]; 
	  evecs[0][1] = evecs[0][0]; 
	  evecs[0][0] = t;
	  t = evecs[1][1]; 
	  evecs[1][1] = evecs[1][0]; 
	  evecs[1][0] = t;
	  t = evecs[2][1]; 
	  evecs[2][1] = evecs[2][0]; 
	  evecs[2][0] = t;
	}
    }
  else
    {
      if (evals[0] > evals[1])
	{
	  // 0 is largest, do nothing
	}
      else
	{
  	  // 1 is largest
	  t = evecs[0][1]; 
	  evecs[0][1] = evecs[0][0]; 
	  evecs[0][0] = t;
	  t = evecs[1][1]; 
	  evecs[1][1] = evecs[1][0]; 
	  evecs[1][0] = t;
	  t = evecs[2][1]; 
	  evecs[2][1] = evecs[2][0]; 
	  evecs[2][0] = t;
	}
    }

  // we are returning the number of iterations Meigen took.
  // too many iterations means our chosen orientation is bad.
  return n; 
}


inline
void
minmax(double &mn, double &mx, double v)
{
  if (v < mn) mn = v;
  else if (v > mx) mx = v;
}


static moment *RAPID_moment = 0;
static tri *RAPID_tri = 0;
static box *RAPID_boxes = 0;
static int RAPID_boxes_inited = 0;


/*

  There are <n> tri structures in an array starting at <t>.
  
  We are told that the mean point is <mp> and the orientation
  for the parent box will be <or>.  The split axis is to be the 
  vector given by <ax>.

  <or>, <ax>, and <mp> are model space coordinates.

*/



int
RAPID_model::build_hierarchy()
{
  // allocate the boxes and set the box list globals

  num_boxes_alloced = num_tris * 2;
  b = new box[num_boxes_alloced];
  if (b == 0) return RAPID_ERR_MODEL_OUT_OF_MEMORY;
  RAPID_boxes = b;
  RAPID_boxes_inited = 1;   // we are in process of initializing b[0].
  
  // Determine initial orientation, mean point, and splitting axis.

  int i; 
  accum M;
  
  //  double F1[3];
  //  double S1[6];
  double C[3][3];
  
  RAPID_moment = new moment[num_tris]; 
  if (RAPID_moment == 0)
    {
      delete [] b;
      return RAPID_ERR_MODEL_OUT_OF_MEMORY;
    }
  compute_moments(RAPID_moment, tris, num_tris);

  clear_accum(M);  
  for(i=0; i<num_tris; i++)
    accum_moment(M, RAPID_moment[i]);
  
  mean_from_accum(b[0].pT, M);
  covariance_from_accum(C, M);
  
  eigen_and_sort1(b[0].pR, C);

  // create the index list
  int *t = new int[num_tris];
  if (t == 0)
    {
      delete [] b;
      delete [] RAPID_moment;
      return RAPID_ERR_MODEL_OUT_OF_MEMORY;
    }
  for(i=0; i<num_tris; i++) t[i] = i;

  // set the tri pointer
  RAPID_tri = tris;
  
  // do the build
  int rc = b[0].split_recurse(t, num_tris);
  if (rc != RAPID_OK)
    {
      delete [] b;
      delete [] RAPID_moment;
      delete [] t;
      return RAPID_ERR_MODEL_OUT_OF_MEMORY;
    }
  
  // free the moment list
  delete [] RAPID_moment;  RAPID_moment = 0;

  // null the tri pointer
  RAPID_tri = 0;
  
  // free the index list
  delete [] t;

  return RAPID_OK;
}



inline
void
reaccum_moments(accum &A, int *t, int n)
{
  clear_accum(A);
  for(int i=0; i<n; i++)
      accum_moment(A, RAPID_moment[t[i]]);
}


int
box::split_recurse(int *t, int n)
{
  // The orientation for the parent box is already assigned to this->pR.
  // The axis along which to split will be column 0 of this->pR.
  // The mean point is passed in on this->pT.

  // When this routine completes, the position and orientation in model
  // space will be established, as well as its dimensions.  Child boxes
  // will be constructed and placed in the parent's CS.

  if (n == 1)
    {
      return split_recurse(t);
    }
  
  // walk along the tris for the box, and do the following:
  //   1. collect the max and min of the vertices along the axes of <or>.
  //   2. decide which group the triangle goes in, performing appropriate swap.
  //   3. accumulate the mean point and covariance data for that triangle.

  accum M1, M2;
  double C[3][3];
  double c[3];
  double minval[3], maxval[3];

  int rc;   // for return code on procedure calls.
  int in;
  tri *ptr;
  int i;
  double axdmp;
  int n1 = 0;  // The number of tris in group 1.  
  // Group 2 will have n - n1 tris.

  // project approximate mean point onto splitting axis, and get coord.
  axdmp = (pR[0][0] * pT[0] + pR[1][0] * pT[1] + pR[2][0] * pT[2]);

  clear_accum(M1);
  clear_accum(M2);

  MTxV(c, pR, RAPID_tri[t[0]].p1);
  minval[0] = maxval[0] = c[0];
  minval[1] = maxval[1] = c[1];
  minval[2] = maxval[2] = c[2];
  for(i=0; i<n; i++)
    {
      in = t[i];
      ptr = RAPID_tri + in;
      
      MTxV(c, pR, ptr->p1);
      minmax(minval[0], maxval[0], c[0]);
      minmax(minval[1], maxval[1], c[1]);
      minmax(minval[2], maxval[2], c[2]);

      MTxV(c, pR, ptr->p2);
      minmax(minval[0], maxval[0], c[0]);
      minmax(minval[1], maxval[1], c[1]);
      minmax(minval[2], maxval[2], c[2]);

      MTxV(c, pR, ptr->p3);
      minmax(minval[0], maxval[0], c[0]);
      minmax(minval[1], maxval[1], c[1]);
      minmax(minval[2], maxval[2], c[2]);

      // grab the mean point of the in'th triangle, project
      // it onto the splitting axis (1st column of pR) and
      // see where it lies with respect to axdmp.
      mean_from_moment(c, RAPID_moment[in]);
      
      if (((pR[0][0]*c[0] + pR[1][0]*c[1] + pR[2][0]*c[2]) < axdmp)
	  && ((n!=2)) || ((n==2) && (i==0)))    
	{
	  // accumulate first and second order moments for group 1
	  accum_moment(M1, RAPID_moment[in]);

	  // put it in group 1 by swapping t[i] with t[n1]
	  int temp = t[i];
	  t[i] = t[n1];
	  t[n1] = temp;
	  n1++;
	}
      else
	{
	  // accumulate first and second order moments for group 2
	  accum_moment(M2, RAPID_moment[in]);

	  // leave it in group 2
	  // do nothing...it happens by default
	}
    }

  // done using this->pT as a mean point.


  // error check!
  if ((n1 == 0) || (n1 == n))
    {
      // our partitioning has failed: all the triangles fell into just
      // one of the groups.  So, we arbitrarily partition them into
      // equal parts, and proceed.

      n1 = n/2;
      
      // now recompute accumulated stuff
      reaccum_moments(M1, t, n1);
      reaccum_moments(M2, t + n1, n - n1);
    }

  // With the max and min data, determine the center point and dimensions
  // of the parent box.

  c[0] = (minval[0] + maxval[0])*0.5;
  c[1] = (minval[1] + maxval[1])*0.5;
  c[2] = (minval[2] + maxval[2])*0.5;

  pT[0] = c[0] * pR[0][0] + c[1] * pR[0][1] + c[2] * pR[0][2];
  pT[1] = c[0] * pR[1][0] + c[1] * pR[1][1] + c[2] * pR[1][2];
  pT[2] = c[0] * pR[2][0] + c[1] * pR[2][1] + c[2] * pR[2][2];
  d[0] = (maxval[0] - minval[0])*0.5;
  d[1] = (maxval[1] - minval[1])*0.5;
  d[2] = (maxval[2] - minval[2])*0.5;

  // allocate new boxes
  P = RAPID_boxes + RAPID_boxes_inited++;
  N = RAPID_boxes + RAPID_boxes_inited++;

  // Compute the orienations for the child boxes (eigenvectors of
  // covariance matrix).  Select the direction of maximum spread to be
  // the split axis for each child.
  
  double tR[3][3];
  
  if (n1 > 1)
    {
      mean_from_accum(P->pT, M1);
      covariance_from_accum(C, M1);

      if (eigen_and_sort1(tR, C) > 30)
	{
	  // unable to find an orientation.  We'll just pick identity.
	  Midentity(tR);
	}

      McM(P->pR, tR);
      if ((rc = P->split_recurse(t, n1)) != RAPID_OK) return rc;
    }
  else
    {
      if ((rc = P->split_recurse(t)) != RAPID_OK) return rc;
    }
  McM(C, P->pR);  MTxM(P->pR, pR, C);   // and F1
  VmV(c, P->pT, pT);  MTxV(P->pT, pR, c);
  
  if ((n-n1) > 1)
    {      
      mean_from_accum(N->pT, M2);
      covariance_from_accum (C, M2);

      if (eigen_and_sort1(tR, C) > 30)
	{
	  // unable to find an orientation.  We'll just pick identity.
	  Midentity(tR);
	}
      
      McM(N->pR, tR);
      if ((rc = N->split_recurse(t + n1, n - n1)) != RAPID_OK) return rc;
    }
  else
    {
      if ((rc = N->split_recurse(t+n1)) != RAPID_OK) return rc;
    }
  McM(C, N->pR); MTxM(N->pR, pR, C);
  VmV(c, N->pT, pT);  MTxV(N->pT, pR, c);  

  return RAPID_OK;
}

int
box::split_recurse(int *t)
{
  // For a single triangle, orientation is easily determined.
  // The major axis is parallel to the longest edge.
  // The minor axis is normal to the triangle.
  // The in-between axis is determine by these two.

  // this->pR, this->d, and this->pT are set herein.

  P = N = 0;
  tri *ptr = RAPID_tri + t[0];

  // Find the major axis: parallel to the longest edge.
  double u12[3], u23[3], u31[3];

  // First compute the squared-lengths of each edge
  VmV(u12, ptr->p1, ptr->p2);  
  double d12 = VdotV(u12,u12);
  VmV(u23, ptr->p2, ptr->p3);  
  double d23 = VdotV(u23,u23);
  VmV(u31, ptr->p3, ptr->p1);  
  double d31 = VdotV(u31,u31);

  // Find the edge of longest squared-length, normalize it to
  // unit length, and put result into a0.
  double a0[3];
  double l;  
  if (d12 > d23)
    {
      if (d12 > d31)
	{
	  l = 1.0 / sqrt(d12); 
	  a0[0] = u12[0] * l; 
	  a0[1] = u12[1] * l;
	  a0[2] = u12[2] * l;
	}
      else 
	{
	  l = 1.0 / sqrt(d31);
	  a0[0] = u31[0] * l;
	  a0[1] = u31[1] * l;
	  a0[2] = u31[2] * l;
	}
    }
  else 
    {
      if (d23 > d31)
	{
	  l = 1.0 / sqrt(d23);
	  a0[0] = u23[0] * l;
	  a0[1] = u23[1] * l;
	  a0[2] = u23[2] * l;
	}
      else
	{
	  l = 1.0 / sqrt(d31);
	  a0[0] = u31[0] * l;
	  a0[1] = u31[1] * l;
	  a0[2] = u31[2] * l;
	}
    }

  // Now compute unit normal to triangle, and put into a2.
  double a2[3];
  VcrossV(a2, u12, u23);
  l = 1.0 / Vlength(a2);  a2[0] *= l;  a2[1] *= l;  a2[2] *= l;

  // a1 is a2 cross a0.
  double a1[3];
  VcrossV(a1, a2, a0);

  // Now make the columns of this->pR the vectors a0, a1, and a2.
  pR[0][0] = a0[0];  pR[0][1] = a1[0];  pR[0][2] = a2[0];
  pR[1][0] = a0[1];  pR[1][1] = a1[1];  pR[1][2] = a2[1];
  pR[2][0] = a0[2];  pR[2][1] = a1[2];  pR[2][2] = a2[2];
  
  // Now compute the maximum and minimum extents of each vertex 
  // along each of the box axes.  From this we will compute the 
  // box center and box dimensions.
  double minval[3], maxval[3];
  double c[3];
  
  MTxV(c, pR, ptr->p1);
  minval[0] = maxval[0] = c[0];
  minval[1] = maxval[1] = c[1];
  minval[2] = maxval[2] = c[2];

  MTxV(c, pR, ptr->p2);
  minmax(minval[0], maxval[0], c[0]);
  minmax(minval[1], maxval[1], c[1]);
  minmax(minval[2], maxval[2], c[2]);
  
  MTxV(c, pR, ptr->p3);
  minmax(minval[0], maxval[0], c[0]);
  minmax(minval[1], maxval[1], c[1]);
  minmax(minval[2], maxval[2], c[2]);
  
  // With the max and min data, determine the center point and dimensions
  // of the box
  c[0] = (minval[0] + maxval[0])*0.5;
  c[1] = (minval[1] + maxval[1])*0.5;
  c[2] = (minval[2] + maxval[2])*0.5;

  pT[0] = c[0] * pR[0][0] + c[1] * pR[0][1] + c[2] * pR[0][2];
  pT[1] = c[0] * pR[1][0] + c[1] * pR[1][1] + c[2] * pR[1][2];
  pT[2] = c[0] * pR[2][0] + c[1] * pR[2][1] + c[2] * pR[2][2];

  d[0] = (maxval[0] - minval[0])*0.5;
  d[1] = (maxval[1] - minval[1])*0.5;
  d[2] = (maxval[2] - minval[2])*0.5;
  
  // Assign the one triangle to this box
  trp = ptr;

  return RAPID_OK;
}
