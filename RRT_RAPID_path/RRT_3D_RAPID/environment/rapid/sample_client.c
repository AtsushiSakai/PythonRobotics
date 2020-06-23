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

\*************************************************************************/


#include <math.h>
#include <stdio.h>
#include "RAPID.H"
#define LISTS 0

#define M_PI 3.14159265358979323846


main()
{
  // first, get a couple of boxes in which to put our models

  RAPID_model *b1 = new RAPID_model;
  RAPID_model *b2 = new RAPID_model;
  
  // then, load the boxes with triangles.  The following loads each 
  // with a torus of 2*n1*n2 triangles.

  fprintf(stderr, "loading tris into RAPID_model objects...");  fflush(stderr);
  
  double a = 1.0;  // major radius of the tori
  double b = 0.2;  // minor radius of the tori

  int n1 = 50;     // tori will have n1*n2*2 triangles each
  int n2 = 50;

  int uc, vc;
  int count = 0;
  
  for(uc=0; uc<n1; uc++)
    for(vc=0; vc<n2; vc++)
      {
	double u1 = (2.0*M_PI*uc) / n1; 
	double u2 = (2.0*M_PI*(uc+1)) / n1; 
	double v1 = (2.0*M_PI*vc) / n2; 
	double v2 = (2.0*M_PI*(vc+1)) / n2; 

	double p1[3], p2[3], p3[3], p4[3];

	p1[0] = (a - b * cos(v1)) * cos(u1);
	p2[0] = (a - b * cos(v1)) * cos(u2);
	p3[0] = (a - b * cos(v2)) * cos(u1);
	p4[0] = (a - b * cos(v2)) * cos(u2);
	p1[1] = (a - b * cos(v1)) * sin(u1);
	p2[1] = (a - b * cos(v1)) * sin(u2);
	p3[1] = (a - b * cos(v2)) * sin(u1);
	p4[1] = (a - b * cos(v2)) * sin(u2);
	p1[2] = b * sin(v1);
	p2[2] = b * sin(v1);
	p3[2] = b * sin(v2);
	p4[2] = b * sin(v2);

	b1->AddTri(p1, p2, p3, count);
	b1->AddTri(p4, p2, p3, count+1);
	b2->AddTri(p1, p2, p3, count);
	b2->AddTri(p4, p2, p3, count+1);

	count += 2;
      }
  fprintf(stderr, "done\n");  fflush(stderr);
  fprintf(stderr, "Tori have %d triangles each.\n", count);
  fprintf(stderr, "building hierarchies...");  fflush(stderr);
  b1->EndModel();
  b2->EndModel();
  fprintf(stderr, "done.\n"); fflush(stderr); 
  
  // now we are free to call the interference detect routine.
  // but first, construct the transformations which define the placement
  // of our two hierarchies in world space:

  // this placement causes them to overlap a large amount.

  double R1[3][3], R2[3][3], T1[3], T2[3];
  
  R1[0][0] = R1[1][1] = R1[2][2] = 1.0;
  R1[0][1] = R1[1][0] = R1[2][0] = 0.0;
  R1[0][2] = R1[1][2] = R1[2][1] = 0.0;

  R2[0][0] = R2[1][1] = R2[2][2] = 1.0;
  R2[0][1] = R2[1][0] = R2[2][0] = 0.0;
  R2[0][2] = R2[1][2] = R2[2][1] = 0.0;
  
  T1[0] = 1.0;  T1[1] = 0.0; T1[2] = 0.0;
  T2[0] = 0.0;  T2[1] = 0.0; T2[2] = 0.0;

  // now we can perform a collision query:

  RAPID_Collide(R1, T1, b1, R2, T2, b2, RAPID_ALL_CONTACTS);

  // looking at the report, we can see where all the contacts were, and
  // also how many tests were necessary:

  printf("All contacts between overlapping tori:\n");
  
  printf("Num box tests: %d\n", RAPID_num_box_tests);
  printf("Num contact pairs: %d\n", RAPID_num_contacts);
#if LISTS
  int i;
  for(i=0; i<RAPID_num_contacts; i++)
    {
      printf("\t contact %4d: tri %4d and tri %4d\n", 
	     i, RAPID_contact[i].id1, RAPID_contact[i].id2);
    }
#endif

  // Notice the RAPID_ALL_CONTACTS flag we used in the call to collide().
  // The alternative is to use the FIRST_CONTACT flag, instead.
  // the result is that the collide routine searches for any contact,
  // but not all of them.  It takes many many fewer tests to locate a single
  // contact.

  RAPID_Collide(R1, T1, b1, R2, T2, b2, RAPID_FIRST_CONTACT);

  printf("First contact between overlapping tori:\n");
  
  printf("Num box tests: %d\n", RAPID_num_box_tests);
  printf("Num contact pairs: %d\n", RAPID_num_contacts);
#if LISTS
  for(i=0; i<RAPID_num_contacts; i++)
    {
      printf("\t contact %4d: tri %4d and tri %4d\n", 
	     i, RAPID_contact[i].id1, RAPID_contact[i].id2);
    }
#endif

  // by rotating one of them around the x-axis 90 degrees, they 
  // are now interlocked, but not quite touching.

  R1[0][0] = 1.0;  R1[0][1] = 0.0;  R1[0][2] = 0.0;
  R1[1][0] = 0.0;  R1[1][1] = 0.0;  R1[1][2] =-1.0;
  R1[2][0] = 0.0;  R1[2][1] = 1.0;  R1[2][2] = 0.0;
  
  RAPID_Collide(R1, T1, b1, R2, T2, b2, RAPID_FIRST_CONTACT);

  printf("No contact between interlocked but nontouching tori:\n");
  
  printf("Num box tests: %d\n", RAPID_num_box_tests);
  printf("Num contact pairs: %d\n", RAPID_num_contacts);
#if LISTS
  for(i=0; i<RAPID_num_contacts; i++)
    {
      printf("\t contact %4d: tri %4d and tri %4d\n", 
	     i, RAPID_contact[i].id1, RAPID_contact[i].id2);
    }
#endif

  // by moving one of the tori closer to the other, they
  // almost touch.  This is the case that requires a lot
  // of work wiht methods which use bounding boxes of limited
  // aspect ratio.  Oriented bounding boxes are more efficient
  // at determining noncontact than spheres, octree, or axis-aligned
  // bounding boxes for scenarios like this.  In this case, the interlocked
  // tori are separated by 0.0001 at their closest point.


  T1[0] = 1.5999;
  
  RAPID_Collide(R1, T1, b1, R2, T2, b2, RAPID_FIRST_CONTACT);

  printf("Many tests required for interlocked but almost touching tori:\n");
  
  printf("Num box tests: %d\n", RAPID_num_box_tests);
  printf("Num contact pairs: %d\n", RAPID_num_contacts);
#if LISTS
  for(i=0; i<RAPID_num_contacts; i++)
    {
      printf("\t contact %4d: tri %4d and tri %4d\n", 
	     i, RAPID_contact[i].id1, RAPID_contact[i].id2);
    }
#endif

  return 0;  
}

