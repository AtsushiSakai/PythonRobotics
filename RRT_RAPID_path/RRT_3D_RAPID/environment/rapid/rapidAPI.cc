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

#include "rapidAPI.h"
#include <math.h>
#include <stdio.h>

/*************************
  Rapid interface
**************************/

RapidAPI* new_RapidAPI(){
  return new RapidAPI();
}

void add_obstacle_face(RapidAPI* api, double p1[3], double p2[3], double p3[3]){
  api->API_add_obstacle_face(p1, p2, p3);
}

void add_robot_face(RapidAPI* api, double p1[3], double p2[3], double p3[3]){
  api->API_add_robot_face(p1, p2, p3);
}

void finish_faces(RapidAPI* api){
  api->API_finish_faces();
}

bool check_collision(RapidAPI* api, double P[16]){
  return api->API_check_collision(P);
}

/*************************
  Rapid API
**************************/

void RapidAPI::API_add_obstacle_face(double p1[3], double p2[3], double p3[3]){
  b1->AddTri(p1, p2, p3, b1_counter++);
}

void RapidAPI::API_add_robot_face(double p1[3], double p2[3], double p3[3]){
  b2->AddTri(p1, p2, p3, b2_counter++);
}

void RapidAPI::API_finish_faces(void){
  b1->EndModel();
  b2->EndModel();
}

bool RapidAPI::API_check_collision(double P[16]){

  double R1[3][3], R2[3][3], T1[3], T2[3];
  //first rotation matrix is always 0
  R1[0][0] = R1[1][1] = R1[2][2] = 1.0;
  R1[0][1] = R1[1][0] = R1[2][0] = 0.0;
  R1[0][2] = R1[1][2] = R1[2][1] = 0.0;

  //second matrix is extracted from P
  R2[0][0] = P[0];
  R2[0][1] = P[1];
  R2[0][2] = P[2];
  R2[1][0] = P[4];
  R2[1][1] = P[5];
  R2[1][2] = P[6];
  R2[2][0] = P[8];
  R2[2][1] = P[9];
  R2[2][2] = P[10];
  
  //first translation 
  T1[0] = 0.0;
  T1[1] = 0.0;
  T1[2] = 0.0;

  //second translation
  T2[0] = P[3];
  T2[1] = P[7];
  T2[2] = P[11];

  // now we can perform a collision query:
  RAPID_Collide(R1, T1, b1, R2, T2, b2, RAPID_FIRST_CONTACT);

  bool ret = true;
  if(RAPID_num_contacts == 0){
    ret = false;
  }
  return ret;
}
