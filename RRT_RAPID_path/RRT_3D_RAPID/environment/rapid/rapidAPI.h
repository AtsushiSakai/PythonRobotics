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
#ifndef __RAPIDAPI_H_
#define __RAPIDAPI_H_

#include "rapid.h"

class RapidAPI{
  private:
    RAPID_model *b1;
    RAPID_model *b2;    
    
    int b1_counter;
    int b2_counter;
  public:
    RapidAPI(){
      b1 = new RAPID_model;
      b2 = new RAPID_model;
      b1_counter = 0;
      b2_counter = 0;
    }

    void API_add_obstacle_face(double p1[3], double p2[3], double p3[3]);
    void API_add_robot_face(double p1[3], double p2[3], double p3[3]);

    void API_finish_faces(void);

    bool API_check_collision(double P[16]);

};


#ifdef __cplusplus
extern "C" {
#endif

    RapidAPI* new_RapidAPI();
  
    void add_obstacle_face(RapidAPI* api, double p1[3], double p2[3], double p3[3]);
    void add_robot_face(RapidAPI* api, double p1[3], double p2[3], double p3[3]);

    void finish_faces(RapidAPI* api);

    bool check_collision(RapidAPI* api, double P[16]);

#ifdef __cplusplus
}
#endif

#endif
