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
#include <stdio.h>
#include <stdlib.h>
#include "matvec.h"
#include "overlap.h"
#include "obb.h"

extern int RAPID_initialized;
void RAPID_initialize();

RAPID_model::RAPID_model() 
{
  if (!RAPID_initialized) RAPID_initialize();

  b = 0;
  num_boxes_alloced = 0;

  tris = 0;
  num_tris = 0;
  num_tris_alloced = 0;
  build_state = RAPID_BUILD_STATE_CONST;  
}

RAPID_model::~RAPID_model()
{
  if (!RAPID_initialized) RAPID_initialize();

  // the boxes pointed to should be deleted.
  delete [] b;

  // the triangles pointed to should be deleted.
  delete [] tris;
}

int RAPID_initialized = 0;

void
RAPID_initialize()
{
  RAPID_num_box_tests = 0;
  RAPID_num_contacts = 0;
  RAPID_contact = 0;

  RAPID_initialized = 1;
}             

