
// these are the data elements and procedure declarations necessary for
// the RAPID_model class, but which we don't want visible in the 
// RAPID.H file, which the client programmer sees.

// This is an unconventional use of '#include' directive, but it reduces
// conceptual overhead for the client.

  box *b;
  int num_boxes_alloced;

  tri *tris;
  int num_tris;
  int num_tris_alloced;

  int build_state;
  
  int build_hierarchy();
  
  int RAPID_Collide(double R1[3][3], double T1[3], 
		       double s1, RAPID_model *RAPID_model1,
		       double R2[3][3], double T2[3], 
		       double s2, RAPID_model *RAPID_model2,
		       int flag);
