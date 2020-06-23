
This is the RAPID collision detection package.

Compiling in UNIX
-----------------
Just type "make" in this directory to compile the package
and the test program.

The library will be compiled into 

	libRAPID.a

The only include file you need in your client
application is 

	RAPID.H

The sample client application, which will be built
in this directory is,

	sample_client

Run this program, and look over its source code 
(in sample_client.C) to get a feel for how to use RAPID.

A simple text user manual can be found in

	doc/user_manual.txt


Compiling using Microsoft Visual C++ 5.0
----------------------------------------
Go to the MSVC_Compile directory, open the project workspace
file RAPID201.dsw with Microsoft Developer Studio, and build
the "sample_client" project. The following files will be 
produced in this directory:

	RAPID.lib
        -- the RAPID library,

        sample_client.exe
        -- a sample client application program.

Run sample_client, and look over its source code 
(in sample_client.C) to get a feel for how to use RAPID.

To use RAPID.lib, you can add the file to your project, and 
include the header file RAPID.H in your source code.

A simple text user manual can be found in

	doc/user_manual.txt

---

Send comments and questions to "geom@cs.unc.edu",
and use "RAPID" in the subject line.
