//-------------------------------------------------------------------------
//                  The Motion Strategy Library (MSL)
//-------------------------------------------------------------------------
//
// Copyright (c) 2003 University of Illinois and Steven M. LaValle
// All rights reserved.
//
// Developed by:                Motion Strategy Laboratory
//                              University of Illinois
//                              http://msl.cs.uiuc.edu/msl/
//
// Versions of the Motion Strategy Library from 1999-2001 were developed
// in the Department of Computer Science, Iowa State University.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal with the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Motion Strategy Laboratory, University
//       of Illinois, nor the names of its contributors may be used to 
//       endorse or promote products derived from this Software without 
//       specific prior written permission.
//
// The software is provided "as is", without warranty of any kind,
// express or implied, including but not limited to the warranties of
// merchantability, fitness for a particular purpose and
// noninfringement.  In no event shall the contributors or copyright
// holders be liable for any claim, damages or other liability, whether
// in an action of contract, tort or otherwise, arising from, out of or
// in connection with the software or the use of other dealings with the
// software.
//
//-------------------------------------------------------------------------



#include <math.h>
#include <stdlib.h>

//#include <fx.h>
#include "../../configs/configFOX.h"

#include "../../include/MSL/rrt.h"
#include "../../include/MSL/rendergl.h"
#include "../../include/MSL/guiplanner.h"
#include "../../include/MSL/setup.h"
#include "../../include/MSL/defs.h"
#include "../../include/MSL/util.h"

#include "modelnew.h"

int main(int argc, char *argv[])
{
  string path;
  GuiPlanner *gui;


  Model *m = NULL;
  Geom *g = NULL;
  Problem *prob;

  if (argc < 2) {
    cout << "Usage:    plangl <problem path>\n";
    exit(-1);
  }

#ifdef WIN32
  path = string(argv[1]);
#else
  path = string(argv[1])+"/";
#endif

  if (!is_directory(path)) {
    cout << "Error:   Directory does not exist\n";
    exit(-1);
  }

#ifdef WIN32
  path = path + "/";
#endif

  SetupProblem(m,g,path);
  
  prob = new Problem(g,m,path);

  gui = new GuiPlanner(new RenderGL(new Scene(prob, path), path),
		       new RRTConCon(prob));

  gui->setArguments( argc, argv );
  
  gui->Start();

  return 0;
}

