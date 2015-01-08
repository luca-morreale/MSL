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









#include <fstream>
#include <math.h>
using namespace std;

#include </home/roomba/Documents/tri/MSLlibrary/include/MSL/model2d.h>
#include "modelnew.h"


// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidCarNew
//
// *********************************************************************
// *********************************************************************


Model2DRigidCarNew::Model2DRigidCarNew(string path = ""):Model2DRigid(path) {
  double alpha;
  MSLVector u;
  int i,j;
  READ_PARAMETER_OR_ERROR(NumBodies);
printf("numbodies: %d\n", NumBodies);
  StateDim = 3*NumBodies;
  InputDim = 2*NumBodies;
  
  printf("\nStateDim: %d\n", StateDim);
  
  MaxSteeringAngle = PI/12.0;
  CarLength = 2.0;


  //READ_PARAMETER_OR_DEFAULT(LowerState,MSLVector(StateDim));
  //READ_PARAMETER_OR_DEFAULT(UpperState,MSLVector(StateDim));
printf ("MSL vector of 0: %d", u.length() ) ;
u = MSLVector(3);
printf ("MSL vector of 4: %d", u.length() );
//printf("\nInput dim: %d", InputDim); 
  // Make the list of Inputs
  Inputs.clear();  // Otherwise its parent constructor will make some inputs


/**for (i = 0; i < StateDim; i++){
    for (j = 0; j < StateDim; j++){
	for (alpha = -MaxSteeringAngle; alpha <= MaxSteeringAngle; alpha += 2.0*MaxSteeringAngle/6.0)
    		Inputs.push_back(MSLVector(1.0,alpha)); }

    for (j = 0; j < StateDim; j++){
      	for (alpha = -MaxSteeringAngle; alpha <= MaxSteeringAngle; alpha += 2.0*MaxSteeringAngle/6.0)
    		Inputs.push_back(MSLVector(-1.0,alpha)); }
  }**/
 i = 0;
  for (alpha = -MaxSteeringAngle; alpha <= MaxSteeringAngle; 
       alpha += 2.0*MaxSteeringAngle/6.0) {
    /**u[0] = 1.0;
    u[1] = -1.0;
    u[2] = 0.0;
    u[3] = 0.0; 
	printf("\n\nDim of MSLVector: %d\n\n", u.dim());
    Inputs.push_back(MSLVector(u)); 
 	printf("i:%d\n",i);**/
	i++;
   // Inputs.push_back(MSLVector(-1.0,alpha,0,0)); 
   // Inputs.push_back(MSLVector(0,0,1.0,alpha)); 
   // Inputs.push_back(MSLVector(0,0,-1.0,alpha));
  }

	
  READ_OPTIONAL_PARAMETER(Inputs);
	printf ("here\n");
  
}
/**
double Model2DRigidCarNew::Metric(const MSLVector &x1, const MSLVector &x2) {

  double d,fd,dtheta;
  int i;

  d = 0.0;

  for (i = 0; i < NumBodies; i++) {
    fd = fabs(x1[3*i+2]-x2[3*i+2]);
    dtheta = min(fd,2.0*PI - fd);
    //d += sqr(x1[3*i] - x2[3*i]);
    d += pow(x1[3*i] - x2[3*i],2);
    //d += sqr(x1[3*i+1] - x2[3*i+1]);
    d += pow(x1[3*i+1] - x2[3*i+1],2);
    //d += sqr(dtheta);
    d += pow(dtheta,2);
  }

  return sqrt(d);
}


MSLVector Model2DRigidCarNew::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {
  MSLVector dx(6);
  dx[0] = u[0]*cos(x[2]);
  dx[1] = u[0]*sin(x[2]);
  dx[2] = u[0]*tan(u[1])/CarLength;
  
  dx[3] = u[2]*cos(x[2]);
  dx[4] = u[2]*sin(x[2]);
  dx[5] = u[2]*tan(u[3])/CarLength;
  return dx;
}**/
