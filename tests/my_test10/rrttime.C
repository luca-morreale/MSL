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

#include "rrttime.h"


// *********************************************************************
// *********************************************************************
// CLASS:     RRTTime 
// 
// *********************************************************************
// *********************************************************************

RRTTime::RRTTime(Problem *problem): RRTConCon(problem) {
}



// This function essentially iterates Extend until it has to stop
// The same action is used for every iteration
bool RRTTime::Connect(const MSLVector &x, 
		  MSLTree *t,
		  MSLNode *&nn, bool forward = true) {
  MSLNode *nn_prev,*n_best;
  MSLVector nx,nx_prev,u_best;
  bool success;
  double d,d_prev,clock;
  int steps, time;
  
  n_best = SelectNode(x,t,forward);
  u_best = SelectInput(n_best->State(),x,nx,success,forward); 
	time = 0;  
	steps = 0;
           // nx gets next state
  if (success) {   // If a collision-free input was found
    d = P->Metric(nx,x); d_prev = d;
    nx_prev = nx; // Initialize
    nn = n_best;
    clock = PlannerDeltaT;
    //Satisfied sends nx to collisionfree in geompqp.c.
    //modify nx to include a time parameter starting at t = 0
    printf("Now printing nx : \n");
    nx.print();
    while ((P->Satisfied(nx))&&
	   (clock <= ConnectTimeLimit)&&
	   (d <= d_prev))
      {
			SatisfiedCount++;
			steps++; // Number of steps made in connecting
			nx_prev = nx;
			d_prev = d; nn_prev = nn;
			// Uncomment line below to select best action each time
			//u_best = SelectInput(g.inf(nn),x,nx,success,forward);
			if (Holonomic) {
				nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
			}
			else { // Nonholonomic
			  if (forward)
				nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
			  else
				nx = P->Integrate(nx_prev,u_best,-PlannerDeltaT);
		}
		d = P->Metric(nx,x);
		clock += PlannerDeltaT;
		time++;
		// Uncomment the subsequent two lines to
		//   make each intermediate node added
		//nn = g.new_node(nx_prev); // Make a new node
		//g.new_edge(nn_prev,nn,u_best);
      }
    nn = t->Extend(n_best, nx_prev, u_best, steps*PlannerDeltaT);
//    printf("\nPrinting n_best:\n");
//    n_best->State().print();
//    printf("\nPrinting time:\n");
//    cout<<n_best->Time();
//    printf("\nPrinting nx_prev : \n");
//    nx_prev.print();
//    printf("\nPrinting u_best : \n");
//    u_best.print();
//    printf("\n");
  }

  return success;
}


/**
bool RRT::Plan()
{
  int i;
  double d;
  MSLNode *n,*nn,*n_goal;
  MSLVector nx,u_best;
  list<MSLNode*> path;

  // Keep track of time
  float t = used_time();

  // Make the root node of G
  if (!T)
    T = new MSLTree(P->InitialState);

  nn = T->Root();

  i = 0;
  n = SelectNode(P->GoalState,T);
  n_goal = n;

  GoalDist = P->Metric(n->State(),P->GoalState);
  while ((i < NumNodes)&&(!GapSatisfied(n_goal->State(),P->GoalState))) {
    if (Extend(ChooseState(),T,nn)) { 
      d = P->Metric(nn->State(),P->GoalState);
      if (d < GoalDist) {  // Decrease if goal closer
	GoalDist = d;
	BestState = nn->State();
	n_goal = nn;
	//cout << "GoalDist " << GoalDist << "\n";
      }
    }
    i++;
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n"; 

  // Get the solution path
  if (GapSatisfied(n_goal->State(),P->GoalState)) {
    cout << "Success\n";
    path = T->PathToRoot(n_goal);
    path.reverse();
    RecordSolution(path); // Write to Path and Policy
    return true;
  }
  else {
    cout << "Failure\n";
    return false;
  }
}**/


