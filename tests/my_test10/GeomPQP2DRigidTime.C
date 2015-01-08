#undef max
#undef min

#include <fstream>
#include <math.h>
#include <cmath>


#include "/home/roomba/Documents/tri/MSLlibrary/include/MSL/geomPQP.h"

#include "GeomPQP2DRigidTime.h"






GeomPQP2DRigidTime::GeomPQP2DRigidTime(string path = ""):GeomPQP2DRigidMulti(path) {
  
  // FilePath is set in the base class
  LoadEnvironment(FilePath);
  LoadRobot(FilePath);
  LoadMovingObstacle(FilePath);
  
  READ_OPTIONAL_PARAMETER(CollisionPairs);
}

//Read file with configs through time. save to a vector
void GeomPQP2DRigidTime::LoadMovingObstacle(string path) {
	std::ifstream fin;

		fprintf(stderr, "Reading the Moving Obstacles\n");

	  MovingObsConf.clear();
	  fin.open((FilePath+"BodyConf").c_str());

	  if (fin)
	    fin >> MovingObsConf;

	  fin.close();

	  /**
	  int pos;
	  std::cout << "mylist contains:";
	  for (std::list<MSLVector>::iterator it=MovingObsConf.begin(); it != MovingObsConf.end(); ++it)
	      std::cout << ' ' << *it<<'\n';
	  **/
	  printf("Done reading the moving obstacles. \n");
}



/**
 * @param time is used as vector index for the moving obstacles
 */
//Add time param. Check first robot against others. Don't do pairwise checks.
//Modify collision checking to eliminate pairwise checks ..test first robot only
bool GeomPQP2DRigidTime::CollisionFree(const MSLVector &q){
	  int i,j;
	  list<MSLVector>::iterator v;

	  PQP_CollideResult cres;

	  printf("\nPrinting q:\n");
	  MSLVector qi;
	  qi = q;
	  qi.print();

	  SetTransformationSingle(q);
	  // Check for collisions with obstacles
	  //for (i = 0; i < NumBodies; i++) {

	  PQP_Collide(&cres,RR[0],TR[0],&Ro[0],RO,TO,&Ob,PQP_FIRST_CONTACT);
	  printf("\nPrinting in collision free\n");

	  if (cres.NumPairs() >= 1) return false;

	 // }

	  // Check for pairwise collisions
	  forall(v,CollisionPairs) {
	    i = (int) v->operator[](0);
	    j = (int) v->operator[](1);
	    PQP_Collide(&cres,RR[i],TR[i],&Ro[i],RR[j],TR[j],&Ro[j],PQP_FIRST_CONTACT);
	    if (cres.NumPairs() >= 1)
	      return false;
	  }




	  return true;
}


double GeomPQP2DRigidTime::DistanceComp(const MSLVector &q){
  int i,j;
  list<MSLVector>::iterator v;
  double dist = INFINITY;

  PQP_DistanceResult dres;  
  SetTransformation(q);

  // Check for collisions with obstacles
  for (i = 0; i < NumBodies; i++) {    
    PQP_Distance(&dres,RR[i],TR[i],&Ro[i],RO,TO,&Ob,0.0,0.0);    
    if (dres.Distance() < dist)
      dist = dres.Distance();
  }
  
  // Check for pairwise collisions
  forall(v,CollisionPairs) {
    i = (int) v->operator[](0); 
    j = (int) v->operator[](1);
    PQP_Distance(&dres,RR[i],TR[i],&Ro[i],RR[j],TR[j],&Ro[j],0.0,0.0);    
    if (dres.Distance() < dist)
      dist = dres.Distance();
  }

  return dist;
}


void GeomPQP2DRigidTime::SetTransformation(const MSLVector &q){

  int i;
  MSLVector qi(3);

  for (i = 0; i < NumBodies; i++) {
	// Get the configuration
    qi[0] = q[i*3];
    qi[1] = q[i*3+1];
    qi[2] = q[i*3+2]; 

    // Set translation
    TR[i][0]=(PQP_REAL)qi[0];
    TR[i][1]=(PQP_REAL)qi[1];
    TR[i][2]=0.0;
    
    // Set yaw rotation
    RR[i][0][0] = (PQP_REAL)(cos(qi[2]));
    RR[i][0][1] = (PQP_REAL)(-sin(qi[2]));
    RR[i][0][2] = 0.0;
    RR[i][1][0] = (PQP_REAL)(sin(qi[2]));
    RR[i][1][1] = (PQP_REAL)(cos(qi[2]));
    RR[i][1][2] = 0.0;
    RR[i][2][0] = 0.0;
    RR[i][2][1] = 0.0;
    RR[i][2][2] = 1.0;    
  }
}

  void GeomPQP2DRigidTime::SetTransformationSingle(const MSLVector &q){

	  int i;
	  MSLVector qi(3);

      i = 0;
      qi[0] = q[i];
      qi[1] = q[i+1];
      qi[2] = q[i+2];



      // Set translation
      TR[i][0]=(PQP_REAL)qi[0];
      TR[i][1]=(PQP_REAL)qi[1];
      TR[i][2]=0.0;

      // Set yaw rotation
      RR[i][0][0] = (PQP_REAL)(cos(qi[2]));
      RR[i][0][1] = (PQP_REAL)(-sin(qi[2]));
      RR[i][0][2] = 0.0;
      RR[i][1][0] = (PQP_REAL)(sin(qi[2]));
      RR[i][1][1] = (PQP_REAL)(cos(qi[2]));
      RR[i][1][2] = 0.0;
      RR[i][2][0] = 0.0;
      RR[i][2][1] = 0.0;
      RR[i][2][2] = 1.0;
      //printf("printing in set trans");
}

