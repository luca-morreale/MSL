#undef max
#undef min

#include <fstream>
#include <math.h>
#include <cmath>
#include "/home/roomba/Documents/tri/MSLlibrary/include/MSL/geomPQP.h"
#include "GeomPQP2DRigidTime.h"



GeomPQP2DRigidTime::GeomPQP2DRigidTime(string path = ""):GeomPQP2DRigid(path) {
	 // Compute the maximum deviates -- 2D with rotation
	  double dmax = 0.0;
	  double mag;
	  time = 0;
	  p2 = 0;
	  list<MSLTriangle>::iterator tr;

	  LoadRobot(path);

	  forall(tr,Robot) {
	    // Find maximum effective radius; ignore z distance here
	    //mag = sqrt(sqr(tr->p1.xcoord())+sqr(tr->p1.ycoord()));
	    mag = sqrt(pow(tr->p1.xcoord(), 2)+pow(tr->p1.ycoord(), 2));
	    if (mag > dmax)
	      dmax = mag;
	    //mag = sqrt(sqr(tr->p2.xcoord())+sqr(tr->p2.ycoord()));
	    mag = sqrt(pow(tr->p2.xcoord(),2)+pow(tr->p2.ycoord(),2));
	    if (mag > dmax)
	      dmax = mag;
	  }

	  MaxDeviates = MSLVector(1.0,1.0,dmax);

	  LoadMovingObstacle(path);
	  LoadObsModels(path);

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

	  //Uncomment to print contents
	  /**
	  int pos;
	  std::cout << "mylist contains:";
	  for (std::list<MSLVector>::iterator it=MovingObsConf.begin(); it != MovingObsConf.end(); ++it)
	      std::cout << ' ' << *it<<'\n';
	  **/
	  printf("Done reading the moving obstacles. \n");
}



bool GeomPQP2DRigidTime::CollisionFree(const MSLVector &q){
  MSLVector MO, initial; //Moving Obstacle
  SetTransformation(q);
  int pos, i;
  pos = 0;
  for (std::list<MSLVector>::iterator it=MovingObsConf.begin(); it != MovingObsConf.end(); ++it){
	  if (pos==p2) {
		  MO = *it;
		  //std::cout << ' ' << *it<<'\n';

		  if ( distance( it, MovingObsConf.end() ) == 1 ) {
			 //Reversing the list
			 MovingObsConf.reverse();// at one before end
			 p2 = 0;
		  }
	  }
	  pos++;
  }

  SetObsTransformation(MO); //Moving Obstacles Transformations




  PQP_CollideResult cres;

  //Checks collisions of first robot against all static obstacles
  PQP_Collide(&cres,RR,TR,&Ro,RO,TO,&Ob,PQP_FIRST_CONTACT);
  if  (cres.NumPairs() != 0)
    return false;
    //Checks collisions of first robot against all moving obstacles
    for (i = 0; i < NumBodies; i++) {
	PQP_Collide(&cres,RR,TR,&Ro,MRO[i],MTO[i],&MOb[i],PQP_FIRST_CONTACT);
	if  (cres.NumPairs() != 0)
	    return false;
    }

   if  (cres.NumPairs() == 0){
	  time++;
	  p2++;
	  MO=q;
	  MO.print();
	  printf("\n");
	  //printf("The time : %d\n",time);
  }
  return (cres.NumPairs() == 0);

}




void GeomPQP2DRigidTime::SetObsTransformation(const MSLVector &q){

  int i;
  MSLVector qi(3);

  for (i = 0; i < NumBodies; i++) {
	// Get the configuration
	qi[0] = q[i*3];
	qi[1] = q[i*3+1];
	qi[2] = q[i*3+2];

	// Set translation
	MTO[i][0]=(PQP_REAL)qi[0];
	MTO[i][1]=(PQP_REAL)qi[1];
	MTO[i][2]=0.0;

	// Set yaw rotation
	MRO[i][0][0] = (PQP_REAL)(cos(qi[2]));
	MRO[i][0][1] = (PQP_REAL)(-sin(qi[2]));
	MRO[i][0][2] = 0.0;
	MRO[i][1][0] = (PQP_REAL)(sin(qi[2]));
	MRO[i][1][1] = (PQP_REAL)(cos(qi[2]));
	MRO[i][1][2] = 0.0;
	MRO[i][2][0] = 0.0;
	MRO[i][2][1] = 0.0;
	MRO[i][2][2] = 1.0;
  }
}


void GeomPQP2DRigidTime::LoadObsModels(string FilePath){

	  int i,j;
	  string fname;
	  list<MSLPolygon> pl;

	  char* s = new char[50];

	  // First check how many robot parts there are Robot0, Robot1, ...
	  i = 0;
	  sprintf(s,"%s/Robot%d",FilePath.c_str(),i);
	  while (is_file(s)) {
	    i++;
	    sprintf(s,"%s/Robot%d",FilePath.c_str(),i);
	  }
	  //NumBodies = i-1; //Move this

	  if (NumBodies == 0)
	    cout << "ERROR: No robot files at " << FilePath << "\n";

	  vector<list<MSLTriangle> > Robot;
	  Robot = vector<list<MSLTriangle> >(NumBodies);
	  MOb = vector<PQP_Model>(NumBodies);
	  for (i = 0; i < NumBodies; i++) {

	    sprintf(s,"%s/Robot%d",FilePath.c_str(),i+1);
	    std::ifstream fin(s);
	    pl.clear();
	    fin >> pl;

	    Robot[i] = PolygonsToTriangles(pl,5.0);
	    j=0;
	    list<MSLTriangle>::iterator t;

	    MOb[i].BeginModel();
	    PQP_REAL p1[3],p2[3],p3[3];
	    forall(t,Robot[i]){
	      p1[0] = (PQP_REAL) t->p1.xcoord();
	      p1[1] = (PQP_REAL) t->p1.ycoord();
	      p1[2] = (PQP_REAL) t->p1.zcoord();
	      p2[0] = (PQP_REAL) t->p2.xcoord();
	      p2[1] = (PQP_REAL) t->p2.ycoord();
	      p2[2] = (PQP_REAL) t->p2.zcoord();
	      p3[0] = (PQP_REAL) t->p3.xcoord();
	      p3[1] = (PQP_REAL) t->p3.ycoord();
	      p3[2] = (PQP_REAL) t->p3.zcoord();
	      MOb[i].AddTri(p1,p2,p3,j);
	      j++;
	    }
	    MOb[i].EndModel();
	  }
}

