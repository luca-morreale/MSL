#undef max
#undef min

#include <list>
#include <vector>

//#include "../../configs/configPQP.h"
#include </home/roomba/Documents/tri/MSLlibrary/pqp/include/PQP.h>
#include "/home/roomba/Documents/tri/MSLlibrary/include/MSL/geomPQP.h"

#include <cstdlib>
#include <cstdio>

#include <string.h>

#include "/home/roomba/Documents/tri/MSLlibrary/include/MSL/defs.h"
#include "/home/roomba/Documents/tri/MSLlibrary/include/MSL/geom.h"
#include "/home/roomba/Documents/tri/MSLlibrary/include/MSL/point.h"  // This includes the MSLPolygon class
#include "/home/roomba/Documents/tri/MSLlibrary/include/MSL/triangle.h"


//! 2D rigid body

class GeomPQP2DRigidTime: public GeomPQP2DRigid {
 public:
  GeomPQP2DRigidTime(string path);
  virtual ~GeomPQP2DRigidTime() {};
  list<MSLVector> MovingObsConf;
  PQP_REAL MRO[MAXBODIES][3][3];
  PQP_REAL MTO[MAXBODIES][3];
  vector <PQP_Model> MOb;
  int time;
  int p2;
  virtual bool CollisionFree(const MSLVector &q); // Input is configuration
  virtual void LoadMovingObstacle(string path);//Load moving obstacles
  virtual void SetObsTransformation(const MSLVector &q);
  virtual void LoadObsModels(string path);

};
