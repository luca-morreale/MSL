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

class GeomPQP2DRigidTime: public GeomPQP2DRigidMulti {
 private:
  vector<list<MSLTriangle> > Robot;
  vector<PQP_Model> Ro;
  //vector<MSLVector> Ot; //vector of other robot
  list<MSLVector> MovingObsConf;
  list<MSLVector> CollisionPairs; // Index pairs to check for collision

  PQP_REAL TR[][3];  // Robot translation
  PQP_REAL RR[][3][3];  // Robot rotation
  //PQP_REAL TR[MAXBODIES][3];  // Robot translation
  //PQP_REAL RR[MAXBODIES][3][3];  // Robot rotation
 public:
  bool SelfCollisionCheck;
  GeomPQP2DRigidTime(string path);
  virtual ~GeomPQP2DRigidTime() {};
  virtual void LoadMovingObstacle(string path);//Load moving obstacles
  virtual bool CollisionFree(const MSLVector &q); // Input is configuration
  virtual double DistanceComp(const MSLVector &q);  // Distance in world
  void SetTransformation(const MSLVector &q); // Input is configuration
  void SetTransformationSingle(const MSLVector &q); // Input is configuration
//  void SetTransformationMulti(const MSLVector &q); // Input is configuration
};
