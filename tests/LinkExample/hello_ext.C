#include <list>
#include <string>
#include <fstream>
using namespace std;

#include </home/roomba/Documents/tri/MSLlibrary/include/MSL/model3d.h>
#include </home/roomba/Documents/tri/MSLlibrary/include/MSL/geomPQP.h>
#include </home/roomba/Documents/tri/MSLlibrary/include/MSL/problem.h>
#include </home/roomba/Documents/tri/MSLlibrary/include/MSL/rrt.h>


char const* greet()
{
	string filepath("./data");
	//Geom *g = new GeomPQP3DRigid(filepath);
	return "hello, world";
}
 
#include <boost/python.hpp>
 
BOOST_PYTHON_MODULE(hello_ext)
{
    using namespace boost::python;
    def("greet", greet);
}
