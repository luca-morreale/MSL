### Makefile for GL-based RRT Planner

.SUFFIXES: .C .o

# General configurable part of the library
include ../Makefile.config

######### Finds python.h
PYH = -I /usr/include/python2.7

# location of the Python header files
PYTHON_VERSION = 2.7
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)
 
# location of the Boost Python include files and library
BOOST_INC = /home/roomba/Documents/tri/boost_1_55_0
BOOST_LIB = /home/roomba/Documents/tri/boost_1_55_0/libs

######### MSL Configuration
MSLINC = -I../../include
MSLLIB = -L../../lib -lMSL
MSLFLAGS = 

######## INC configuration

#INC = $(XINC) $(MSLINC) $(PQPINC) $(FOXINC) $(OTHERINC)
INC = $(CPPFLAGS) $(XINC) $(MSLINC) $(OTHERINC)

######## LIB configuration

#LIB = $(XLIB) $(MSLLIB) $(PQPLIB) $(FOXLIB) $(OTHERLIB)
LIB = $(LDFLAGS) $(MSLLIB) $(FOXLIB) $(XLIB) $(PQPLIB) $(OTHERLIB)

######## Flags

FLAGS = $(C++FLAGS) $(FOXFLAGS)


####### How to make things

.C.o:
	$(C++) $(FLAGS) -c $< $(INC)

all: helical nogui slide 

helical: helical.C modelnew.o
	$(C++) $(FLAGS) helical.C modelnew.o -o helical $(INC) $(LIB) $(GLINC) $(GLLIB)

nogui: nogui.C 
	$(C++) $(C++FLAGS) nogui.C -o nogui $(MSLINC) $(LIB) $(INC) $(PQPLIB) $(PYH)

slide: slide.C rrtslide.o
	$(C++) $(FLAGS) slide.C rrtslide.o -o slide $(INC) $(LIB) $(GLINC) $(GLLIB)

clean:
	rm -f *.o *.so helical slide nogui
