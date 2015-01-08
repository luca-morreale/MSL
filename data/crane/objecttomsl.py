from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import math
import sys
numVerts = 0  
verts = []  
norms = []  
vertsOut = []  
normsOut = []  
filename = sys.argv[1]
for line in open(filename, "r"):  
            vals = line.split()  
            if vals[0] == "v":  
                v = map(float, vals[1:4])  
                verts.append(v)  
            if vals[0] == "vn":  
                n = map(float, vals[1:4])  
                norms.append(n)  
            if vals[0] == "f":
		i=0
		vertex1=[]
		vertex2=[]
                vertex3=[]
                for f in vals[1:]:  
                    w = f.split("/")  
                    # OBJ Files are 1-indexed so we must subtract 1 below  
                    vertsOut.append(list(verts[int(w[0])-1])) 		    
		    if i==0:
			vertex1=[verts[int(w[0])-1][0],verts[int(w[0])-1][1],verts[int(w[0])-1][2]]			
		    if i==1:
			vertex2=[verts[int(w[0])-1][0],verts[int(w[0])-1][1],verts[int(w[0])-1][2]]
                    if i==2:
			vertex3=[verts[int(w[0])-1][0],verts[int(w[0])-1][1],verts[int(w[0])-1][2]]    
			print str(tuple(vertex1)),str(tuple(vertex2)),str(tuple(vertex3))              
                    numVerts += 1 
		    i=i+1 
      
