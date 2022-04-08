#!/usr/bin/python

import sys
import math
import numpy as np

class SphericalFit:
    def __init__(self):
        #TODO remove if not needed
        i = 9

    #	fit a sphere to X,Y, and Z data points
    #	returns the radius and center points of
    #	the best fit sphere
    #takes sets of x y and z parameters as input spW is represented as a list of values in one dimension separated by ','
    def calculateFittingSphere(self, spX, spY, spZ):
        #point0 equals (xValues[0]|yValues[0]|zValues[0]) or in general pointI equals (xValues[I]|yValues[I]|zValues[I])
        #   Assemble the A matrix
        print(type(spX))
        print(type(spY))
        print(type(spZ))
        xValues = np.array(spX)
        yValues = np.array(spY)
        zValues = np.array(spZ)
        A = np.zeros((len(xValues),4))
        A[:,0] = xValues*2
        A[:,1] = yValues*2
        A[:,2] = zValues*2
        A[:,3] = 1
        #   Assemble the f matrix
        f = np.zeros((len(xValues),1))
        f[:,0] = (xValues*xValues) + (yValues*yValues) + (zValues*zValues)
        print(type(f))
        print(type(A))
        C, residules, rank, singval = np.linalg.lstsq(A,f, rcond=None)
        #   solve for the radius
        t = (C[0]*C[0])+(C[1]*C[1])+(C[2]*C[2])+C[3]
        radius = math.sqrt(t)

        return radius, C[0], C[1], C[2]
    
    def doSphereFit(self, points):
        xVals = list()
        yVals = list(float)
        zVals = list(float)
        for i in range(0, len(points)):
            xVals.append(float(points[i][0]))
            yVals.append(float(points[i][1]))
            zVals.append(float(points[i][2]))
        print(type(xVals))
        print(type(yVals))
        print(type(zVals))
        return self.calculateFittingSphere(xVals, yVals, zVals)


#example call
#           py script filepath              x coord.    y coord.    z coord.
# python .\Studienarbeit\sphericalFit.py -1,1,0,0,0,0 0,0,-1,1,0,0 0,0,0,0,-1,1
if __name__ == "__main__":
    print('Spherical Fit Main called')
    if len(sys.argv)==4:
        print('arg1: ', str(sys.argv[1]))
        print('arg2: ', str(sys.argv[2]))
        print('arg3: ', str(sys.argv[3]))
        sphere = SphericalFit()
        xVals = [float(x) for x in sys.argv[1].split(',')]
        yVals = [float(x) for x in sys.argv[2].split(',')]
        zVals = [float(x) for x in sys.argv[3].split(',')]
        radius, x0, y0, z0 = sphere.doSphereFit(xVals, yVals, zVals)
        print('Radius: ', radius, 'Center: (', str(x0), '|', str(y0), '|', str(z0), ')')
    else:
        print('Amount of arguments equals ', len(sys.argv))

