/*
 *  GLUtils.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 26/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef GLUtils_h
#define GLUtils_h

#include <ode/ode.h>
#include <gl.h>

#include "PGDMath.h"

struct Colour
{
    GLfloat r;
    GLfloat g;
    GLfloat b;
    GLfloat alpha;
    
    void SetColour(GLfloat rf, GLfloat gf, GLfloat bf, GLfloat af) { r = rf;  g = gf;  b = bf;  alpha = af; };
    void SetColour(Colour &c) { r = c.r;  g = c.g;  b = c.b;  alpha = c.alpha; };
};

enum ColourMap
{
    GreyColourMap,
    SinColourMap
};

class GLUtils 
{
public:
    
    static void DrawAxes(GLfloat x, GLfloat y, GLfloat z, GLfloat ox = 0, GLfloat oy = 0, GLfloat oz = 0);
    static void DrawCylinder(dVector3 p1, dVector3 p2, GLfloat radius, Colour &colour);
    static void DrawCylinder(dVector3 position, dVector3 axis, GLfloat length, GLfloat radius, Colour &colour, bool useAxisLengthScaling = false);
    static void DrawCylinder(dVector3 p, dMatrix3 R, dReal length, dReal radius, Colour &colour, dReal ox, dReal oy, dReal oz);
    static void DrawPath(pgd::Vector *pathCoordinates, int numPathCoordinates, dReal radius, Colour &colour);
    static void OutputText(GLfloat x, GLfloat y, GLfloat z, char *text, double size, int plane);
    static void SetColourFromMap(GLfloat index, ColourMap m, Colour *mappedColour);
};

#endif
