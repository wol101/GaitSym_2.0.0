/*
 *  GLUtils.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 26/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifdef USE_OPENGL

#include <iostream>
#include <vector>

#include <gl.h>

#ifdef USE_GLUT_FOR_CHARACTERS
#include <glut.h>
#else
#include "StrokeFont.h"
#endif

#include "GLUtils.h"
#include "PGDMath.h"
#include "FacetedConicSegment.h"
#include "FacetedSphere.h"
#include "FacetedPolyline.h"

// length of vector a
#define LENGTHOF(a) \
        sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

#define LENGTH2OF(a) \
        (a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

// Widget drawing routines

void
GLUtils::DrawAxes(GLfloat x, GLfloat y, GLfloat z, GLfloat ox, GLfloat oy, GLfloat oz)
{
    glPushMatrix();
    glTranslatef(ox, oy, oz);


    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(x, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glEnd();

    glBegin(GL_LINES);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, y, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glEnd();

    glBegin(GL_LINES);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, z);
    glVertex3f(0.0, 0.0, 0.0);
    glEnd();

    glEnable(GL_LIGHTING);

    glPopMatrix();
}


// draw a cylinder from p1 to p2

void
GLUtils::DrawCylinder(dVector3 p1, dVector3 p2, GLfloat radius, Colour &colour)
{
    dVector3 axis;
    axis[0] = p2[0] - p1[0];
    axis[1] = p2[1] - p1[1];
    axis[2] = p2[2] - p1[2];
    GLfloat length = LENGTHOF(axis);
    DrawCylinder(p1, axis, length, radius, colour);
}

// Draw a cylinder with centre of one end at position aligned to axis
void
GLUtils::DrawCylinder(dVector3 position, dVector3 axis, GLfloat length, GLfloat radius, Colour &colour, bool useAxisLengthScaling)
{
    if (useAxisLengthScaling) length = length * sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);

#ifdef USE_CHECKLINES
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1.0, 1.0, 1.0);
    glVertex3f(position[0], position[1], position[2]);
    glVertex3f(position[0] + size * axis[0], position[1] + size * axis[1], position[2] + size * axis[2]);
    glEnd();
    glEnable(GL_LIGHTING);
#endif

    const static int kSides = 128;
    FacetedConicSegment cylinder(length, radius, radius, kSides, 0, 0, 0);
    cylinder.SetColour(colour);
    cylinder.SetDisplayRotationFromAxis(axis[0], axis[1], axis[2], false);
    cylinder.SetDisplayPosition(position[0], position[1], position[2]);
    cylinder.Draw();

#if 0
    dsSetColorAlpha(colour.r, colour.g, colour.b, colour.alpha);

    // calculate the rotation needed to get the axis pointing the right way
    dNormalize3(axis);
    dVector3 p, q;
    // calculate 2 perpendicular vectors
    dPlaneSpace(axis, p, q);
    // assemble the matrix
    dMatrix3 R;
    R[3] = R[7] = R[11] = 0;

    R[0] =    p[0]; R[4] =    p[1]; R[8] =     p[2];
    R[1] =    q[0]; R[5] =    q[1]; R[9] =     q[2];
    R[2] = axis[0]; R[6] = axis[1]; R[10] = axis[2];

    p[0] = position[0] + (axis[0] * length / 2);
    p[1] = position[1] + (axis[1] * length / 2);
    p[2] = position[2] + (axis[2] * length / 2);

    GLUtils::DrawCylinder(p, R, length, radius, colour, 0, 0, 0);
#endif
}

// draw a Cylinder aligned to the z axis
void
GLUtils::DrawCylinder(dVector3 p, dMatrix3 R, dReal length, dReal radius, Colour &colour, dReal ox, dReal oy, dReal oz)
{
    const static int kSides = 128;
    FacetedConicSegment cylinder(length, radius, radius, kSides, ox, oy, oz);
    cylinder.SetColour(colour);
    cylinder.SetDisplayRotation(R);
    cylinder.SetDisplayPosition(p[0], p[1], p[2]);
    cylinder.Draw();
}

/*
// draw a path made up of cylinders with spheres at the vertices
void GLUtils::DrawPath(pgd::Vector *pathCoordinates, int numPathCoordinates, dReal radius, Colour &colour)
{
    if (radius == 0)
    {
        glDisable(GL_LIGHTING);
        glBegin(GL_LINE_STRIP);
        glColor4f(colour.r, colour.g, colour.b, colour.alpha);
        glVertex3f(pathCoordinates[0].x, pathCoordinates[0].y, pathCoordinates[0].z);
        for (int i = 1; i < numPathCoordinates; i++)
        {
            glVertex3f(pathCoordinates[i].x, pathCoordinates[i].y, pathCoordinates[i].z);
        }
        glEnd();
        glEnable(GL_LIGHTING);
        return;
    }

    dVector3 p1, p2;
    for (int i = 0; i < numPathCoordinates - 1; i++)
    {
        p1[0] = pathCoordinates[i].x;
        p1[1] = pathCoordinates[i].y;
        p1[2] = pathCoordinates[i].z;
        p2[0] = pathCoordinates[i + 1].x;
        p2[1] = pathCoordinates[i + 1].y;
        p2[2] = pathCoordinates[i + 1].z;
        DrawCylinder(p1, p2, radius, colour);
    }
    for (int i = 0; i < numPathCoordinates; i++)
    {
        const static int kLevels = 3;
        FacetedSphere sphere(radius, kLevels);
        sphere.SetColour(colour);
        sphere.SetDisplayPosition(pathCoordinates[i].x, pathCoordinates[i].y, pathCoordinates[i].z);
        sphere.Draw();
    }
}
*/

// draw a path made up of a swept circle
void GLUtils::DrawPath(pgd::Vector *pathCoordinates, int numPathCoordinates, dReal radius, Colour &colour)
{
    const static int kSides = 128;
    if (radius == 0)
    {
        glDisable(GL_LIGHTING);
        glBegin(GL_LINE_STRIP);
        glColor4f(colour.r, colour.g, colour.b, colour.alpha);
        glVertex3f(pathCoordinates[0].x, pathCoordinates[0].y, pathCoordinates[0].z);
        for (int i = 1; i < numPathCoordinates; i++)
        {
            glVertex3f(pathCoordinates[i].x, pathCoordinates[i].y, pathCoordinates[i].z);
        }
        glEnd();
        glEnable(GL_LIGHTING);
        return;
    }

    // this is the checkline
    /*
    glDisable(GL_LIGHTING);
    glBegin(GL_LINE_STRIP);
    glColor4f(colour.r, colour.g, colour.b, colour.alpha);
    glVertex3f(pathCoordinates[0].x, pathCoordinates[0].y, pathCoordinates[0].z);
    for (int i = 1; i < numPathCoordinates; i++)
    {
        glVertex3f(pathCoordinates[i].x, pathCoordinates[i].y, pathCoordinates[i].z);
    }
    glEnd();
    glEnable(GL_LIGHTING);
    */


    std::vector<pgd::Vector> polyline;
    for (int i = 0; i < numPathCoordinates; i++) polyline.push_back(pathCoordinates[i]);
    FacetedPolyline fPoly(&polyline, radius, kSides);

    fPoly.SetColour(colour);
    // fPoly.SetBadMesh(true); // shouldn't be needed - these are hopefully well formed
    fPoly.Draw();
}

// write some 3D text at specified location x, y, z
// plane = 0; draw in x = 0 plane
// plane = 1; draw in y = 0 plane
// plane = 2; draw in z = 0 plane
void GLUtils::OutputText(GLfloat x, GLfloat y, GLfloat z, char *text, double size, int plane)
{
    if (plane < 0) plane = 0;
    else if (plane > 2) plane = 2;

#ifdef USE_GLUT_FOR_CHARACTERS
    glPushMatrix();
    glTranslatef(x, y, z);
    // characters are roughly 100 units so gScale accordingly
    glScalef( size / 100, size / 100, size / 100);
    // with no rotation, text is in the z = 0 plane
    if (plane == 0) glRotatef(90, 0, 1, 0); // x = 0 plane
    else if (plane == 1) glRotatef(90, 1, 0, 0); // y = 0 plane
    char *p;
    for (p = text; *p; p++)
        glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, *p);
    glPopMatrix();
#else
    glPushMatrix();
    glTranslatef(x, y, z);
    // with no rotation, text is in the z = 0 plane
    if (plane == 0) glRotatef(90, 0, 1, 0); // x = 0 plane
    else if (plane == 1) glRotatef(90, 1, 0, 0); // y = 0 plane
    StrokeString(
                 text,          /* character string */
                 strlen(text),  /* number of characters to draw */
                 0,             /* x coordinate of bottom left of character */
                 0,              /* y coordinate ... */
                 size,          /* cwidth of character cell */
                 size,          /* cheight of character cell */
                 0,             /* 0 - left, 1 - centre, 2 - right */
                 0,             /* 0 - bottom, 1 - centre, 2 - top */
                 0             /* if non-zero rotate everything 90 degrees AC */
    );
    glPopMatrix();
#endif
}

// set colour based on a colour map indexed from 0 to 1
void GLUtils::SetColourFromMap(GLfloat index, ColourMap m, Colour *mappedColour)
{
    const int precision = 256;
    static bool firstTime = true;
    static Colour greyMap[precision];
    static Colour sinMap[precision];
    int i;
    GLfloat v;
    GLfloat r;

    if (firstTime)
    {
        firstTime = false;
        for (i = 0; i < precision; i++)
        {
            v = (GLfloat)i / (GLfloat)(precision - 1);
            r = v * 2 * M_PI;
            greyMap[i].SetColour(v, v, v, 1);
            sinMap[i].SetColour((sin(r / 2 + M_PI / 2) + 1) / 2, (sin(r - M_PI / 2) + 1) / 2, (sin(r / 2 + 3 * M_PI / 2) + 1) / 2, 1);
        }
    }

    i = (int)(0.5 + index * (GLfloat)(precision - 1));
    if (i < 0) i = 0;
    else if (i >= precision) i = precision - 1;

    switch (m)
    {
        case GreyColourMap:
            mappedColour->SetColour(greyMap[i]);
            break;

        case SinColourMap:
            mappedColour->SetColour(sinMap[i]);
            break;
    }

    // printf("index = %f, %d\n", index, i);
    // printf("%f %f %f %f\n", mappedColour->r, mappedColour->g, mappedColour->b, mappedColour->alpha);
}

#endif

