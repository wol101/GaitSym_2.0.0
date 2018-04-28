/*
 *  MyEnvironment.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat Mar 26 2005.
 *  Copyright (c) 2005 Bill Sellers. All rights reserved.
 *
 */


#include <ode/ode.h>

#ifdef USE_OPENGL
#include <gl.h>
#include "GLUtils.h"

// ruler definition
static float gRulerMin = -500;
static float gRulerMax = 500;
static float gRulerTextSize = 0.1;
static float gRulerTextInterval = 1.0;
static float gRulerTickSize = 0.05;
static float gRulerTickInterval = 0.5;

#endif

#include "Environment.h"
#include "Geom.h"


Environment::Environment()
{
}

Environment::~Environment()
{
    std::vector<Geom *>::const_iterator iter1;
    for (iter1=m_GeomList.begin(); iter1 != m_GeomList.end(); iter1++)
        delete *iter1;
}

void Environment::AddGeom(Geom *geom)
{
    m_GeomList.push_back(geom);
}

#ifdef USE_OPENGL
void Environment::Draw()
{
    if (m_Visible == false) return;

    for (unsigned int i = 0; i < m_GeomList.size(); i++)
        m_GeomList[i]->Draw();
        
    // draw origin axes
    GLUtils::DrawAxes(m_AxisSize[0], m_AxisSize[1], m_AxisSize[2]);
    
    // draw ruler
    glDisable(GL_LIGHTING);
    glColor4f(m_Colour.r, m_Colour.g, m_Colour.b, m_Colour.alpha);
    char buffer[256];
    GLfloat v;
    for (v = gRulerMin; v <= gRulerMax; v += gRulerTextInterval)
    {
        sprintf(buffer, "%.1f", v);
        GLUtils::OutputText(v, 0, -2 * gRulerTextSize, buffer, gRulerTextSize, 1);
    }
    
    for (v = gRulerMin; v <= gRulerMax; v += gRulerTickInterval)
    {
        glBegin(GL_LINES);
        glVertex3f(v, 0, 0);
        glVertex3f(v, 0, -gRulerTickSize);
        glEnd();
    }
    glBegin(GL_LINES);
    glVertex3f(gRulerMin, 0, 0);
    glVertex3f(gRulerMax, 0, 0);
    glEnd();
    glEnable(GL_LIGHTING);

}
#endif
