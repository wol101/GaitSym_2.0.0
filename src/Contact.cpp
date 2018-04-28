/*
 *  Contact.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/02/2006.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#include "Contact.h"

#ifdef USE_OPENGL
#include "GLUtils.h"
extern int gDrawContactForces;
extern std::string gOBJName;
#endif

// length of vector a
#define LENGTHOF(a) \
        sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

#define LENGTH2OF(a) \
        (a[0]*a[0]+a[1]*a[1]+a[2]*a[2])


#ifdef USE_OPENGL
void Contact::Draw()
{
    if (m_Visible == false) return;

    if (gDrawContactForces)
    {
        gOBJName = m_Name;
        GLUtils::DrawAxes(m_AxisSize[0], m_AxisSize[1], m_AxisSize[2], 
                m_ContactPosition[0], m_ContactPosition[1], m_ContactPosition[2]);
        
        dVector3 direction;
        memcpy(direction, m_ContactJointFeedback.f1, sizeof(dVector3));
        dReal force = LENGTHOF(direction);
        direction[0] /= force;
        direction[1] /= force;
        direction[2] /= force;
        GLUtils::DrawCylinder(m_ContactPosition, direction, force * m_ForceScale, m_ForceRadius, m_ForceColour);
    }
}
#endif
