/*
 *  Strap.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Strap_h
#define Strap_h

#include "NamedObject.h"
#include <vector>

class Body;
class FacetedObject;

struct PointForce
{
    Body *body;         // this is the body the force is applied to
    dVector3 point;     // this is the position of action in world coordinates
    dVector3 vector;    // this is the direction of action (magnitude acts as a scaling factor)
};

class Strap: public NamedObject
{
public:

    Strap();
    virtual ~Strap();

    dReal GetLength() { return m_Length; };
    dReal GetVelocity() { return m_Velocity; };

    void SetTension(dReal tension) { m_Tension = tension; };
    dReal GetTension() { return m_Tension; };

    std::vector<PointForce *> *GetPointForceList() { return &m_PointForceList; };

    virtual void Calculate(dReal deltaT) = 0;

#ifdef USE_OPENGL
    virtual void Draw() = 0;
    void SetRadius(GLfloat strapSize) { m_Radius = strapSize; };
    void SetForceColour(Colour &colour) { m_ForceColour = colour; };
    void SetForceRadius(GLfloat forceSize) { m_ForceRadius = forceSize; };
    void SetForceScale(GLfloat forceScale) { m_ForceScale = forceScale; };
#endif

protected:

    dReal m_Length;
    dReal m_LastLength;
    dReal m_Velocity;
    dReal m_Tension;

    std::vector<PointForce *> m_PointForceList;

#ifdef USE_OPENGL
    Colour m_ForceColour;
    GLfloat m_ForceRadius;
    GLfloat m_ForceScale;
    GLfloat m_Radius;

    std::vector<FacetedObject *> m_DrawList;
    dReal m_LastDrawTime;
#endif
};

#endif

