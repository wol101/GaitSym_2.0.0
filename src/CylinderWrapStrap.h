/*
 *  CylinderWrapStrap.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef CylinderWrapStrap_h
#define CylinderWrapStrap_h

#include "Strap.h"
#include "PGDMath.h"

class CylinderWrapStrap: public Strap
{
public:

    CylinderWrapStrap();
    virtual ~CylinderWrapStrap();

    void SetOrigin(Body *body, dVector3 point);
    void SetInsertion(Body *body, dVector3 point);
    void SetOrigin(Body *body, const char *buf);
    void SetInsertion(Body *body, const char *buf);
    
    void SetCylinderBody(Body *body);
    void SetCylinderRadius(dReal radius) { m_CylinderRadius = radius; };
    void SetCylinderPosition(dReal x, dReal y, dReal z);
    void SetCylinderQuaternion(dReal q0, dReal q1, dReal q2, dReal q3);
    void SetCylinderPosition(const char *buf);
    void SetCylinderQuaternion(const char *buf);
    void SetNumWrapSegments(int num) { m_NumWrapSegments = num; };

    virtual void Calculate(dReal deltaT);

    void GetOrigin(Body **body, dVector3 pos) { *body = m_OriginBody; pos[0] = m_OriginPosition.x; pos[1] = m_OriginPosition.y; pos[2] = m_OriginPosition.z; };
    void GetInsertion(Body **body, dVector3 pos) { *body = m_InsertionBody; pos[0] = m_InsertionPosition.x; pos[1] = m_InsertionPosition.y; pos[2] = m_InsertionPosition.z; };
    void GetCylinder(Body **body, dVector3 pos, dReal *radius, dQuaternion q);
    
#ifdef USE_OPENGL
    virtual void Draw();
    void SetCylinderColour(Colour &colour)  { m_CylinderColour = colour; };
    void SetCylinderLength(GLfloat length)  { m_CylinderLength = length; };
#endif
    
protected:

    Body *m_OriginBody;
    pgd::Vector m_OriginPosition;
    Body *m_InsertionBody;
    pgd::Vector m_InsertionPosition;
    
    Body *m_CylinderBody;
    pgd::Vector m_CylinderPosition;
    pgd::Quaternion m_CylinderQuaternion;
    dReal m_CylinderRadius;
    int m_NumWrapSegments;
    
    int m_WrapStatus;
    
    pgd::Vector *m_PathCoordinates;
    int m_NumPathCoordinates;

#ifdef USE_OPENGL
    Colour m_CylinderColour;
    GLfloat m_CylinderLength;
#endif
};

#endif

