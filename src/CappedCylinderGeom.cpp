/*
 *  CappedCylinderGeom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#include "CappedCylinderGeom.h"

#ifdef USE_OPENGL
#include "FacetedConicSegment.h"
#include "FacetedSphere.h"
extern int gDrawGeomsFlag;
extern std::string gOBJName;
#endif

CappedCylinderGeom::CappedCylinderGeom(dSpaceID space, dReal radius, dReal length)
{
    // create the geom
#ifdef OLD_ODE
    m_GeomID = dCreateCCylinder(0, radius, length);
#else
    m_GeomID = dCreateCapsule(space, radius, length);
#endif
    dGeomSetData(m_GeomID, this);
}

#ifdef USE_OPENGL
void CappedCylinderGeom::Draw()
{
    if (m_Visible == false) return;

    if (gDrawGeomsFlag)
    {
        gOBJName = m_Name;

        const dReal *bodyRotation = dBodyGetRotation(dGeomGetBody(m_GeomID));
        const dReal *cylinderRelPosition = dGeomGetOffsetPosition(m_GeomID);
        const dReal *cylinderRelRotation = dGeomGetOffsetRotation(m_GeomID);

        dVector3 p;
        dMatrix3 r;

        // get the cylinder position in world coordinates
        dBodyGetRelPointPos(dGeomGetBody(m_GeomID), cylinderRelPosition[0],cylinderRelPosition[1], cylinderRelPosition[2], p);

        //combine the body rotation with the cylinder rotation to get combined rotation from world coordinates
        dMultiply0(r, bodyRotation, cylinderRelRotation, 3, 3, 3);

        // get the length and radius
        dReal radius, length;
#ifdef OLD_ODE
        dGeomCCylinderGetParams (m_GeomID, &radius, &length);
#else
        dGeomCapsuleGetParams (m_GeomID, &radius, &length);
#endif

        // and draw the capped cylinder
        const static int kSides = 128;
        FacetedConicSegment cylinder(length, radius, radius, kSides, 0, 0, -(length / 2));
        cylinder.SetColour(m_Colour);
        cylinder.SetDisplayRotation(r);
        cylinder.SetDisplayPosition(p[0], p[1], p[2]);
        cylinder.Draw();
        // and draw the spheres
        const static int kLevels = 3;
        FacetedSphere sphere(radius, kLevels);
        sphere.SetColour(m_Colour);
        sphere.SetDisplayRotation(r);
        dVector3 p1, p2;
        p1[0] = 0; p1[1] = 0; p1[2] = length / 2;
        dMULTIPLY0_331(p2, r, p1);
        sphere.SetDisplayPosition(p[0] + p2[0], p[1] + p2[1], p[2] + p2[2]);
        sphere.Draw();
        sphere.SetDisplayPosition(p[0] - p2[0], p[1] - p2[1], p[2] - p2[2]);
        sphere.Draw();
    }
}
#endif
