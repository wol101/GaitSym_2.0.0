/*
 *  SphereGeom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 05/12/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#include "SphereGeom.h"

#ifdef USE_OPENGL
#include "FacetedSphere.h"
extern int gDrawGeomsFlag;
extern std::string gOBJName;
#endif

SphereGeom::SphereGeom(dSpaceID space, dReal radius)
{
    // create the geom
    m_GeomID = dCreateSphere(space, radius);
    dGeomSetData(m_GeomID, this);
}

#ifdef USE_OPENGL
void SphereGeom::Draw()
{
    if (m_Visible == false) return;

    if (gDrawGeomsFlag)
    {
        gOBJName = m_Name;

        const dReal *bodyRotation = dBodyGetRotation(dGeomGetBody(m_GeomID));;
        const dReal *cylinderRelPosition = dGeomGetOffsetPosition(m_GeomID);
        const dReal *cylinderRelRotation = dGeomGetOffsetRotation(m_GeomID);

        dVector3 p;
        dMatrix3 r;

        // get the cylinder position in world coordinates
        dBodyGetRelPointPos(dGeomGetBody(m_GeomID), cylinderRelPosition[0],cylinderRelPosition[1], cylinderRelPosition[2], p);

        //combine the body rotation with the cylinder rotation to get combined rotation from world coordinates
        dMultiply0(r, bodyRotation, cylinderRelRotation, 3, 3, 3);

        // get the radius
        dReal radius = dGeomSphereGetRadius (m_GeomID);

        // and draw the sphere
        const static int kLevels = 3;
        FacetedSphere sphere(radius, kLevels);
        sphere.SetColour(m_Colour);
        sphere.SetDisplayPosition(p[0], p[1], p[2]);
        sphere.SetDisplayRotation(r);
        sphere.Draw();
    }

}
#endif
