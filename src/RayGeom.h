/*
 *  SphereGeom.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 14/08/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

#ifndef RAYGEOM_H
#define RAYGEOM_H

#include "Geom.h"

class RayGeom: public Geom
{
public:

    RayGeom(dSpaceID space, dReal length, dReal x, dReal y, dReal z, dReal dx, dReal dy, dReal dz);

    void SetParams(int firstContact, int backfaceCull, int closestHit);

#ifdef USE_OPENGL
    virtual void Draw();
#endif
};

#endif // RAYGEOM_H
