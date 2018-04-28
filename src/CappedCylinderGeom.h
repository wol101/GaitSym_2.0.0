/*
 *  CappedCylinderGeom.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef CappedCylinderGeom_h
#define CappedCylinderGeom_h

#include "Geom.h"

class CappedCylinderGeom:public Geom
{
public:
    
    CappedCylinderGeom(dSpaceID space, dReal radius, dReal length);
    
#ifdef USE_OPENGL
    virtual void Draw();
#endif
};


#endif

