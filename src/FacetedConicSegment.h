/*
 *  FacetedConicSegment.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 06/01/2006.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

#ifndef FacetedConicSegment_h
#define FacetedConicSegment_h

#include <sstream>
#include "FacetedObject.h"

class FacetedConicSegment: public FacetedObject
{
public:
    FacetedConicSegment(dReal length, dReal r1, dReal r2, int sides, dReal ox, dReal oy, dReal oz);

    virtual void WritePOVRay(std::ostringstream &theString);

protected:
    dReal m_R1;
    dReal m_R2;
    dReal m_Length;
    dReal m_OX;
    dReal m_OY;
    dReal m_OZ;
    int m_Sides;
};

#endif
