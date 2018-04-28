/*
 *  Strap.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include "Strap.h"

#ifdef USE_OPENGL
#include "FacetedObject.h"
#endif

Strap::Strap()
{
    m_Length = 0;
    m_LastLength = 0;
    m_Velocity = 0;
    m_Tension = 0;

#ifdef USE_OPENGL
    m_LastDrawTime = -1;
#endif

}

Strap::~Strap()
{
    std::vector<PointForce *>::const_iterator iter1;
    for (iter1 = m_PointForceList.begin(); iter1 != m_PointForceList.end(); iter1++)
        delete *iter1;

#ifdef USE_OPENGL
    std::vector<FacetedObject *>::const_iterator iterFO;
    for (iterFO = m_DrawList.begin(); iterFO != m_DrawList.end(); iterFO++)
        delete *iterFO;
#endif
}

