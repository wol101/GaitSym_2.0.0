/*
 *  SimpleStrap.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 21/04/2006.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef SimpleStrap_h
#define SimpleStrap_h

#include "Strap.h"

class Body;

class SimpleStrap:public Strap
{
public:

    virtual void Calculate(dReal deltaT)
    {
        if (deltaT)
        {
            m_Velocity = (m_Length - m_LastLength) / deltaT;
        }
        else
        {
            m_Velocity = 0;
        }
        m_LastLength = m_Length;
    };
    
    void SetLength(dReal length) { m_Length = length; };

#ifdef USE_OPENGL
    virtual void Draw() {};
#endif

};

#endif
