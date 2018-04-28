/*
 *  FixedJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 20/09/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#ifndef FixedJoint_h
#define FixedJoint_h

#include "Joint.h"

class FixedJoint: public Joint
{
    public:
    
    FixedJoint(dWorldID worldID);
    
    void SetFixed(); 
    
#ifdef USE_OPENGL
    virtual void Draw() {};
#endif
    
};



#endif
