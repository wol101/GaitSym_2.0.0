/*
 *  BallJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/12/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#ifndef BallJoint_h
#define BallJoint_h

#include "Joint.h"

class BallJoint: public Joint
{
public:

    BallJoint(dWorldID worldID);

    void SetBallAnchor (dReal x, dReal y, dReal z);
    void SetBallAnchor(const char *buf);

    void GetBallAnchor(dVector3 result);
    void GetBallAnchor2(dVector3 result);

    virtual void Dump();

#ifdef USE_OPENGL
    virtual void Draw();
#endif

};



#endif
