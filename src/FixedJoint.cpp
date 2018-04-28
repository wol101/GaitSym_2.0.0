/*
 *  FixedJoint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 20/09/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include "FixedJoint.h"

FixedJoint::FixedJoint(dWorldID worldID)
{
    m_JointID = dJointCreateFixed(worldID, 0);
    dJointSetData(m_JointID, this);
    
    dJointSetFeedback(m_JointID, &m_JointFeedback); 
}

void
FixedJoint::SetFixed()
{
    dJointSetFixed (m_JointID); 
}
