/*
 *  Joint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include "Joint.h"

Joint::~Joint()
{
	dJointDestroy (m_JointID);
}

void Joint::Attach(dBodyID body1, dBodyID body2)
{
	dJointAttach(m_JointID, body1, body2);
}
	
dBodyID Joint::GetBody(int index)
{
	return dJointGetBody(m_JointID, index);
}
	
dJointFeedback *Joint::GetFeedback()
{
	return dJointGetFeedback(m_JointID);
}


