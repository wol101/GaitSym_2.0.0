/*
 *  Joint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// wrapper class for a joint

#ifndef Joint_h
#define Joint_h

#include "NamedObject.h"

class Joint: public NamedObject
{
public:
    
    // Note constructor in derived functions need to set m_JointID
    // m_JointFeedback only needs to be set if GetFeedback is used (good idea though)
    
    virtual ~Joint();
    
	void Attach(dBodyID body1, dBodyID body2);
	dBodyID GetBody(int index);
	
	dJointFeedback *GetFeedback();
	    
#ifdef USE_OPENGL
    virtual void Draw() = 0;
#endif

protected:
        
    dJointID m_JointID;
	dJointFeedback m_JointFeedback;
};

#endif
