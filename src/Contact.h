/*
 *  Contact.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/02/2002.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Contact_h
#define Contact_h

#include "NamedObject.h"

class Contact:public NamedObject
{
public:
    
    void SetJointID(dJointID jointID) { m_JointID = jointID; };
        
    dJointID GetJointID() { return m_JointID; };
    dJointFeedback* GetJointFeedback() { return &m_ContactJointFeedback; };
    dVector3* GetContactPosition() { return &m_ContactPosition; };
    
#ifdef USE_OPENGL
    void Draw();
    void SetForceColour(Colour &colour) { m_ForceColour = colour; };
    void SetForceRadius(GLfloat forceSize) { m_ForceRadius = forceSize; };
    void SetForceScale(GLfloat forceScale) { m_ForceScale = forceScale; };
#endif

protected:
        
    dJointID m_JointID;
    dJointFeedback m_ContactJointFeedback;
    dVector3 m_ContactPosition;

#ifdef USE_OPENGL
    Colour m_ForceColour;
    GLfloat m_ForceRadius;
    GLfloat m_ForceScale;
#endif
};


#endif

