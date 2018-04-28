/*
 *  FloatingHingeJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 300/12/2006.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

#ifndef FloatingHingeJoint_h
#define FloatingHingeJoint_h

#include "Joint.h"

class FloatingHingeJoint: public Joint
{
public:
    
    FloatingHingeJoint(dWorldID worldID);
	
    void SetFloatingHingeAxis(dReal x, dReal y, dReal z); 
    void SetFloatingHingeAxis(const char *buf);
    
    void SetStartAngleReference(dReal startAngleReference);
    void SetJointStops(dReal loStop, dReal hiStop);
    
    void GetFloatingHingeAnchor(dVector3 result); 
    void GetFloatingHingeAnchor2(dVector3 result); 
    void GetFloatingHingeAxis(dVector3 result); 
    
    dReal GetFloatingHingeAngle(); 
    dReal GetFloatingHingeAngleRate(); 
    
#ifdef USE_OPENGL
    virtual void Draw();
#endif

protected:
        
    dReal m_StartAngleReference;
};



#endif
