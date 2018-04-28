/*
 *  Muscle.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Muscle_h
#define Muscle_h

#include "NamedObject.h"
#include "Strap.h"
#include <vector>

class Muscle:public NamedObject
{
public:
    
    Muscle(Strap *strap);
    virtual ~Muscle();
    
    dReal GetLength() { return m_Strap->GetLength(); };
    dReal GetVelocity() { return m_Strap->GetVelocity(); };
    dReal GetTension() { return m_Strap->GetTension(); };
    dReal GetPower() { return -m_Strap->GetTension() * m_Strap->GetVelocity(); };
    
    void CalculateStrap(dReal deltaT) { m_Strap->Calculate(deltaT); }; 
    
    virtual void SetActivation(dReal activation, dReal duration) = 0;
    virtual dReal GetActivation() = 0;
    virtual dReal GetMetabolicPower() = 0;
    
    std::vector<PointForce *> *GetPointForceList() { return m_Strap->GetPointForceList(); };
    
    Strap *GetStrap() { return m_Strap; };
    
    void SetAllVisible(bool v) { SetVisible(v);  m_Strap->SetVisible(v); };

#ifdef USE_OPENGL
    virtual void Draw();
#endif
    
protected:
    
    Strap *m_Strap;
};

#endif

