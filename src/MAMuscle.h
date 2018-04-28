/*
 *  MAMuscle.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// MAMuscle - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a parallel spring element

#ifndef MAMuscle_h
#define MAMuscle_h

#include "Muscle.h"

class Strap;

class MAMuscle : public Muscle
{
public:

    MAMuscle(Strap *strap);
    ~MAMuscle();

    void SetVMax(dReal vMax) { m_VMax = vMax; };
    void SetF0(dReal f0) { m_F0 = f0; };
    void SetK(dReal k) { m_K = k; };

    virtual dReal GetMetabolicPower();

    virtual void SetActivation(dReal activation, dReal duration) { SetAlpha(activation); };
    virtual dReal GetActivation() { return m_Alpha; };

    virtual void Dump();

protected:

    void SetAlpha(dReal alpha);

    dReal m_VMax;
    dReal m_F0;
    dReal m_K;
    dReal m_Alpha;
};








#endif // MAMuscle_h
