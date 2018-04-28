/*
 *  DampedSpringMuscle.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// DampedSpringMuscle - implementation of a damped spring strap force

#ifndef DampedSpringMuscle_h
#define DampedSpringMuscle_h

#include "Muscle.h"

class Strap;

class DampedSpringMuscle : public Muscle
{
public:

    DampedSpringMuscle(Strap *strap);
    ~DampedSpringMuscle();

    void SetDamping(dReal d) { m_Damping = d; }; // value is in N/m2
    void SetSpringConstant(dReal k) { m_SpringConstant = k; }; // value is in N/m2
    void SetUnloadedLength(dReal l) { m_UnloadedLength = l; }; // value is in m
    void SetArea(dReal a) { m_Area = a; }; // value is in m2
    dReal GetDamping() { return m_Damping; }; // value is in N/m2
    dReal GetSpringConstant() { return m_SpringConstant; }; // value is in N/m2
    dReal GetUnloadedLength() { return m_UnloadedLength; }; // value is in m
    dReal GetArea() { return m_Area; }; // value is in m2
    dReal GetElasticEnergy();

    virtual void SetActivation(dReal activation, dReal duration);
    virtual dReal GetActivation() { return m_Activation; };
    virtual dReal GetMetabolicPower() { return 0; };

    virtual void Dump();

protected:

    dReal m_Damping;
    dReal m_SpringConstant;
    dReal m_UnloadedLength;
    dReal m_Area;

    dReal m_Activation;
};








#endif
