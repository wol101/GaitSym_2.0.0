/*
 *  MAMuscleExtended.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 21/04/2006.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

// MAMuscleExtended - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a serial and parallel spring element

#ifndef MAMuscleExtended_h
#define MAMuscleExtended_h

#include "Muscle.h"

class Strap;
class MAMuscle;
class DampedSpringMuscle;
class SimpleStrap;
class Filter;

class MAMuscleExtended : public Muscle
{
public:

    MAMuscleExtended(Strap *strap);
    ~MAMuscleExtended();

    void SetSerialElasticProperties(dReal springConstant, dReal unloadedLength);
    void SetParallelElasticProperties(dReal springConstant, dReal unloadedLength);
    void SetMuscleProperties(dReal vMax, dReal F0, dReal K);
    //void SetSmoothing(int smoothing);
    void SetActivationKinetics(bool activationKinetics) { m_ActivationKinetics = activationKinetics; };

    virtual dReal GetMetabolicPower();

    virtual void SetActivation(dReal activation, dReal timeIncrement);
    virtual dReal GetActivation() { return m_Stim; };

    dReal GetFCE() { return fce; };
    dReal GetLPE() { return lpe; };
    dReal GetFPE() { return fpe; };
    dReal GetLSE() { return lse; };
    dReal GetFSE() { return fse; };
    dReal GetVCE() { return vce; };
    dReal GetVPE() { return vce; };
    dReal GetVSE() { return GetVelocity() - vce; };
    dReal GetESE() { if (lse > sse) return (0.5 * (lse - sse) * (lse - sse) * ese); else return 0; }; // energy serial element
    dReal GetEPE() { if (lpe > spe) return (0.5 * (lpe - spe) * (lpe - spe) * epe); else return 0; }; // energy parallel element
    dReal GetPSE() { return GetVSE() * -fse; }; // power serial element
    dReal GetPPE() { return GetVPE() * -fpe; }; // power parallel element
    dReal GetPCE() { return GetVCE() * -fce; }; // power contractile element
    dReal GetSSE() { return sse; };
    dReal GetSPE() { return spe; };

    virtual void Dump();

protected:

    dReal m_Stim;
    dReal m_Act;

    bool m_ActivationKinetics;

    dReal spe; // slack length parallel element (m)
    dReal epe; // elastic constant parallel element (N/m)
    dReal sse; // slack length serial element (m)
    dReal ese; // elastic constant serial element (N/m)

    dReal k; // shape constant
    dReal vmax; // maximum shortening velocity (m/s)
    dReal f0; // isometric force (N)

    dReal fce; // contractile force
    dReal lpe; // contractile and parallel length
    dReal fpe; // parallel element force
    dReal lse; // serial length
    dReal fse; // serial element force

    dReal vce; // contractile element velocity (m/s)

    dReal lastlpe; // last parallel element length (m)

};








#endif // MAMuscleExtended_h
