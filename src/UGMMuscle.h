/*
 *  UGMMuscle.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Mon Aug 16 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 *  This muscle model is based on:
 *  Umberger BR, Gerritsen KG, Martin PE.
 *  A model of human muscle energy expenditure.
 *  Comput Methods Biomech Biomed Engin. 2003 Apr;6(2):99-111.
 */

#ifndef UGMMuscle_h
#define UGMMuscle_h

#include "Muscle.h"

class Strap;

class UGMMuscle : public Muscle
{
public:

    UGMMuscle(Strap *strap);
    ~UGMMuscle();

    enum StrainModel
    {
        linear = 0,
        square
    };

    void SetStim(dReal stim, dReal timeIncrement);
    void SetFibreComposition(dReal fastTwitchFraction);
    void SetMuscleGeometry(dReal pcsa, dReal optimumFibreLength, dReal relativeWidth, dReal tendonLength,
                           StrainModel serialStrainModel, dReal serialStrainAtFmax,
                           StrainModel parallelStrainModel, dReal parallelStrainAtFmax);
    void SetModellingConstants(dReal specificTension, dReal relativeContractionVelocity, dReal muscleDensity);
    void SetAerobic(bool f) { if (f) m_s = 1.5; else m_s = 1.0; };
    void AllowReverseWork(bool f) { m_allowReverseWork = f; };

    virtual dReal GetMetabolicPower();

    dReal GetFCE() { return m_fce; };
    dReal GetLPE() { return m_lce; };
    dReal GetFPE() { return m_Strap->GetTension() - m_fce; };
    dReal GetLSE() { return m_Strap->GetLength() - m_lce; };
    dReal GetFSE() { return m_Strap->GetTension(); };
    dReal GetVCE() { return m_vce; };
    dReal GetVPE() { return m_vce; };
    dReal GetVSE() { return m_Strap->GetVelocity() - m_vce; };

    dReal GetPSE() { return GetVSE() * - GetFSE(); }; // power serial element
    dReal GetPPE() { return GetVPE() * - GetFPE(); }; // power parallel element
    dReal GetPCE() { return GetVCE() * - GetFCE(); }; // power contractile element
    dReal GetSSE() { return m_tendonlength; };
    dReal GetSPE() { return m_lceopt; };

    dReal GetESE();
    dReal GetEPE();

    virtual void SetActivation(dReal activation, dReal duration) { SetStim(activation, duration); };
    virtual dReal GetActivation() { return m_stim; };

    virtual void Dump();

protected:

    dReal m_specifictension;
    dReal m_density;
    dReal m_act;
    dReal m_stim;
    dReal m_ft;
    dReal m_arel;
    dReal m_brel;
    dReal m_tact;
    dReal m_tdeact;
    dReal m_pcsa;
    dReal m_lceopt;
    dReal m_width;
    dReal m_fmax;
    dReal m_s;
    dReal m_mass;
    dReal m_tendonlength;
    dReal m_fmaxecc;
    dReal m_slopfac;
    dReal m_amh;
    dReal m_vmaxst;
    dReal m_vmaxft;
    dReal m_shst;
    dReal m_shft;
    dReal m_lh;
    dReal m_fiso;
    dReal m_lce;
    dReal m_vce;
    dReal m_fce;
    dReal m_kse;
    dReal m_kpe;
    StrainModel m_parallelStrainModel;
    dReal m_parallelStrainAtFmax;
    StrainModel m_serialStrainModel;
    dReal m_serialStrainAtFmax;
    bool m_allowReverseWork;
    bool m_newObject;
};

#endif



