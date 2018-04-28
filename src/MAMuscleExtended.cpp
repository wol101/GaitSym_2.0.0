/*
 *  MAMuscleExtended.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 21/04/2006.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

// MAMuscleExtended - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a parallel spring element

#include <ode/ode.h>

#include "SimpleStrap.h"
#include "MAMuscleExtended.h"
#include "DebugControl.h"
#include "Simulation.h"

// Simulation global
extern Simulation *gSimulation;

// defines for Mathematica CForm output
#define Rule(x,y) x = (y)
#define Sqrt(x) sqrt(x)

inline dReal Power(dReal x, int y)
{
    switch (y)
    {
        case 0:
            return 1;

         case 1:
            return x ;

        case 2:
            return x * x;

        case 3:
            return x * x * x;

        case 4:
            return x * x * x * x;

        case 5:
            return x * x * x * x * x;

        default:
            return pow(x, y);
    }
}

// constructor

MAMuscleExtended::MAMuscleExtended(Strap *strap): Muscle(strap)
{
    m_Stim = 0;
    m_Act = 0;

    m_ActivationKinetics = true;

    spe = 0; // slack length parallel element (m)
    epe = 0; // elastic constant parallel element (N/m)
    sse = 0; // slack length serial element (m)
    ese = 0; // elastic constant serial element (N/m)

    k = 0; // shape constant
    vmax = 0; // maximum shortening velocity (m/s)
    f0 = 0; // isometric force (N)
}

// destructor
MAMuscleExtended::~MAMuscleExtended()
{
}

// set the muscle elastic properties
// if tendon length is set to < 0 then set it to a default slack value (doen later though)
void MAMuscleExtended::SetSerialElasticProperties(dReal springConstant, dReal unloadedLength)
{
    sse = unloadedLength; // slack length serial element (m)
        ese = springConstant; // elastic constant serial element (N/m)
}

void MAMuscleExtended::SetParallelElasticProperties(dReal springConstant, dReal unloadedLength)
{
        spe = unloadedLength; // slack length parallel element (m)
        epe = springConstant; // elastic constant parallel element (N/m)
}

// set the muscle contractile properties
void MAMuscleExtended::SetMuscleProperties(dReal vMax, dReal F0, dReal K)
{
    k = K; // shape constant
        vmax = vMax; // maximum shortening velocity (m/s)
    f0 = F0; // isometric force
}

// set the proportion of muscle fibres that are active
// calculates the tension in the strap

void MAMuscleExtended::SetActivation(dReal activation, dReal timeIncrement)
{
    const dReal goodEnough = 1e-10; // some help for rounding errors

    if (activation < 0) activation = 0;
    else if (activation > 1) activation = 1;
    m_Stim = activation;

    if (m_ActivationKinetics)
    {
        // using activation kinetics from UGM model
        double ft = 0.5; // arbitrary set activation kinetics as 50% fast twitch
        double tact = 80e-3 - 0.47e-3 * ft; // Umberger et al 2003 eq 4
        double tdeact = 90e-3 - 0.56e-3 * ft; // Umberger et al 2003 eq 4
        double t2 = 1 / tdeact;
        double t1 = 1 / tact - t2;
        // Nagano & Gerritsen 2001 A2
        double qdot = (m_Stim - m_Act) * (t1 * m_Stim + t2);
        m_Act += qdot * timeIncrement;
        if (m_Act < 0.001) m_Act = 0.001; // m_Act never drops to zero in practice
    }
    else
        m_Act = m_Stim;

    dReal alpha = m_Act;
    dReal len; // total length of system

    len = m_Strap->GetLength();

    int progress = 0;
    while (progress == 0)
    {
        // need to do something about first run with timeIncrement = 0 and lastlpe not set to anything useful
        if (timeIncrement == 0)
        {
            // handle sse < 0
            if (sse < 0) sse = len - spe;

            vce = 0;
            lpe = ((m_Act*f0*k*(vce+vmax)+(epe*spe+ese*(len-sse))*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
            if (lpe > spe) // pe not slack
            {
                fpe = ((epe*(m_Act*f0*k*(vce+vmax)+ese*(len-spe-sse)*(vce-k*vmax)))/((epe+ese)*(vce-k*vmax)));
                lse = ((-(m_Act*f0*k*(vce+vmax))+(epe*(len-spe)+ese*sse)*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
                fse = ((-(m_Act*ese*f0*k*(vce+vmax))+epe*ese*(len-spe-sse)*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
                fce = ((m_Act*f0*k*(vce+vmax))/(-vce+k*vmax));
            }
            else
            {
                lpe = (len - sse + (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
                lse = (sse - (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
                fse = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));
                fpe = (0);
                fce = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));
            }
            break;
        }

        // First assume vce <= 0 (concentric)
        // Solution 1
        Rule(fpe,(epe*(alpha*f0*k + epe*lastlpe + ese*lastlpe + ese*len - epe*spe - 2*ese*spe - ese*sse + epe*k*timeIncrement*vmax +
                       ese*k*timeIncrement*vmax - Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*
                                                       vmax + Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2))))/
             (2.*(epe + ese)));
        Rule(fse,-(ese*(alpha*f0*k + epe*lastlpe + ese*lastlpe - 2*epe*len - ese*len + epe*spe + 2*epe*sse + ese*sse +
                        epe*k*timeIncrement*vmax + ese*k*timeIncrement*vmax -
                        Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                             Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2))))/(2.*(epe + ese)));
        Rule(lse,-(alpha*f0*k + epe*lastlpe + ese*lastlpe - 2*epe*len - ese*len + epe*spe - ese*sse + epe*k*timeIncrement*vmax +
                   ese*k*timeIncrement*vmax - Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                                                   Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2)))/(2.*(epe + ese)));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 1;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // Solution 2
        Rule(fpe,(epe*
                  (alpha*f0*k + epe*lastlpe + ese*lastlpe + ese*len - epe*spe - 2*ese*spe - ese*sse + epe*k*timeIncrement*vmax +
                   ese*k*timeIncrement*vmax + Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*
                                                   vmax + Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2))))/
             (2.*(epe + ese)));
        Rule(fse,-(ese*(alpha*f0*k + epe*lastlpe + ese*lastlpe - 2*epe*len - ese*len + epe*spe + 2*epe*sse + ese*sse +
                        epe*k*timeIncrement*vmax + ese*k*timeIncrement*vmax +
                        Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                             Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2))))/(2.*(epe + ese)));
        Rule(lse,-(alpha*f0*k + epe*lastlpe + ese*lastlpe - 2*epe*len - ese*len + epe*spe - ese*sse + epe*k*timeIncrement*vmax +
                   ese*k*timeIncrement*vmax + Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                                                   Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2)))/(2.*(epe + ese)));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 2;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // Now try assuming vce > 0 (eccentric)

        // Solution 1
        Rule(fpe,(epe*(-850.5*alpha*f0 - 50.*alpha*f0*k + 472.5*epe*lastlpe + 472.5*ese*lastlpe + 472.5*ese*len - 472.5*epe*spe -
                       945.*ese*spe - 472.5*ese*sse - 62.5*epe*k*timeIncrement*vmax - 62.5*ese*k*timeIncrement*vmax -
                       0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                               Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                               Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                             1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                               epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                        lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                               236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                          epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                               15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                               alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                              k*(-47250. + 24999.999999999993*k)*timeIncrement*vmax) +
                                                         ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                              k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax))))/
                                              Power(timeIncrement,2))))/(945.*epe + 945.*ese));
        Rule(fse,(ese*(850.5*alpha*f0 + 50.*alpha*f0*k + epe*(-472.5*lastlpe + 945.*len - 472.5*spe - 945.*sse) +
                       ese*(-472.5*lastlpe + 472.5*len - 472.5*sse) + 62.5*epe*k*timeIncrement*vmax + 62.5*ese*k*timeIncrement*vmax +
                       0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                               Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                               Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                             1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                               epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                        lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                               236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                          epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                               15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                               alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                              k*(-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax) +
                                                         ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                              k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax))
                                                         ))/Power(timeIncrement,2))))/(945.*epe + 945.*ese));
        Rule(lse,(850.5*alpha*f0 + 50.*alpha*f0*k - 472.5*epe*lastlpe - 472.5*ese*lastlpe + 945.*epe*len + 472.5*ese*len - 472.5*epe*spe +
                  472.5*ese*sse + 62.5*epe*k*timeIncrement*vmax + 62.5*ese*k*timeIncrement*vmax +
                  0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                          Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                          Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                        1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                          epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                   lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                          236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                     epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                          15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                          alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                         k*(-47250. + 24999.999999999993*k)*timeIncrement*vmax) +
                                                    ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                         k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax))))/
                                         Power(timeIncrement,2)))/(945.*epe + 945.*ese));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 3;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // Solution 2
        Rule(fpe,
             (epe*(-850.5*alpha*f0 - 50.*alpha*f0*k + 472.5*epe*lastlpe + 472.5*ese*lastlpe + 472.5*ese*len - 472.5*epe*spe - 945.*ese*spe -
                   472.5*ese*sse - 62.5*epe*k*timeIncrement*vmax - 62.5*ese*k*timeIncrement*vmax +
                   0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                           Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                           Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                         1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                           epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                    lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                           236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                      epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                           15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                           alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                          k*(-47250. + 24999.999999999993*k)*timeIncrement*vmax) +
                                                     ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                          k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax))))/
                                          Power(timeIncrement,2))))/(945.*epe + 945.*ese));
        Rule(fse,(ese*(850.5*alpha*f0 + 50.*alpha*f0*k + epe*(-472.5*lastlpe + 945.*len - 472.5*spe - 945.*sse) +
                       ese*(-472.5*lastlpe + 472.5*len - 472.5*sse) + 62.5*epe*k*timeIncrement*vmax + 62.5*ese*k*timeIncrement*vmax -
                       0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                               Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                               Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                             1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                               epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                        lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                               236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                          epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                               15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                               alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                              k*(-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax) +
                                                         ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                              k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax))
                                                         ))/Power(timeIncrement,2))))/(945.*epe + 945.*ese));
        Rule(lse,(850.5*alpha*f0 + 50.*alpha*f0*k - 472.5*epe*lastlpe - 472.5*ese*lastlpe + 945.*epe*len + 472.5*ese*len - 472.5*epe*spe +
                  472.5*ese*sse + 62.5*epe*k*timeIncrement*vmax + 62.5*ese*k*timeIncrement*vmax -
                  0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                          Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                          Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                        1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                          epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                   lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                          236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                     epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                          15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                          alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                         k*(-47250. + 24999.999999999993*k)*timeIncrement*vmax) +
                                                    ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                         k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax))))/
                                         Power(timeIncrement,2)))/(945.*epe + 945.*ese));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 4;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // now consider special case if (lpe < spe) // pe slack special case

        // First assume vce <= 0 (concentric)
        // Solution 1
        Rule(lpe,(alpha*f0*k + ese*(lastlpe + len - sse + k*timeIncrement*vmax) -
                  Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                       Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/(2.*ese));
        Rule(fpe,0);
        Rule(fse,(-(alpha*f0*k) - ese*(lastlpe - len + sse + k*timeIncrement*vmax) +
                  Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                       Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/2.);

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 5;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // Solution 2
        Rule(lpe,(alpha*f0*k + ese*(lastlpe + len - sse + k*timeIncrement*vmax) +
                  Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                       Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/(2.*ese));
        Rule(fpe,0);
        Rule(fse,(-(alpha*f0*k) - ese*(lastlpe - len + sse + k*timeIncrement*vmax) -
                  Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                       Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/2.);

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 6;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // Now check if (vce > 0) (eccentric)

        // Solution 1
        Rule(lpe,(alpha*f0*(-0.9 - 0.052910052910052914*k) + 0.5*ese*lastlpe + 0.5*ese*len - 0.5*ese*sse -
                  0.06613756613756615*ese*k*timeIncrement*vmax - 0.0005291005291005291*timeIncrement*
                  Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                        Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse +
                                      893025.0000000005*Power(sse,2) + 236250.00000000006*k*len*timeIncrement*vmax -
                                      236250.00000000006*k*sse*timeIncrement*vmax + 15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                      lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000006*k*timeIncrement*vmax)) +
                        alpha*ese*f0*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                      k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 25000.000000000004*k)*timeIncrement*vmax)))/
                       Power(timeIncrement,2)))/ese);
        Rule(fpe,0.);
        Rule(fse,
             alpha*f0*(0.9 + 0.052910052910052914*k) - 0.5*ese*lastlpe + 0.5*ese*len - 0.5*ese*sse +
             0.06613756613756615*ese*k*timeIncrement*vmax + 0.0005291005291005291*timeIncrement*
             Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                   Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse +
                                 893025.0000000005*Power(sse,2) + 236250.00000000006*k*len*timeIncrement*vmax - 236250.00000000006*k*sse*timeIncrement*vmax +
                                 15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                 lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000006*k*timeIncrement*vmax)) +
                   alpha*ese*f0*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                 k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 25000.000000000004*k)*timeIncrement*vmax)))/
                  Power(timeIncrement,2)));

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 7;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // Solution 2
        Rule(lpe,(alpha*f0*(-0.9 - 0.052910052910052914*k) + 0.5*ese*lastlpe + 0.5*ese*len - 0.5*ese*sse -
                  0.06613756613756615*ese*k*timeIncrement*vmax + 0.0005291005291005291*timeIncrement*
                  Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                        Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse +
                                      893025.0000000005*Power(sse,2) + 236250.00000000006*k*len*timeIncrement*vmax -
                                      236250.00000000006*k*sse*timeIncrement*vmax + 15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                      lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000006*k*timeIncrement*vmax)) +
                        alpha*ese*f0*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                      k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 25000.000000000004*k)*timeIncrement*vmax)))/
                       Power(timeIncrement,2)))/ese);
        Rule(fpe,0.);
        Rule(fse,
             alpha*f0*(0.9 + 0.052910052910052914*k) - 0.5*ese*lastlpe + 0.5*ese*len - 0.5*ese*sse +
             0.06613756613756615*ese*k*timeIncrement*vmax - 0.0005291005291005291*timeIncrement*
             Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                   Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse +
                                 893025.0000000005*Power(sse,2) + 236250.00000000006*k*len*timeIncrement*vmax - 236250.00000000006*k*sse*timeIncrement*vmax +
                                 15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                 lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000006*k*timeIncrement*vmax)) +
                   alpha*ese*f0*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                 k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 25000.000000000004*k)*timeIncrement*vmax)))/
                  Power(timeIncrement,2)));

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 8;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // no consistent result found - usually because vce is out of range
        // we shouldn't get here too often but a few time - especially at the beginning of a simulation
        // or after an impact should be OK
        progress = 9;
        if (m_Strap->GetVelocity() > vmax) vce = vmax;
        else if (m_Strap->GetVelocity() < -vmax) vce = -vmax;
        else vce = 0;
        lpe = ((m_Act*f0*k*(vce+vmax)+(epe*spe+ese*(len-sse))*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
        if (lpe > spe) // pe not slack
        {
            fpe = ((epe*(m_Act*f0*k*(vce+vmax)+ese*(len-spe-sse)*(vce-k*vmax)))/((epe+ese)*(vce-k*vmax)));
            fse = ((-(m_Act*ese*f0*k*(vce+vmax))+epe*ese*(len-spe-sse)*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
            /*lse = ((-(m_Act*f0*k*(vce+vmax))+(epe*(len-spe)+ese*sse)*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
            fce = ((m_Act*f0*k*(vce+vmax))/(-vce+k*vmax));*/
        }
        else
        {
            progress = 10;
            lpe = (len - sse + (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
            fse = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));
            fpe = (0);
            /*lse = (sse - (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
            fce = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));*/
        }
        lse = len - lpe;
        fce = fse - fpe;
        if (fce < 0) fce = 0; // sanity check
        if (fse < 0) fse = 0; // sanity check
    }

    lastlpe = lpe;
    m_Strap->SetTension(fse);

    if (gDebug == MAMuscleExtendedDebug)
    {
        if (DebugFilters("SetActivation", m_Name))
        {
            static int firstTime = true;
            if (firstTime) // only happens once
            {
                firstTime = false;
                *gDebugStream << "MAMuscleExtended::SetActivation " <<
                " progress " <<
                " m_Stim " <<
                " m_Act " <<
                " len " <<
                " fpe " <<
                " lpe " <<
                " lse " <<
                " fse " <<
                " fce " <<
                " m_Velocity " <<
                " m_Tension " <<
                " vce " <<
                " serialStrainEnergy " <<
                " parallelStrainEnergy " <<
                "\n";
            }

            dReal serialStrainEnergy = 0.5 * (lse - sse) * (lse - sse) * ese;
            dReal parallelStrainEnergy = 0.5 * (lpe - spe) * (lpe - spe) * epe;

            *gDebugStream << m_Name <<
            " " << progress <<
            " " << m_Stim <<
            " " << m_Act <<
            " " << len <<
            " " << fpe <<
            " " << lpe <<
            " " << lse <<
            " " << fse <<
            " " << fce <<
            " " << m_Strap->GetVelocity() <<
            " " << m_Strap->GetTension() <<
            " " << vce <<
            " " << serialStrainEnergy <<
            " " << parallelStrainEnergy <<
            "\n";
        }
    }
}

#if 0
void MAMuscleExtended::SetActivation(dReal activation, dReal timeIncrement)
{
    if (activation < 0) activation = 0;
    else if (activation > 1) activation = 1;
    m_Stim = activation;

    if (m_ActivationKinetics)
    {
        // using activation kinetics from UGM model
        double ft = 0.5; // arbitrary set activation kinetics as 50% fast twitch
        double tact = 80e-3 - 0.47e-3 * ft; // Umberger et al 2003 eq 4
        double tdeact = 90e-3 - 0.56e-3 * ft; // Umberger et al 2003 eq 4
        double t2 = 1 / tdeact;
        double t1 = 1 / tact - t2;
        // Nagano & Gerritsen 2001 A2
        double qdot = (m_Stim - m_Act) * (t1 * m_Stim + t2);
        m_Act += qdot * timeIncrement;
    }
    else
        m_Act = m_Stim;

        dReal fce; // contractile force
    dReal lpe; // contractile and parallel length
    dReal fpe; // parallel element force
    dReal lse; // serial length
    dReal fse; // serial element force
    dReal len; // total length of system

    len = m_Strap->GetLength();

    // velocity fix
    if (vce < -vmax) vce = -vmax;
    else if (vce > vmax) vce = vmax;

    if (vce <= 0) // concentric
    {
        lpe = ((m_Act*f0*k*(vce+vmax)+(epe*spe+ese*(len-sse))*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
        if (lpe > spe) // pe not slack
        {
            fpe = ((epe*(m_Act*f0*k*(vce+vmax)+ese*(len-spe-sse)*(vce-k*vmax)))/((epe+ese)*(vce-k*vmax)));
            // lse = ((-(m_Act*f0*k*(vce+vmax))+(epe*(len-spe)+ese*sse)*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
            // fse = ((-(m_Act*ese*f0*k*(vce+vmax))+epe*ese*(len-spe-sse)*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
            fce = ((m_Act*f0*k*(vce+vmax))/(-vce+k*vmax));
        }
        else
        {
            lpe = (len - sse + (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
            // lse = (sse - (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
            // fse = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));
            fpe = (0);
            fce = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));
        }
    }
    else // eccentric
    {
        lpe = (0./(1.*epe+1.*ese)-1.*sse+(1.*ese*len+1.*epe*spe+1.*epe*sse)/(1.*epe+1.*ese)+(m_Act*f0*((13.608+0.8*k)*vce+1.*k*vmax))/((-1.*epe-1.*ese)*(7.56*vce+k*vmax)));
        if (lpe > spe) // pe not slack
        {
            fpe = (((epe*epe)*(1.*len-1.*spe-1.*sse))/(-1.*epe-1.*ese)+epe*(0./(1.*epe+1.*ese)+1.*len-1.*spe-1.*sse+(m_Act*f0*(13.608*vce+0.8*k*vce+1.*k*vmax))/((-1.*epe-1.*ese)*(7.56*vce+k*vmax))));
            // lse = (1.*sse-(1.*((ese*(0.+epe*(1.*len-1.*spe-1.*sse)))/(-1.*epe-1.*ese)+(1.*m_Act*ese*f0*(1.8+(0.8*k*(vce-1.*vmax))/(7.56*vce+k*vmax)))/(-1.*epe-1.*ese)))/ese);
            // fse = ((ese*(0.+epe*(-1.*len+1.*spe+1.*sse)))/(-1.*epe-1.*ese)-(1.*m_Act*ese*f0*(1.8+(0.8*k*(vce-1.*vmax))/(7.56*vce+k*vmax)))/(-1.*epe-1.*ese));
            fce = (1.*m_Act*f0*(1.8+(0.8*k*(vce-1.*vmax))/(7.56*vce+k*vmax)));
        }
        else
        {
            lpe = (1.*len - 1.*sse + (0. + (m_Act*f0*(-13.608*vce - 0.8*k*vce - 1.*k*vmax))/(7.56*vce + k*vmax))/ese);
            // lse = (1.*sse + (0. + (m_Act*f0*(13.608*vce + 0.8*k*vce + 1.*k*vmax))/(7.56*vce + k*vmax))/ese);
            // fse = (0. + (m_Act*f0*((13.608 + 0.8*k)*vce + 1.*k*vmax))/(7.56*vce + k*vmax));
            fpe = (0.);
            fce = (1.*m_Act*f0*(1.8 + (0.8*k*(vce - 1.*vmax))/(7.56*vce + k*vmax)));
        }
    }
    lse = len - lpe;
    fse = fpe + fce;

    m_Strap->SetTension(fse);

    // calculate a new vce
    if (timeIncrement)
    {
        newvce = (lpe - lastlpe) / timeIncrement;
    }
    else
        newvce = 0;

    if (vcelistsize)
    {
        vcelist[vcelistindex % vcelistsize] = newvce;
        vcelistindex++; // this will fail if vcelistindex overflows but that that should take a very long time indeed!
        vcesum += (newvce - vcelist[vcelistindex % vcelistsize]);
        vce = vcesum / vcelistsize;
    }
    else
    {
        vce = newvce;
    }

    lastlpe = lpe;

    if (gDebug == MAMuscleExtendedDebug)
    {
        if (DebugFilters("SetActivation", m_Name))
        {
            if (timeIncrement == 0) // only happens at initialisation
            {
                *gDebugStream << "MAMuscleExtended::SetActivation " <<
                " m_Stim " <<
                " m_Act " <<
                " len " <<
                " fpe " <<
                " lpe " <<
                " lse " <<
                " fse " <<
                " fce " <<
                " m_Velocity " <<
                " m_Tension " <<
                " vce " <<
                " newvce " <<
                " rippleCount " <<
                " ripple " <<
                " serialStrainEnergy " <<
                " parallelStrainEnergy " <<
                "\n";
            }

            int rippleCount = 0, i;
            dReal ripple = 0;
            dReal a, b, c;
            for (i = 2; i < vcelistsize; i++)
            {
                a = vcelist[i - 2];
                b = vcelist[i - 1];
                c = vcelist[i];
                if (a < b && b > c)
                {
                    rippleCount++;
                    ripple += (b - a);
                }
                else if (a > b && b < c)
                {
                    rippleCount++;
                    ripple += (a - b);
                }
            }

            dReal serialStrainEnergy = 0.5 * (lse - sse) * (lse - sse) * ese;
            dReal parallelStrainEnergy = 0.5 * (lpe - spe) * (lpe - spe) * epe;

            *gDebugStream << m_Name <<
            " " << m_Stim <<
            " " << m_Act <<
            " " << len <<
            " " << fpe <<
            " " << lpe <<
            " " << lse <<
            " " << fse <<
            " " << fce <<
            " " << m_Strap->GetVelocity() <<
            " " << m_Strap->GetTension() <<
            " " << vce <<
            " " << newvce <<
            " " << rippleCount <<
            " " << ripple <<
            " " << serialStrainEnergy <<
            " " << parallelStrainEnergy <<
            "\n";
        }
    }
}
#endif

// calculate the metabolic power of the muscle

dReal MAMuscleExtended::GetMetabolicPower()
{
    // m_Velocity is negative when muscle shortening
    // we need the sign the other way round
    dReal relV = -vce / vmax;

    // limit relV
    if (relV > 1) relV = 1;
    else if (relV < -1) relV = -1;

    dReal relVSquared = relV * relV;
    dReal relVCubed = relVSquared * relV;

    dReal sigma = (0.054 + 0.506 * relV + 2.46 * relVSquared) /
        (1 - 1.13 * relV + 12.8 * relVSquared - 1.64 * relVCubed);

    if (gDebug == MAMuscleExtendedDebug)
    {
        if (DebugFilters("GetMetabolicPower", m_Name))
        {
            *gDebugStream << "MAMuscle::GetMetabolicPower " << m_Name <<
            " m_Act " << m_Act <<
            " f0 " << f0 <<
            " vmax " << vmax <<
            " m_Velocity " << m_Strap->GetVelocity() <<
            " sigma " << sigma <<
            " power " << m_Act * f0 * vmax * sigma << "\n";
        }
    }
    return (m_Act * f0 * vmax * sigma);
}

void MAMuscleExtended::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (m_Name.size() == 0) std::cerr << "NamedObject::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tstim\tact\tspe\tepe\tsse\tese\tk\tvmax\tf0\tfce\tlpe\tfpe\tlse\tfse\tvce\tvse\tense\tenpe\tpse\tppe\tpce\ttension\tlength\tvelocity\tPMECH\tPMET\n";
        }
    }


    if (m_DumpStream)
    {
        *m_DumpStream << gSimulation->GetTime() << "\t" <<m_Stim << "\t" << m_Act << "\t" << spe << "\t" <<
                epe << "\t" << sse << "\t" << ese << "\t" << k << "\t" << vmax << "\t" << f0 << "\t" <<
                fce << "\t" << lpe << "\t" << fpe << "\t" << lse << "\t" << fse << "\t" << vce << "\t" <<
                GetVSE() << "\t" << GetESE() << "\t" << GetEPE() << "\t" << GetPSE() << "\t" << GetPPE() << "\t" << GetPCE() <<
                "\t" << m_Strap->GetTension() << "\t" << m_Strap->GetLength() << "\t" << m_Strap->GetVelocity() <<
                "\t" << m_Strap->GetVelocity() * m_Strap->GetTension() << "\t" << GetMetabolicPower() <<
                "\n";
    }
}

