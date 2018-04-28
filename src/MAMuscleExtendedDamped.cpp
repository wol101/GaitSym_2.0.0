/*
 *  MAMuscleExtendedDamped.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 14/03/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

// MAMuscleExtendedDamped - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a parallel spring element with damping

#include <ode/ode.h>

#include "SimpleStrap.h"
#include "MAMuscleExtendedDamped.h"
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

MAMuscleExtendedDamped::MAMuscleExtendedDamped(Strap *strap): Muscle(strap)
{
    m_Stim = 0;
    m_Act = 0;

    m_ActivationKinetics = true;

    spe = 0; // slack length parallel element (m)
    epe = 0; // elastic constant parallel element (N/m)
    sse = 0; // slack length serial element (m)
    ese = 0; // elastic constant serial element (N/m)

    dpe = 0; // parallel damping coefficient (Ns/m)
    dse = 0; // serial damping coefficient (Ns/m)


    k = 0; // shape constant
    vmax = 0; // maximum shortening velocity (m/s)
    f0 = 0; // isometric force (N)
}

// destructor
MAMuscleExtendedDamped::~MAMuscleExtendedDamped()
{
}

// set the muscle elastic properties
// if tendon length is set to < 0 then set it to a default slack value (doen later though)
void MAMuscleExtendedDamped::SetSerialElasticProperties(dReal springConstant, dReal dampingConstant, dReal unloadedLength)
{
    sse = unloadedLength; // slack length serial element (m)
        ese = springConstant; // elastic constant serial element (N/m)
    dse = dampingConstant; // serial damping coefficient (Ns/m)
}

void MAMuscleExtendedDamped::SetParallelElasticProperties(dReal springConstant, dReal dampingConstant, dReal unloadedLength)
{
        spe = unloadedLength; // slack length parallel element (m)
        epe = springConstant; // elastic constant parallel element (N/m)
    dpe = dampingConstant; // parallel damping coefficient (Ns/m)
}

// set the muscle contractile properties
void MAMuscleExtendedDamped::SetMuscleProperties(dReal vMax, dReal F0, dReal K)
{
    k = K; // shape constant
        vmax = vMax; // maximum shortening velocity (m/s)
    f0 = F0; // isometric force
}

// set the proportion of muscle fibres that are active
// calculates the tension in the strap

void MAMuscleExtendedDamped::SetActivation(dReal activation, dReal timeIncrement)
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
    dReal vel = m_Strap->GetVelocity();

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
        Rule(fpe,(Power(epe,2)*timeIncrement*(lastlpe - spe + k*timeIncrement*vmax) +
                  dpe*(-(ese*(lastlpe - len + sse)) + dse*vel + k*(alpha*f0 + (dpe + dse + ese*timeIncrement)*vmax)) +
                  epe*(alpha*f0*k*timeIncrement + ese*timeIncrement*(lastlpe + len - 2*spe - sse + k*timeIncrement*vmax) + dpe*(lastlpe - spe + 2*k*timeIncrement*vmax) +
                       dse*(2*lastlpe - 2*spe + timeIncrement*(vel + k*vmax))) -
                  dpe*Sqrt(4*k*(dpe + dse + (epe + ese)*timeIncrement)*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse) - dse*vel)*vmax +
                           Power(epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + dse*vel + k*(alpha*f0 + (dpe + dse + (epe + ese)*timeIncrement)*vmax),2)) -
                  epe*timeIncrement*Sqrt(4*k*(dpe + dse + (epe + ese)*timeIncrement)*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse) - dse*vel)*vmax +
                                         Power(epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + dse*vel + k*(alpha*f0 + (dpe + dse + (epe + ese)*timeIncrement)*vmax),2)))/
             (2.*(dpe + dse + (epe + ese)*timeIncrement)));

        Rule(fse,(-(alpha*f0*k*(dse + ese*timeIncrement)) + Power(dse,2)*(vel - k*vmax) +
                  dse*(2*dpe*vel - dpe*k*vmax + epe*(lastlpe - spe + 2*timeIncrement*vel - k*timeIncrement*vmax) -
                       ese*(lastlpe - len + sse - timeIncrement*vel + 2*k*timeIncrement*vmax) +
                       Sqrt(Power(alpha,2)*Power(f0,2)*Power(k,2) + Power(epe*lastlpe + ese*lastlpe - ese*len - epe*spe + ese*sse - dse*vel +
                                                                          k*(dpe + dse + (epe + ese)*timeIncrement)*vmax,2) +
                            2*alpha*f0*k*(-(epe*lastlpe) - ese*lastlpe + ese*len + epe*spe - ese*sse + dse*vel + (2 + k)*(dpe + dse + (epe + ese)*timeIncrement)*vmax))) +
                  ese*(-(dpe*(2*(lastlpe - len + sse) + k*timeIncrement*vmax)) -
                       timeIncrement*(ese*(lastlpe - len + sse + k*timeIncrement*vmax) + epe*(lastlpe - 2*len + spe + 2*sse + k*timeIncrement*vmax)) +
                       timeIncrement*Sqrt(Power(alpha,2)*Power(f0,2)*Power(k,2) +
                                          Power(epe*lastlpe + ese*lastlpe - ese*len - epe*spe + ese*sse - dse*vel + k*(dpe + dse + (epe + ese)*timeIncrement)*vmax,2) +
                                          2*alpha*f0*k*(-(epe*lastlpe) - ese*lastlpe + ese*len + epe*spe - ese*sse + dse*vel + (2 + k)*(dpe + dse + (epe + ese)*timeIncrement)*vmax))))/
             (2.*(dpe + dse + (epe + ese)*timeIncrement)));

        Rule(lse,(-2*lastlpe + 2*len -
                  (timeIncrement*(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + dse*vel))/(dpe + dse + (epe + ese)*timeIncrement) -
                  k*timeIncrement*vmax + (timeIncrement*Sqrt(Power(alpha,2)*Power(f0,2)*Power(k,2) +
                                                             Power(epe*lastlpe + ese*lastlpe - ese*len - epe*spe + ese*sse - dse*vel + k*(dpe + dse + (epe + ese)*timeIncrement)*vmax,2) +
                                                             2*alpha*f0*k*(-(epe*lastlpe) - ese*lastlpe + ese*len + epe*spe - ese*sse + dse*vel + (2 + k)*(dpe + dse + (epe + ese)*timeIncrement)*vmax)))/
                  (dpe + dse + (epe + ese)*timeIncrement))/2.);

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 1;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // Solution 2
        Rule(fpe,
             (Power(epe,2)*timeIncrement*(lastlpe - spe + k*timeIncrement*vmax) +
              dpe*(-(ese*(lastlpe - len + sse)) + dse*vel + k*(alpha*f0 + (dpe + dse + ese*timeIncrement)*vmax)) +
              epe*(alpha*f0*k*timeIncrement + ese*timeIncrement*(lastlpe + len - 2*spe - sse + k*timeIncrement*vmax) + dpe*(lastlpe - spe + 2*k*timeIncrement*vmax) +
                   dse*(2*lastlpe - 2*spe + timeIncrement*(vel + k*vmax))) +
              dpe*Sqrt(4*k*(dpe + dse + (epe + ese)*timeIncrement)*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse) - dse*vel)*vmax +
                       Power(epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + dse*vel + k*(alpha*f0 + (dpe + dse + (epe + ese)*timeIncrement)*vmax),2)) +
              epe*timeIncrement*Sqrt(4*k*(dpe + dse + (epe + ese)*timeIncrement)*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse) - dse*vel)*vmax +
                                     Power(epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + dse*vel + k*(alpha*f0 + (dpe + dse + (epe + ese)*timeIncrement)*vmax),2)))/
             (2.*(dpe + dse + (epe + ese)*timeIncrement)));

        Rule(fse,-(alpha*f0*k*(dse + ese*timeIncrement) + Power(dse,2)*(-vel + k*vmax) +
                   dse*(-2*dpe*vel + dpe*k*vmax + epe*(-lastlpe + spe - 2*timeIncrement*vel + k*timeIncrement*vmax) +
                        ese*(lastlpe - len + sse - timeIncrement*vel + 2*k*timeIncrement*vmax) +
                        Sqrt(Power(alpha,2)*Power(f0,2)*Power(k,2) + Power(epe*lastlpe + ese*lastlpe - ese*len - epe*spe + ese*sse - dse*vel +
                                                                           k*(dpe + dse + (epe + ese)*timeIncrement)*vmax,2) +
                             2*alpha*f0*k*(-(epe*lastlpe) - ese*lastlpe + ese*len + epe*spe - ese*sse + dse*vel + (2 + k)*(dpe + dse + (epe + ese)*timeIncrement)*vmax))) +
                   ese*(2*dpe*(lastlpe - len + sse) + dpe*k*timeIncrement*vmax +
                        timeIncrement*(ese*(lastlpe - len + sse + k*timeIncrement*vmax) + epe*(lastlpe - 2*len + spe + 2*sse + k*timeIncrement*vmax) +
                                       Sqrt(Power(alpha,2)*Power(f0,2)*Power(k,2) + Power(epe*lastlpe + ese*lastlpe - ese*len - epe*spe + ese*sse - dse*vel +
                                                                                          k*(dpe + dse + (epe + ese)*timeIncrement)*vmax,2) +
                                            2*alpha*f0*k*(-(epe*lastlpe) - ese*lastlpe + ese*len + epe*spe - ese*sse + dse*vel + (2 + k)*(dpe + dse + (epe + ese)*timeIncrement)*vmax)))))/
             (2.*(dpe + dse + (epe + ese)*timeIncrement)));

        Rule(lse,-(dpe*(2*lastlpe - 2*len + k*timeIncrement*vmax) +
                   dse*(2*lastlpe - 2*len + timeIncrement*(vel + k*vmax)) +
                   timeIncrement*(epe*(lastlpe - 2*len + spe) + ese*(lastlpe - len - sse) + k*(alpha*f0 + (epe + ese)*timeIncrement*vmax) +
                                  Sqrt(Power(alpha,2)*Power(f0,2)*Power(k,2) + Power(epe*lastlpe + ese*lastlpe - ese*len - epe*spe + ese*sse - dse*vel +
                                                                                     k*(dpe + dse + (epe + ese)*timeIncrement)*vmax,2) +
                                       2*alpha*f0*k*(-(epe*lastlpe) - ese*lastlpe + ese*len + epe*spe - ese*sse + dse*vel + (2 + k)*(dpe + dse + (epe + ese)*timeIncrement)*vmax))))/
             (2.*(dpe + dse + (epe + ese)*timeIncrement)));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 2;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // Now try assuming vce > 0 (eccentric)

        // Solution 1

        Rule(fpe,(alpha*f0*(-850.5 - 50.*k)*(dpe + epe*timeIncrement) +
                  Power(epe,2)*timeIncrement*(472.5*lastlpe - 472.5*spe - 62.50000000000001*k*timeIncrement*vmax) +
                  epe*(dpe*(472.5*lastlpe - 472.5*spe - 125.00000000000001*k*timeIncrement*vmax) +
                       ese*timeIncrement*(472.5*lastlpe + 472.5*len - 945.*spe - 472.5*sse - 62.50000000000001*k*timeIncrement*vmax) +
                       dse*(945.*lastlpe - 945.*spe + 472.5*timeIncrement*vel - 62.50000000000001*k*timeIncrement*vmax)) +
                  dpe*(472.5000000000001*dse*vel - 62.5*dpe*k*vmax - 62.5*dse*k*vmax +
                       ese*(-472.5000000000001*lastlpe + 472.5000000000001*len - 472.5000000000001*sse - 62.5*k*timeIncrement*vmax)) -
                  0.5*dpe*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) + 15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) +
                               dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) + Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                               Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                             236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                             15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                               Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                             236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                             lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                               ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                                    dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                         236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                               epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                                    dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                                    ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.78605e6*len - 1.78605e6*spe + 1.78605e6*sse - 472500.*k*timeIncrement*vmax) +
                                         k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) + len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) +
                                         spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                               alpha*f0*(dpe*k*(-47250. + 24999.999999999993*k)*vmax +
                                         dse*((-3.2148900000000014e6 - 189000.00000000003*k)*vel + 24999.999999999993*(-1.8900000000000006 + k)*k*vmax) +
                                         epe*((3.2148900000000023e6 + 189000.00000000003*k)*lastlpe + (-3.2148900000000023e6 - 189000.00000000003*k)*spe +
                                              k*(-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax) +
                                         ese*((3.2148900000000023e6 + 189000.00000000003*k)*lastlpe - 3.2148900000000014e6*len + 3.2148900000000014e6*sse +
                                              k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax)))) -
                  0.5*epe*timeIncrement*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                             15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) + dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) +
                                             Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                                             Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                                           236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                                           15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                                             Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                                           236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                                           lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                                             ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                                                  dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                                       236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                                             epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                                                  dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                                                  ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.78605e6*len - 1.78605e6*spe + 1.78605e6*sse - 472500.*k*timeIncrement*vmax) +
                                                       k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) + len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) +
                                                       spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                                             alpha*f0*(dpe*k*(-47250. + 24999.999999999993*k)*vmax +
                                                       dse*((-3.2148900000000014e6 - 189000.00000000003*k)*vel + 24999.999999999993*(-1.8900000000000006 + k)*k*vmax) +
                                                       epe*((3.2148900000000023e6 + 189000.00000000003*k)*lastlpe + (-3.2148900000000023e6 - 189000.00000000003*k)*spe +
                                                            k*(-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax) +
                                                       ese*((3.2148900000000023e6 + 189000.00000000003*k)*lastlpe - 3.2148900000000014e6*len + 3.2148900000000014e6*sse +
                                                            k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax)))))/
             (945.*dpe + 945.*dse + 945.*epe*timeIncrement + 945.*ese*timeIncrement));

        Rule(fse,(850.5*alpha*dse*f0 + 50.*alpha*dse*f0*k + 472.5*dse*epe*lastlpe - 945.*dpe*ese*lastlpe - 472.5*dse*ese*lastlpe + 945.*dpe*ese*len +
                  472.5*dse*ese*len - 472.5*dse*epe*spe - 945.*dpe*ese*sse - 472.5*dse*ese*sse + 850.5*alpha*ese*f0*timeIncrement + 50.*alpha*ese*f0*k*timeIncrement -
                  472.5*epe*ese*lastlpe*timeIncrement - 472.5*Power(ese,2)*lastlpe*timeIncrement + 945.*epe*ese*len*timeIncrement + 472.5*Power(ese,2)*len*timeIncrement -
                  472.5*epe*ese*spe*timeIncrement - 945.*epe*ese*sse*timeIncrement - 472.5*Power(ese,2)*sse*timeIncrement + 945.*dpe*dse*vel + 472.5*Power(dse,2)*vel +
                  945.*dse*epe*timeIncrement*vel + 472.5*dse*ese*timeIncrement*vel + 62.5*dpe*dse*k*vmax + 62.5*Power(dse,2)*k*vmax + 62.5*dse*epe*k*timeIncrement*vmax +
                  62.5*dpe*ese*k*timeIncrement*vmax + 125.*dse*ese*k*timeIncrement*vmax + 62.5*epe*ese*k*Power(timeIncrement,2)*vmax +
                  62.5*Power(ese,2)*k*Power(timeIncrement,2)*vmax + 0.5*dse*
                  Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) + 15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) +
                       dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) + Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                       Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                     236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                     15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                       Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                     236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                     lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                       ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                            dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                 236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                       epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                            dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                            ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.78605e6*len - 1.78605e6*spe + 1.78605e6*sse - 472500.*k*timeIncrement*vmax) +
                                 k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) + len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) +
                                 spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                       alpha*f0*(dpe*k*(-47250.00000000001 + 24999.999999999996*k)*vmax +
                                 dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                 epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                      k*(-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax) +
                                 ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                      k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax)))) +
                  0.5*ese*timeIncrement*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                             15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) + dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) +
                                             Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                                             Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                                           236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                                           15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                                             Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                                           236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                                           lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                                             ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                                                  dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                                       236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                                             epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                                                  dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                                                  ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.78605e6*len - 1.78605e6*spe + 1.78605e6*sse - 472500.*k*timeIncrement*vmax) +
                                                       k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) + len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) +
                                                       spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                                             alpha*f0*(dpe*k*(-47250.00000000001 + 24999.999999999996*k)*vmax +
                                                       dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                                       epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                            k*(-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax) +
                                                       ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                            k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax)))))/
             (945.*dpe + 945.*dse + 945.*epe*timeIncrement + 945.*ese*timeIncrement));

        Rule(lse,(-945.*dpe*lastlpe - 945.*dse*lastlpe + 945.*dpe*len + 945.*dse*len + alpha*f0*(850.5 + 50.*k)*timeIncrement - 472.5*epe*lastlpe*timeIncrement -
                  472.5*ese*lastlpe*timeIncrement + 945.*epe*len*timeIncrement + 472.5*ese*len*timeIncrement - 472.5*epe*spe*timeIncrement + 472.5*ese*sse*timeIncrement -
                  472.5*dse*timeIncrement*vel + 62.5*dpe*k*timeIncrement*vmax + 62.5*dse*k*timeIncrement*vmax + 62.5*epe*k*Power(timeIncrement,2)*vmax +
                  62.5*ese*k*Power(timeIncrement,2)*vmax + 0.5*timeIncrement*
                  Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) + 15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) +
                       dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) + Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                       Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                     236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                     15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                       Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                     236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                     lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                       ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                            dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                 236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                       epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                            dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                            ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.7860500000000012e6*len - 1.7860500000000012e6*spe + 1.7860500000000012e6*sse -
                                                                       472500.*k*timeIncrement*vmax) + k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) +
                                 len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) + spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                       alpha*f0*(dpe*k*(-47250.00000000001 + 24999.999999999996*k)*vmax +
                                 dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                 epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                      k*(-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax) +
                                 ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                      k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax)))))/
             (945.*dpe + 945.*dse + 945.*epe*timeIncrement + 945.*ese*timeIncrement));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 3;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // Solution 2

        Rule(fpe,(alpha*f0*(-850.5 - 50.*k)*(dpe + epe*timeIncrement) +
                  Power(epe,2)*timeIncrement*(472.5*lastlpe - 472.5*spe - 62.50000000000001*k*timeIncrement*vmax) +
                  epe*(dpe*(472.5*lastlpe - 472.5*spe - 125.00000000000001*k*timeIncrement*vmax) +
                       ese*timeIncrement*(472.5*lastlpe + 472.5*len - 945.*spe - 472.5*sse - 62.50000000000001*k*timeIncrement*vmax) +
                       dse*(945.*lastlpe - 945.*spe + 472.5*timeIncrement*vel - 62.50000000000001*k*timeIncrement*vmax)) +
                  dpe*(472.5000000000001*dse*vel - 62.5*dpe*k*vmax - 62.5*dse*k*vmax +
                       ese*(-472.5000000000001*lastlpe + 472.5000000000001*len - 472.5000000000001*sse - 62.5*k*timeIncrement*vmax)) +
                  0.5*dpe*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) + 15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) +
                               dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) + Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                               Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                             236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                             15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                               Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                             236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                             lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                               ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                                    dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                         236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                               epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                                    dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                                    ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.78605e6*len - 1.78605e6*spe + 1.78605e6*sse - 472500.*k*timeIncrement*vmax) +
                                         k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) + len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) +
                                         spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                               alpha*f0*(dpe*k*(-47250. + 24999.999999999993*k)*vmax +
                                         dse*((-3.2148900000000014e6 - 189000.00000000003*k)*vel + 24999.999999999993*(-1.8900000000000006 + k)*k*vmax) +
                                         epe*((3.2148900000000023e6 + 189000.00000000003*k)*lastlpe + (-3.2148900000000023e6 - 189000.00000000003*k)*spe +
                                              k*(-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax) +
                                         ese*((3.2148900000000023e6 + 189000.00000000003*k)*lastlpe - 3.2148900000000014e6*len + 3.2148900000000014e6*sse +
                                              k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax)))) +
                  0.5*epe*timeIncrement*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                             15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) + dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) +
                                             Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                                             Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                                           236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                                           15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                                             Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                                           236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                                           lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                                             ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                                                  dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                                       236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                                             epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                                                  dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                                                  ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.78605e6*len - 1.78605e6*spe + 1.78605e6*sse - 472500.*k*timeIncrement*vmax) +
                                                       k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) + len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) +
                                                       spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                                             alpha*f0*(dpe*k*(-47250. + 24999.999999999993*k)*vmax +
                                                       dse*((-3.2148900000000014e6 - 189000.00000000003*k)*vel + 24999.999999999993*(-1.8900000000000006 + k)*k*vmax) +
                                                       epe*((3.2148900000000023e6 + 189000.00000000003*k)*lastlpe + (-3.2148900000000023e6 - 189000.00000000003*k)*spe +
                                                            k*(-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax) +
                                                       ese*((3.2148900000000023e6 + 189000.00000000003*k)*lastlpe - 3.2148900000000014e6*len + 3.2148900000000014e6*sse +
                                                            k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax)))))/
             (945.*dpe + 945.*dse + 945.*epe*timeIncrement + 945.*ese*timeIncrement));

        Rule(fse,(850.5*alpha*dse*f0 + 50.*alpha*dse*f0*k + 472.5*dse*epe*lastlpe - 945.*dpe*ese*lastlpe - 472.5*dse*ese*lastlpe + 945.*dpe*ese*len +
                  472.5*dse*ese*len - 472.5*dse*epe*spe - 945.*dpe*ese*sse - 472.5*dse*ese*sse + 850.5*alpha*ese*f0*timeIncrement + 50.*alpha*ese*f0*k*timeIncrement -
                  472.5*epe*ese*lastlpe*timeIncrement - 472.5*Power(ese,2)*lastlpe*timeIncrement + 945.*epe*ese*len*timeIncrement + 472.5*Power(ese,2)*len*timeIncrement -
                  472.5*epe*ese*spe*timeIncrement - 945.*epe*ese*sse*timeIncrement - 472.5*Power(ese,2)*sse*timeIncrement + 945.*dpe*dse*vel + 472.5*Power(dse,2)*vel +
                  945.*dse*epe*timeIncrement*vel + 472.5*dse*ese*timeIncrement*vel + 62.5*dpe*dse*k*vmax + 62.5*Power(dse,2)*k*vmax + 62.5*dse*epe*k*timeIncrement*vmax +
                  62.5*dpe*ese*k*timeIncrement*vmax + 125.*dse*ese*k*timeIncrement*vmax + 62.5*epe*ese*k*Power(timeIncrement,2)*vmax +
                  62.5*Power(ese,2)*k*Power(timeIncrement,2)*vmax - 0.5*dse*
                  Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) + 15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) +
                       dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) + Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                       Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                     236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                     15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                       Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                     236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                     lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                       ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                            dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                 236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                       epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                            dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                            ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.78605e6*len - 1.78605e6*spe + 1.78605e6*sse - 472500.*k*timeIncrement*vmax) +
                                 k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) + len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) +
                                 spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                       alpha*f0*(dpe*k*(-47250.00000000001 + 24999.999999999996*k)*vmax +
                                 dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                 epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                      k*(-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax) +
                                 ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                      k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax)))) -
                  0.5*ese*timeIncrement*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                             15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) + dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) +
                                             Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                                             Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                                           236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                                           15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                                             Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                                           236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                                           lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                                             ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                                                  dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                                       236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                                             epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                                                  dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                                                  ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.78605e6*len - 1.78605e6*spe + 1.78605e6*sse - 472500.*k*timeIncrement*vmax) +
                                                       k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) + len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) +
                                                       spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                                             alpha*f0*(dpe*k*(-47250.00000000001 + 24999.999999999996*k)*vmax +
                                                       dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                                       epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                            k*(-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax) +
                                                       ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                            k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax)))))/
             (945.*dpe + 945.*dse + 945.*epe*timeIncrement + 945.*ese*timeIncrement));

        Rule(lse,(-945.*dpe*lastlpe - 945.*dse*lastlpe + 945.*dpe*len + 945.*dse*len + alpha*f0*(850.5 + 50.*k)*timeIncrement - 472.5*epe*lastlpe*timeIncrement -
                  472.5*ese*lastlpe*timeIncrement + 945.*epe*len*timeIncrement + 472.5*ese*len*timeIncrement - 472.5*epe*spe*timeIncrement + 472.5*ese*sse*timeIncrement -
                  472.5*dse*timeIncrement*vel + 62.5*dpe*k*timeIncrement*vmax + 62.5*dse*k*timeIncrement*vmax + 62.5*epe*k*Power(timeIncrement,2)*vmax +
                  62.5*ese*k*Power(timeIncrement,2)*vmax - 0.5*timeIncrement*
                  Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) + 15625.*Power(dpe,2)*Power(k,2)*Power(vmax,2) +
                       dpe*dse*k*vmax*(236250.*vel + 31250.*k*vmax) + Power(dse,2)*(893025.*Power(vel,2) + 236250.*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                       Power(epe,2)*(893025.0000000002*Power(lastlpe,2) - 1.7860500000000005e6*lastlpe*spe + 893025.0000000002*Power(spe,2) -
                                     236250.00000000003*k*lastlpe*timeIncrement*vmax + 236250.00000000003*k*spe*timeIncrement*vmax +
                                     15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2)) +
                       Power(ese,2)*(893025.*Power(lastlpe,2) + 893025.*Power(len,2) - 1.78605e6*len*sse + 893025.*Power(sse,2) + 236250.*k*len*timeIncrement*vmax -
                                     236250.*k*sse*timeIncrement*vmax + 15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                     lastlpe*(-1.78605e6*len + 1.78605e6*sse - 236250.*k*timeIncrement*vmax)) +
                       ese*(dpe*k*vmax*(-236250.*lastlpe + 236250.*len - 236250.*sse + 31250.*k*timeIncrement*vmax) +
                            dse*(-1.78605e6*lastlpe*vel + 1.78605e6*len*vel - 1.78605e6*sse*vel - 236250.*k*lastlpe*vmax + 236250.*k*len*vmax - 236250.*k*sse*vmax +
                                 236250.*k*timeIncrement*vel*vmax + 31250.*Power(k,2)*timeIncrement*Power(vmax,2))) +
                       epe*(dpe*k*vmax*(-236250.*lastlpe + 236250.*spe + 31250.*k*timeIncrement*vmax) +
                            dse*(lastlpe*(-1.78605e6*vel - 236250.*k*vmax) + k*timeIncrement*vmax*(236250.*vel + 31250.*k*vmax) + spe*(1.78605e6*vel + 236250.*k*vmax)) +
                            ese*(1.78605e6*Power(lastlpe,2) + lastlpe*(-1.7860500000000012e6*len - 1.7860500000000012e6*spe + 1.7860500000000012e6*sse -
                                                                       472500.*k*timeIncrement*vmax) + k*timeIncrement*vmax*(-236250.*sse + 31250.*k*timeIncrement*vmax) +
                                 len*(1.78605e6*spe + 236250.*k*timeIncrement*vmax) + spe*(-1.78605e6*sse + 236250.*k*timeIncrement*vmax))) +
                       alpha*f0*(dpe*k*(-47250.00000000001 + 24999.999999999996*k)*vmax +
                                 dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                 epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                      k*(-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax) +
                                 ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                      k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax)))))/
             (945.*dpe + 945.*dse + 945.*epe*timeIncrement + 945.*ese*timeIncrement));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 4;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // now consider special case if (lpe < spe) // pe slack special case

        // First assume vce <= 0 (concentric)
        // Solution 1

        Rule(lpe,(dse*(2*lastlpe + timeIncrement*(vel + k*vmax)) + timeIncrement*(alpha*f0*k + ese*(lastlpe + len - sse + k*timeIncrement*vmax)) -
                  timeIncrement*Sqrt(4*k*(dse + ese*timeIncrement)*(alpha*f0 + ese*(lastlpe - len + sse) - dse*vel)*vmax +
                                     Power(alpha*f0*k + dse*(vel + k*vmax) + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/(2.*(dse + ese*timeIncrement)));

        Rule(fpe,0);

        Rule(fse,(-(alpha*f0*k) - ese*lastlpe + ese*len - ese*sse + dse*vel - dse*k*vmax - ese*k*timeIncrement*vmax +
                  Sqrt(4*k*(dse + ese*timeIncrement)*(alpha*f0 + ese*(lastlpe - len + sse) - dse*vel)*vmax +
                       Power(alpha*f0*k + dse*(vel + k*vmax) + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/2.);

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 5;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

         // Solution 2

        Rule(lpe,(dse*(2*lastlpe + timeIncrement*(vel + k*vmax)) + timeIncrement*(alpha*f0*k + ese*(lastlpe + len - sse + k*timeIncrement*vmax)) +
                  timeIncrement*Sqrt(4*k*(dse + ese*timeIncrement)*(alpha*f0 + ese*(lastlpe - len + sse) - dse*vel)*vmax +
                                     Power(alpha*f0*k + dse*(vel + k*vmax) + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/(2.*(dse + ese*timeIncrement)));

        Rule(fpe,0);

        Rule(fse,(-(alpha*f0*k) - ese*lastlpe + ese*len - ese*sse + dse*vel - dse*k*vmax - ese*k*timeIncrement*vmax -
                  Sqrt(4*k*(dse + ese*timeIncrement)*(alpha*f0 + ese*(lastlpe - len + sse) - dse*vel)*vmax +
                       Power(alpha*f0*k + dse*(vel + k*vmax) + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/2.);

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 6;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // Now check if (vce > 0) (eccentric)

        // Solution 1

        Rule(lpe,(dse*(945.*lastlpe + 472.5*timeIncrement*vel - 62.5*k*timeIncrement*vmax) +
                  timeIncrement*(alpha*f0*(-850.5 - 50.*k) + ese*(472.5*lastlpe + 472.5*len - 472.5*sse - 62.5*k*timeIncrement*vmax) -
                                 0.5*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                          Power(dse,2)*(893025.0000000001*Power(vel,2) + 236250.00000000003*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                                          dse*ese*(-1.7860500000000002e6*lastlpe*vel + 1.7860500000000002e6*len*vel - 1.7860500000000002e6*sse*vel - 236250.00000000003*k*lastlpe*vmax +
                                                   236250.00000000003*k*len*vmax - 236250.00000000003*k*sse*vmax + 236250.00000000003*k*timeIncrement*vel*vmax +
                                                   31250.*Power(k,2)*timeIncrement*Power(vmax,2)) +
                                          Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse + 893025.0000000005*Power(sse,2) +
                                                        236250.00000000003*k*len*timeIncrement*vmax - 236250.00000000003*k*sse*timeIncrement*vmax +
                                                        15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                                        lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000003*k*timeIncrement*vmax)) +
                                          alpha*f0*(dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                                    ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                         k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax))))))/
             (945.*dse + 945.*ese*timeIncrement));

        Rule(fpe,0.);

        Rule(fse,
             alpha*f0*(0.9 + 0.052910052910052914*k) + 0.5000000000000001*dse*vel + 0.06613756613756616*dse*k*vmax +
             ese*(-0.5000000000000001*lastlpe + 0.5000000000000001*len - 0.5000000000000001*sse + 0.06613756613756613*k*timeIncrement*vmax) +
             0.0005291005291005291*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                        Power(dse,2)*(893025.0000000001*Power(vel,2) + 236250.00000000003*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                                        dse*ese*(-1.7860500000000002e6*lastlpe*vel + 1.7860500000000002e6*len*vel - 1.7860500000000002e6*sse*vel - 236250.00000000003*k*lastlpe*vmax +
                                                 236250.00000000003*k*len*vmax - 236250.00000000003*k*sse*vmax + 236250.00000000003*k*timeIncrement*vel*vmax +
                                                 31250.*Power(k,2)*timeIncrement*Power(vmax,2)) + Power(ese,2)*
                                        (893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse + 893025.0000000005*Power(sse,2) +
                                         236250.00000000003*k*len*timeIncrement*vmax - 236250.00000000003*k*sse*timeIncrement*vmax +
                                         15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                         lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000003*k*timeIncrement*vmax)) +
                                        alpha*f0*(dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                                  ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                       k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax)))));

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 7;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // Solution 2

        Rule(lpe,(dse*(945.*lastlpe + 472.5*timeIncrement*vel - 62.5*k*timeIncrement*vmax) +
                  timeIncrement*(alpha*f0*(-850.5 - 50.*k) + ese*(472.5*lastlpe + 472.5*len - 472.5*sse - 62.5*k*timeIncrement*vmax) +
                                 0.5*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                          Power(dse,2)*(893025.0000000001*Power(vel,2) + 236250.00000000003*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                                          dse*ese*(-1.7860500000000002e6*lastlpe*vel + 1.7860500000000002e6*len*vel - 1.7860500000000002e6*sse*vel - 236250.00000000003*k*lastlpe*vmax +
                                                   236250.00000000003*k*len*vmax - 236250.00000000003*k*sse*vmax + 236250.00000000003*k*timeIncrement*vel*vmax +
                                                   31250.*Power(k,2)*timeIncrement*Power(vmax,2)) +
                                          Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse + 893025.0000000005*Power(sse,2) +
                                                        236250.00000000003*k*len*timeIncrement*vmax - 236250.00000000003*k*sse*timeIncrement*vmax +
                                                        15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                                        lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000003*k*timeIncrement*vmax)) +
                                          alpha*f0*(dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                                    ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                         k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax))))))/
             (945.*dse + 945.*ese*timeIncrement));

        Rule(fpe,0.);

        Rule(fse,
             alpha*f0*(0.9 + 0.052910052910052914*k) + 0.5000000000000001*dse*vel + 0.06613756613756616*dse*k*vmax +
             ese*(-0.5000000000000001*lastlpe + 0.5000000000000001*len - 0.5000000000000001*sse + 0.06613756613756613*k*timeIncrement*vmax) -
             0.0005291005291005291*Sqrt(10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                        Power(dse,2)*(893025.0000000001*Power(vel,2) + 236250.00000000003*k*vel*vmax + 15625.*Power(k,2)*Power(vmax,2)) +
                                        dse*ese*(-1.7860500000000002e6*lastlpe*vel + 1.7860500000000002e6*len*vel - 1.7860500000000002e6*sse*vel - 236250.00000000003*k*lastlpe*vmax +
                                                 236250.00000000003*k*len*vmax - 236250.00000000003*k*sse*vmax + 236250.00000000003*k*timeIncrement*vel*vmax +
                                                 31250.*Power(k,2)*timeIncrement*Power(vmax,2)) + Power(ese,2)*
                                        (893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse + 893025.0000000005*Power(sse,2) +
                                         236250.00000000003*k*len*timeIncrement*vmax - 236250.00000000003*k*sse*timeIncrement*vmax +
                                         15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                         lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000003*k*timeIncrement*vmax)) +
                                        alpha*f0*(dse*((-3.214890000000001e6 - 189000.00000000003*k)*vel + 24999.999999999996*(-1.8900000000000006 + k)*k*vmax) +
                                                  ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                       k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999996*k)*timeIncrement*vmax)))));

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
        }
        else
        {
            progress = 10;
            lpe = (len - sse + (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
            fse = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));
            fpe = (0);
        }
        lse = len - lpe;
        fce = fse - fpe;
        if (fce < 0) fce = 0; // sanity check
        if (fse < 0) fse = 0; // sanity check
    }

    lastlpe = lpe;
    m_Strap->SetTension(fse);

    if (gDebug == MAMuscleExtendedDampedDebug)
    {
        if (DebugFilters("SetActivation", m_Name))
        {
            static int firstTime = true;
            if (firstTime) // only happens once
            {
                firstTime = false;
                *gDebugStream << "MAMuscleExtendedDamped::SetActivation " <<
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


// calculate the metabolic power of the muscle

dReal MAMuscleExtendedDamped::GetMetabolicPower()
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

    if (gDebug == MAMuscleExtendedDampedDebug)
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

void MAMuscleExtendedDamped::Dump()
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


