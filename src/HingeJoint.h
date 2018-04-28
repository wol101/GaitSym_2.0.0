/*
 *  HingeJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef HingeJoint_h
#define HingeJoint_h

#include "Joint.h"

class HingeJoint: public Joint
{
public:

    HingeJoint(dWorldID worldID);

    void SetHingeAnchor (dReal x, dReal y, dReal z);
    void SetHingeAxis(dReal x, dReal y, dReal z);
    void SetHingeAnchor (const char *buf);
    void SetHingeAxis(const char *buf);

    void SetStartAngleReference(dReal startAngleReference);
    void SetJointStops(dReal loStop, dReal hiStop);
    void SetStopCFM(dReal cfm);
    void SetStopERP(dReal erp);
    void SetStopBounce(dReal bounce);
    void SetStopSpringDamp(dReal springConstant, dReal dampingConstant, dReal integrationStep);
    void SetStopSpringERP(dReal springConstant, dReal ERP, dReal integrationStep);

    void GetHingeAnchor(dVector3 result);
    void GetHingeAnchor2(dVector3 result);
    void GetHingeAxis(dVector3 result);

    dReal GetHingeAngle();
    dReal GetHingeAngleRate();
    dReal GetStopTorque();

    void SetTorqueLimits(dReal loStopTorqueLimit, dReal hiStopTorqueLimit)
    {
        m_LoStopTorqueLimit = loStopTorqueLimit;
        m_HiStopTorqueLimit = hiStopTorqueLimit;
    }
    int TestLimits();

    virtual void Dump();

#ifdef USE_OPENGL
    virtual void Draw();
#endif

protected:

    dReal m_StartAngleReference;

    dReal m_HiStopTorqueLimit;
    dReal m_LoStopTorqueLimit;
};



#endif
