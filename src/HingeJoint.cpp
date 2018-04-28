/*
 *  HingeJoint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <iostream>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif

#include "HingeJoint.h"
#include "DataFile.h"
#include "Body.h"
#include "Simulation.h"
#include "DebugControl.h"

#ifdef USE_OPENGL
#include "GLUtils.h"
extern int gAxisFlag;
extern std::string gOBJName;
#endif

// Simulation global
extern Simulation *gSimulation;


HingeJoint::HingeJoint(dWorldID worldID)
{
    m_JointID = dJointCreateHinge(worldID, 0);
    dJointSetData(m_JointID, this);

    dJointSetFeedback(m_JointID, &m_JointFeedback);
    m_StartAngleReference = 0;
    m_HiStopTorqueLimit = dInfinity;
    m_LoStopTorqueLimit = -dInfinity;
}

void HingeJoint::SetHingeAnchor (dReal x, dReal y, dReal z)
{
    dJointSetHingeAnchor(m_JointID, x, y, z);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void HingeJoint::SetHingeAnchor(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 3)
    {
        std::cerr << "Error in HingeJoint::SetHingeAnchor\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dJointSetHingeAnchor(m_JointID, pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in HingeJoint::SetHingeAnchor\n";
        return; // error condition
    }
    Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dJointSetHingeAnchor(m_JointID, pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in HingeJoint::SetHingeAnchor\n";
            return; // error condition
        }
    }
    dVector3 result;
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos (theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    dJointSetHingeAnchor(m_JointID, result[0], result[1], result[2]);
}

void HingeJoint::SetHingeAxis(dReal x, dReal y, dReal z)
{
    dVector3 v;
    v[0] = x; v[1] = y; v[2] = z;
    dNormalize3(v);
    dJointSetHingeAxis(m_JointID, v[0], v[1], v[2]);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void HingeJoint::SetHingeAxis(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos, result;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 3)
    {
        std::cerr << "Error in HingeJoint::SetHingeAxis\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        SetHingeAxis(pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in HingeJoint::SetHingeAxis\n";
        return; // error condition
    }
    Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            SetHingeAxis(pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in HingeJoint::SetHingeAxis\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyVectorToWorld(theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    SetHingeAxis(result[0], result[1], result[2]);
}

void HingeJoint::GetHingeAnchor(dVector3 result)
{
    dJointGetHingeAnchor(m_JointID, result);
}

void HingeJoint::GetHingeAnchor2(dVector3 result)
{
    dJointGetHingeAnchor2(m_JointID, result);
}

void HingeJoint::GetHingeAxis(dVector3 result)
{
    dJointGetHingeAxis(m_JointID, result);
}

dReal HingeJoint::GetHingeAngle()
{
    return dJointGetHingeAngle(m_JointID) + m_StartAngleReference;
}

dReal HingeJoint::GetHingeAngleRate()
{
    return dJointGetHingeAngleRate(m_JointID);
}

void HingeJoint::SetStartAngleReference(dReal startAngleReference)
{
    m_StartAngleReference = startAngleReference;
}

void HingeJoint::SetJointStops(dReal loStop, dReal hiStop)
{
    if (loStop >= hiStop) throw(__LINE__);

    // correct for m_StartAngleReference
    loStop -= m_StartAngleReference;
    hiStop -= m_StartAngleReference;

    if (loStop < -M_PI) loStop = -dInfinity;
    if (hiStop > M_PI) hiStop = dInfinity;

    // note there is safety feature that stops setting incompatible low and high
    // stops which can cause difficulties. The safe option is to set them twice.

    dJointSetHingeParam (m_JointID, dParamLoStop, loStop);
    dJointSetHingeParam (m_JointID, dParamHiStop, hiStop);
    dJointSetHingeParam (m_JointID, dParamLoStop, loStop);
    dJointSetHingeParam (m_JointID, dParamHiStop, hiStop);
}

// this is the approximate stop torque from the amount past the limit that the joint is
// it doesn't take into account damping and probably isn't actualy the value used anyway
dReal HingeJoint::GetStopTorque()
{
    dReal angle = dJointGetHingeAngle(m_JointID);
    dReal loStop = dJointGetHingeParam (m_JointID, dParamLoStop);
    dReal hiStop = dJointGetHingeParam (m_JointID, dParamHiStop);
    dReal t = 0;
    dReal ERP, CFM;
    dReal kp = 0;
    // dReal kd = 0; // decided not to use damping in these calculations
    if (angle > hiStop)
    {
        ERP = dJointGetHingeParam (m_JointID, dParamStopERP);
        CFM = dJointGetHingeParam (m_JointID, dParamStopCFM);
        kp = ERP / (CFM * gSimulation->GetTimeIncrement());
        //kd = (1 - ERP) / CFM;
        //t = (hiStop - angle) * kp - GetHingeAngleRate() * kd;
        t = (hiStop - angle) * kp;
    }
    else if (angle < loStop)
    {
        ERP = dJointGetHingeParam (m_JointID, dParamStopERP);
        CFM = dJointGetHingeParam (m_JointID, dParamStopCFM);
        kp = ERP / (CFM * gSimulation->GetTimeIncrement());
        //kd = (1 - ERP) / CFM;
        //t = (loStop - angle) * kp - GetHingeAngleRate() * kd;
        t = (loStop - angle) * kp;
    }

    if (gDebug == HingeJointDebug)
    {
        if (DebugFilters("GetStopTorque", m_Name))
            // std::cerr << m_Name << " angle " << GetHingeAngle() << " dangle " << GetHingeAngleRate() << " kp " << kp << " kd " << kd << " t " << t << "\n";
            std::cerr << m_Name << " loStop " <<  loStop + m_StartAngleReference <<  " hiStop " <<  hiStop + m_StartAngleReference <<  " angle " << GetHingeAngle() << " kp " << kp << " t " << t << "\n";
    }

    return t;
}

int HingeJoint::TestLimits()
{
    dReal t = GetStopTorque();
    if (t < m_LoStopTorqueLimit) return -1;
    if (t > m_HiStopTorqueLimit) return 1;
    return 0;
}

void HingeJoint::SetStopCFM(dReal cfm)
{
    dJointSetHingeParam (m_JointID, dParamStopCFM, cfm);
}

void HingeJoint::SetStopERP(dReal erp)
{
    dJointSetHingeParam (m_JointID, dParamStopERP, erp);
}

void HingeJoint::SetStopSpringDamp(dReal springConstant, dReal dampingConstant, dReal integrationStep)
{
    dReal ERP = integrationStep * springConstant/(integrationStep * springConstant + dampingConstant);
    dReal CFM = 1/(integrationStep * springConstant + dampingConstant);
    SetStopERP(ERP);
    SetStopCFM(CFM);
}

void HingeJoint::SetStopSpringERP(dReal springConstant, dReal ERP, dReal integrationStep)
{
    dReal CFM = ERP / (integrationStep * springConstant);
    SetStopERP(ERP);
    SetStopCFM(CFM);
}

void HingeJoint::SetStopBounce(dReal bounce)
{
    dJointSetHingeParam (m_JointID, dParamBounce, bounce);
}

void HingeJoint::Dump()
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
            *m_DumpStream << "Time\tXP\tYP\tZP\tXA\tYA\tZA\tAngle\tAngleRate\tFX1\tFY1\tFZ1\tTX1\tTY1\tTZ1\tFX2\tFY2\tFZ2\tTX2\tTY2\tTZ2\n";
        }
    }


    if (m_DumpStream)
    {
        dVector3 p, a;
        GetHingeAnchor(p);
        GetHingeAxis(a);

        *m_DumpStream << gSimulation->GetTime() << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" <<
                a[0] << "\t" << a[1] << "\t" << a[2] << "\t" << GetHingeAngle() << "\t" << GetHingeAngleRate() << "\t" <<
                m_JointFeedback.f1[0] << "\t" << m_JointFeedback.f1[1] << "\t" << m_JointFeedback.f1[2] << "\t" <<
                m_JointFeedback.t1[0] << "\t" << m_JointFeedback.t1[1] << "\t" << m_JointFeedback.t1[2] << "\t" <<
                m_JointFeedback.f2[0] << "\t" << m_JointFeedback.f2[1] << "\t" << m_JointFeedback.f2[2] << "\t" <<
                m_JointFeedback.t2[0] << "\t" << m_JointFeedback.t2[1] << "\t" << m_JointFeedback.t2[2] << "\t" <<
                "\n";
    }
}

#ifdef USE_OPENGL
void HingeJoint::Draw()
{
    if (m_Visible == false) return;

    dVector3 anchor;
    dVector3 axis;
    dJointGetHingeAnchor(m_JointID, anchor);
    dJointGetHingeAxis(m_JointID, axis);
    if (gAxisFlag)
    {
        gOBJName = m_Name;
        pgd::Vector path[2];
        path[0].x = anchor[0] - axis[0] * m_AxisSize[0];
        path[0].y = anchor[1] - axis[1] * m_AxisSize[1];
        path[0].z = anchor[2] - axis[2] * m_AxisSize[2];
        path[1].x = anchor[0] + axis[0] * m_AxisSize[0];
        path[1].y = anchor[1] + axis[1] * m_AxisSize[1];
        path[1].z = anchor[2] + axis[2] * m_AxisSize[2];
        GLUtils::DrawPath(path, 2, m_AxisSize[0] / 10, m_Colour);
    }
}
#endif

