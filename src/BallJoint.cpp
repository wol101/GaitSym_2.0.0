/*
 *  BallJoint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/12/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif

#include <ode/ode.h>

#include "BallJoint.h"
#include "DataFile.h"
#include "Body.h"
#include "Simulation.h"
#include "DebugControl.h"

#ifdef USE_OPENGL
#include "GLUtils.h"
#include "FacetedSphere.h"
extern int gAxisFlag;
extern std::string gOBJName;
#endif

// Simulation global
extern Simulation *gSimulation;

BallJoint::BallJoint(dWorldID worldID)
{
    m_JointID = dJointCreateBall(worldID, 0);
    dJointSetData(m_JointID, this);

    dJointSetFeedback(m_JointID, &m_JointFeedback);
}

void BallJoint::SetBallAnchor (dReal x, dReal y, dReal z)
{
    dJointSetBallAnchor(m_JointID, x, y, z);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void
BallJoint::SetBallAnchor(const char *buf)
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
        std::cerr << "Error in BallJoint::SetBallAnchor\n";
        return; // error condition
    }

        if (isalpha((int)*lBufPtrs[0]) == 0)
        {
                for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dJointSetBallAnchor(m_JointID, pos[0], pos[1], pos[2]);
                return;
        }

        if (count < 4)
    {
        std::cerr << "Error in BallJoint::SetBallAnchor\n";
        return; // error condition
    }
        Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
        if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dJointSetBallAnchor(m_JointID, pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in BallJoint::SetBallAnchor\n";
            return; // error condition
        }
    }
    dVector3 result;
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos (theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    dJointSetBallAnchor(m_JointID, result[0], result[1], result[2]);
}

void BallJoint::GetBallAnchor(dVector3 result)
{
    dJointGetBallAnchor(m_JointID, result);
}

void BallJoint::GetBallAnchor2(dVector3 result)
{
    dJointGetBallAnchor2(m_JointID, result);
}

void BallJoint::Dump()
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
            *m_DumpStream << "Time\tXP\tYP\tZP\tFX1\tFY1\tFZ1\tTX1\tTY1\tTZ1\tFX2\tFY2\tFZ2\tTX2\tTY2\tTZ2\n";
        }
    }


    if (m_DumpStream)
    {
        dVector3 p, a;
        GetBallAnchor(p);

        *m_DumpStream << gSimulation->GetTime() << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" <<
                m_JointFeedback.f1[0] << "\t" << m_JointFeedback.f1[1] << "\t" << m_JointFeedback.f1[2] << "\t" <<
                m_JointFeedback.t1[0] << "\t" << m_JointFeedback.t1[1] << "\t" << m_JointFeedback.t1[2] << "\t" <<
                m_JointFeedback.f2[0] << "\t" << m_JointFeedback.f2[1] << "\t" << m_JointFeedback.f2[2] << "\t" <<
                m_JointFeedback.t2[0] << "\t" << m_JointFeedback.t2[1] << "\t" << m_JointFeedback.t2[2] << "\t" <<
                "\n";
    }
}

#ifdef USE_OPENGL
void BallJoint::Draw()
{
    if (m_Visible == false) return;

    if (gAxisFlag)
    {
        dVector3 anchor;
        dJointGetBallAnchor(m_JointID, anchor);
        gOBJName = m_Name;
        // and draw the sphere
        const static int kLevels = 3;
        dReal radius = m_AxisSize[0] / 10;
        FacetedSphere sphere(radius, kLevels);
        sphere.SetColour(m_Colour);
        sphere.SetDisplayPosition(anchor[0], anchor[1], anchor[2]);
        // sphere.SetDisplayRotation(r);
        sphere.Draw();
    }
}
#endif
