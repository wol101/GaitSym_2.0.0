/*
 *  DataTargetQuaternion.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif

#include <ode/ode.h>

#include "DataTargetQuaternion.h"
#include "Body.h"
#include "PGDMath.h"
#include "Util.h"
#include "DataFile.h"
#include "Simulation.h"
#include "Geom.h"

// Simulation global
extern Simulation *gSimulation;

DataTargetQuaternion::DataTargetQuaternion()
{
    m_ValueList = 0;
    m_ValueListLength = -1;
}

DataTargetQuaternion::~DataTargetQuaternion()
{
    if (m_TargetTimeList) delete [] m_ValueList;
}

// note in this case the pointer is to a list of the elements of
// size quaternions
// note quaternion is (qs,qx,qy,qz)
void DataTargetQuaternion::SetTargetValues(int size, dReal *values)
{
    int i;
    if (size != m_TargetTimeListLength)
    {
        std::cerr << "DataTargetQuaternion::SetTargetValues error: size = " << size << "\n";
        return;
    }
    if (m_ValueListLength != size)
    {
        if (m_ValueList) delete [] m_ValueList;
        m_ValueListLength = size;
        m_ValueList = new pgd::Quaternion[m_ValueListLength];
    }
    for (i = 0 ; i < m_ValueListLength; i++)
    {
        m_ValueList[i].n = values[i * 4];
        m_ValueList[i].v.x = values[i * 4 + 1];
        m_ValueList[i].v.y = values[i * 4 + 2];
        m_ValueList[i].v.z = values[i * 4 + 3];
        m_ValueList[i].Normalize(); // always do this on input.
    }
}

// note in this case the pointer is to a string which is a list of the elements of
// size quaternions (or angle axis orientations following the standard d or r postscript
// convention
// note quaternion is (qs,qx,qy,qz)
void DataTargetQuaternion::SetTargetValues(const char *buf)
{
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dQuaternion quaternion;
    int i;

    strcpy(lBuf, buf);
    int size = DataFile::ReturnTokens(lBuf, lBufPtrs, l);

    if (size != m_TargetTimeListLength * 4)
    {
        std::cerr << "DataTargetQuaternion::SetTargetValues error: size = " << size << "\n";
        return;
    }

    if (m_ValueListLength != m_TargetTimeListLength)
    {
        if (m_ValueList) delete [] m_ValueList;
        m_ValueListLength = m_TargetTimeListLength;
        m_ValueList = new pgd::Quaternion[m_ValueListLength];
    }
    for (i = 0 ; i < m_ValueListLength; i++)
    {
        Util::GetQuaternion(&lBufPtrs[i * 4], quaternion);
        m_ValueList[i].n = quaternion[0];
        m_ValueList[i].v.x = quaternion[1];
        m_ValueList[i].v.y = quaternion[2];
        m_ValueList[i].v.z = quaternion[3];
    }
}

// returns the degree of match to the stored values
// in this case this is the angle between the two quaternions
dReal DataTargetQuaternion::GetMatchValue()
{
    const dReal *r;
    dReal matchScore = 0;
    Body *body;
    Geom *geom;
    dQuaternion q;
    dReal angle;

    if ((body = dynamic_cast<Body *>(m_Target)) != 0)
    {
        r = body->GetQuaternion();

        angle = pgd::FindAngle(m_ValueList[m_TargetTimeListIndex], pgd::Quaternion(r[0], r[1], r[2], r[3]));

        switch (m_MatchType)
        {
        case linear:
            if (angle < 0) angle = -angle;
            break;

        case square:
            angle = angle * angle;
            break;
        }

        matchScore = m_Weight - m_Slope * angle;

        if (matchScore < m_AbortThreshold)
        {
            //std::cerr << "matchScore " << matchScore << "\n";
            gSimulation->SetDataTargetAbort(true);
        }
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        geom->GetWorldQuaternion(q);

        angle = pgd::FindAngle(m_ValueList[m_TargetTimeListIndex], pgd::Quaternion(q[0], q[1], q[2], q[3]));

        switch (m_MatchType)
        {
        case linear:
            if (angle < 0) angle = -angle;
            break;

        case square:
            angle = angle * angle;
            break;
        }

        matchScore = m_Weight - m_Slope * angle;

        if (matchScore < m_AbortThreshold)
        {
            //std::cerr << "matchScore " << matchScore << "\n";
            gSimulation->SetDataTargetAbort(true);
        }
    }
    else
    {
        std::cerr << "DataTargetQuaternion target missing error " << m_Name << "\n";
    }
    m_TargetTimeListIndex++;
    return matchScore;
}

void DataTargetQuaternion::Dump()
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
            *m_DumpStream << "Time\tTargetQW\tTargetQX\tTargetQY\tTargetQZ\tActualQW\tActualQX\tActualQY\tActualQZ\tAngle\n";
        }
    }

    if (TargetMatch(gSimulation->GetTime()) == false) return;

    Body *body;
    Geom *geom;
    const dReal *r;
    dReal angle;
    dQuaternion q;

    if (m_DumpStream)
    {
        if ((body = dynamic_cast<Body *>(m_Target)) != 0)
        {
            r = body->GetQuaternion();
            angle = pgd::FindAngle(m_ValueList[m_TargetTimeListIndex], pgd::Quaternion(r[0], r[1], r[2], r[3]));
        }
        else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
        {
            geom->GetWorldQuaternion(q);
            angle = pgd::FindAngle(m_ValueList[m_TargetTimeListIndex], pgd::Quaternion(q[0], q[1], q[2], q[3]));
        }

        *m_DumpStream << gSimulation->GetTime() <<
                "\t" << m_ValueList[m_TargetTimeListIndex].n << "\t" << m_ValueList[m_TargetTimeListIndex].v.x << "\t" << m_ValueList[m_TargetTimeListIndex].v.y << "\t" << m_ValueList[m_TargetTimeListIndex].v.z <<
                "\t" << q[0] << "\t" << q[1] << "\t" << q[2] << "\t" << q[3] <<
                "\t" << angle <<
                "\n";
    }
}
