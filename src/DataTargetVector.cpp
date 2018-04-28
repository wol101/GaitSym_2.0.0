/*
 *  DataTargetVector.h
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

#include "DataTargetVector.h"
#include "Body.h"
#include "PGDMath.h"
#include "Util.h"
#include "DataFile.h"
#include "Simulation.h"
#include "HingeJoint.h"
#include "BallJoint.h"
#include "Geom.h"

// Simulation global
extern Simulation *gSimulation;

DataTargetVector::DataTargetVector()
{
    m_ValueList = 0;
    m_ValueListLength = -1;
}

DataTargetVector::~DataTargetVector()
{
    if (m_TargetTimeList) delete [] m_ValueList;
}

// note in this case the pointer is to a list of the elements of
// size vectors
void DataTargetVector::SetTargetValues(int size, dReal *values)
{
    int i;
    if (size != m_TargetTimeListLength)
    {
        std::cerr << "DataTargetVector::SetTargetValues error: size = " << size << "\n";
        return;
    }
    if (m_ValueListLength != size)
    {
        if (m_ValueList) delete [] m_ValueList;
        m_ValueListLength = size;
        m_ValueList = new pgd::Vector[m_ValueListLength];
    }
    for (i = 0 ; i < m_ValueListLength; i++)
    {
        m_ValueList[i].x = values[i * 3];
        m_ValueList[i].y = values[i * 3 + 1];
        m_ValueList[i].z = values[i * 3 + 2];
    }
}

// note in this case the pointer is to a string which is a list of the elements of
// size vector
void DataTargetVector::SetTargetValues(const char *buf)
{
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    int i;

    strcpy(lBuf, buf);
    int size = DataFile::ReturnTokens(lBuf, lBufPtrs, l);

    if (size != m_TargetTimeListLength * 3)
    {
        std::cerr << "DataTargetVector::SetTargetValues error: size = " << size << "\n";
        return;
    }

    if (m_ValueListLength != m_TargetTimeListLength)
    {
        if (m_ValueList) delete [] m_ValueList;
        m_ValueListLength = m_TargetTimeListLength;
        m_ValueList = new pgd::Vector[m_ValueListLength];
    }
    for (i = 0 ; i < m_ValueListLength; i++)
    {
        m_ValueList[i].x = Util::Double(lBufPtrs[i * 3]);
        m_ValueList[i].y = Util::Double(lBufPtrs[i * 3 + 1]);
        m_ValueList[i].z = Util::Double(lBufPtrs[i * 3 + 2]);
    }
}

// returns the degree of match to the stored values
// in this case this is the euclidean between the two vectors
dReal DataTargetVector::GetMatchValue()
{
    const dReal *r;
    dReal matchScore = 0;
    Body *body;
    Geom *geom;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    dReal err;
    dVector3 v;

    if ((body = dynamic_cast<Body *>(m_Target)) != 0)
    {
        r = body->GetPosition();

        err = (pgd::Vector(r[0], r[1], r[2]) - m_ValueList[m_TargetTimeListIndex]).Magnitude2();
        if (m_MatchType == linear) err = sqrt(err);

        matchScore = m_Weight - m_Slope * err;

        if (matchScore < m_AbortThreshold)
        {
            //std::cerr << "matchScore " << matchScore << "\n";
            gSimulation->SetDataTargetAbort(true);
        }
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        geom->GetWorldPosition(v);

        err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[m_TargetTimeListIndex]).Magnitude2();
        if (m_MatchType == linear) err = sqrt(err);

        matchScore = m_Weight - m_Slope * err;

        if (matchScore < m_AbortThreshold)
        {
            //std::cerr << "matchScore " << matchScore << "\n";
            gSimulation->SetDataTargetAbort(true);
        }
    }
    else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
    {
        hingeJoint->GetHingeAnchor(v);

        err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[m_TargetTimeListIndex]).Magnitude2();
        if (m_MatchType == linear) err = sqrt(err);

        matchScore = m_Weight - m_Slope * err;

        if (matchScore < m_AbortThreshold)
        {
            //std::cerr << "matchScore " << matchScore << "\n";
            gSimulation->SetDataTargetAbort(true);
        }
    }
    else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
    {
        hingeJoint->GetHingeAnchor(v);

        err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[m_TargetTimeListIndex]).Magnitude2();
        if (m_MatchType == linear) err = sqrt(err);

        matchScore = m_Weight - m_Slope * err;

        if (matchScore < m_AbortThreshold)
        {
            //std::cerr << "matchScore " << matchScore << "\n";
            gSimulation->SetDataTargetAbort(true);
        }
    }
    else
    {
        std::cerr << "DataTargetVector target missing error " << m_Name << "\n";
    }
    m_TargetTimeListIndex++;
    return matchScore;
}

void DataTargetVector::Dump()
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
            *m_DumpStream << "Time\tTargetX\tTargetY\tTargetZ\tActualX\tActualY\tActualZ\tDistance\n";
        }
    }

    if (TargetMatch(gSimulation->GetTime()) == false) return;

    Body *body;
    Geom *geom;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    const dReal *r;
    dReal err;
    dVector3 v;

    if (m_DumpStream)
    {
        if ((body = dynamic_cast<Body *>(m_Target)) != 0)
        {
            r = body->GetPosition();
            err = (pgd::Vector(r[0], r[1], r[2]) - m_ValueList[m_TargetTimeListIndex]).Magnitude();
        }
        else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
        {
            geom->GetWorldPosition(v);
            err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[m_TargetTimeListIndex]).Magnitude();
            r = v;
        }
        else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
        {
            hingeJoint->GetHingeAnchor(v);
            err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[m_TargetTimeListIndex]).Magnitude();
            r = v;
        }
        else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
        {
            hingeJoint->GetHingeAnchor(v);
            err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[m_TargetTimeListIndex]).Magnitude();
            r = v;
        }

        *m_DumpStream << gSimulation->GetTime() <<
                "\t" << m_ValueList[m_TargetTimeListIndex].x << "\t" << m_ValueList[m_TargetTimeListIndex].y << "\t" << m_ValueList[m_TargetTimeListIndex].z <<
                "\t" << r[0] << "\t" << r[1] << "\t" << r[2] <<
                "\t" << err <<
                "\n";
    }
}
