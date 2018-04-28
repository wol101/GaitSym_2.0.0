/*
 *  DataTargetScalar.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>

#include <ode/ode.h>

#include "DataTargetScalar.h"
#include "Body.h"
#include "Simulation.h"
#include "HingeJoint.h"
#include "BallJoint.h"
#include "Geom.h"

// Simulation global
extern Simulation *gSimulation;

DataTargetScalar::DataTargetScalar()
{
    m_ValueList = 0;
    m_ValueListLength = -1;
    m_DataType = XP;
}

DataTargetScalar::~DataTargetScalar()
{
    if (m_TargetTimeList) delete [] m_ValueList;
}

void DataTargetScalar::SetTargetValues(int size, dReal *values)
{
    int i;
    if (size != m_TargetTimeListLength)
    {
        std::cerr << "DataTargetScalar::SetTargetValues error: size = " << size << "\n";
        return;
    }
    if (m_ValueListLength != size)
    {
        if (m_ValueList) delete [] m_ValueList;
        m_ValueListLength = size;
        m_ValueList = new dReal[m_ValueListLength];
    }
    for (i = 0 ; i < m_ValueListLength; i++)
    {
        m_ValueList[i] = values[i];
    }
}


// returns the degree of match to the stored values

dReal DataTargetScalar::GetMatchValue()
{
    dReal matchScore = 0;
    const dReal *r;
    dVector3 result;
    dQuaternion q;

    Body *body;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    Geom *geom;

    if ((body = dynamic_cast<Body *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case Q0:
            r = body->GetQuaternion();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[0] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case Q1:
            r = body->GetQuaternion();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[1] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case Q2:
            r = body->GetQuaternion();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[2] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case Q3:
            r = body->GetQuaternion();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[3] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case XP:
            r = body->GetPosition();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[0] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case YP:
            r = body->GetPosition();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[1] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case ZP:
            r = body->GetPosition();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[2] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case XV:
            r = body->GetLinearVelocity();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[0] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case YV:
            r = body->GetLinearVelocity();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[1] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case ZV:
            r = body->GetLinearVelocity();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[2] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case XRV:
            r = body->GetAngularVelocity();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[0] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case YRV:
            r = body->GetAngularVelocity();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[1] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case ZRV:
            r = body->GetAngularVelocity();
            matchScore = m_Weight - m_Slope * PositiveFunction(r[2] - m_ValueList[m_TargetTimeListIndex]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
    {
        hingeJoint->GetHingeAnchor(result);
        switch (m_DataType)
        {
        case XP:
            matchScore = m_Weight - m_Slope * PositiveFunction(result[0] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case YP:
            matchScore = m_Weight - m_Slope * PositiveFunction(result[1] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case ZP:
            matchScore = m_Weight - m_Slope * PositiveFunction(result[2] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case Angle:
            matchScore = m_Weight - m_Slope * PositiveFunction(hingeJoint->GetHingeAngle() - m_ValueList[m_TargetTimeListIndex]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
    {
        ballJoint->GetBallAnchor(result);
        switch (m_DataType)
        {
        case XP:
            matchScore = m_Weight - m_Slope * PositiveFunction(result[0] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case YP:
            matchScore = m_Weight - m_Slope * PositiveFunction(result[1] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case ZP:
            matchScore = m_Weight - m_Slope * PositiveFunction(result[2] - m_ValueList[m_TargetTimeListIndex]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case Q0:
            geom->GetWorldQuaternion(q);
            matchScore = m_Weight - m_Slope * PositiveFunction(q[0] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case Q1:
            geom->GetWorldQuaternion(q);
            matchScore = m_Weight - m_Slope * PositiveFunction(q[1] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case Q2:
            geom->GetWorldQuaternion(q);
            matchScore = m_Weight - m_Slope * PositiveFunction(q[2] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case Q3:
            geom->GetWorldQuaternion(q);
            matchScore = m_Weight - m_Slope * PositiveFunction(q[3] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case XP:
            geom->GetWorldPosition(result);
            matchScore = m_Weight - m_Slope * PositiveFunction(result[0] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case YP:
            geom->GetWorldPosition(result);
            matchScore = m_Weight - m_Slope * PositiveFunction(result[1] - m_ValueList[m_TargetTimeListIndex]);
            break;
        case ZP:
            geom->GetWorldPosition(result);
            matchScore = m_Weight - m_Slope * PositiveFunction(result[2] - m_ValueList[m_TargetTimeListIndex]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if (m_Target == 0)
    {
        switch(m_DataType)
        {
        case MetabolicEnergy:
            matchScore = m_Weight - m_Slope * PositiveFunction(gSimulation->GetMetabolicEnergy() - m_ValueList[m_TargetTimeListIndex]);
            break;
        case MechanicalEnergy:
            matchScore = m_Weight - m_Slope * PositiveFunction(gSimulation->GetMechanicalEnergy() - m_ValueList[m_TargetTimeListIndex]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else
    {
        std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataTarget " << m_DataType << "\n";
    }

    if (matchScore < m_AbortThreshold)
    {
        //std::cerr << "matchScore " << matchScore << "\n";
        gSimulation->SetDataTargetAbort(true);
    }

    // bump the time index so we don't reuse this match
    m_TargetTimeListIndex++;
    return matchScore;
}

dReal DataTargetScalar::PositiveFunction(dReal v)
{
    switch (m_MatchType)
    {
    case linear:
        if (v > 0) return v;
        else return -v;
        break;

    case square:
        return v * v;
        break;
    }
    return 0;
}

void DataTargetScalar::Dump()
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
            *m_DumpStream << "Time\tTargetV\tActualV\tError\n";
        }
    }

    if (TargetMatch(gSimulation->GetTime()) == false) return;

    Body *body;
    Geom *geom;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    const dReal *r;
    dReal ref;
    dVector3 result;
    dQuaternion q;

    if (m_DumpStream)
    {
        if ((body = dynamic_cast<Body *>(m_Target)) != 0)
        {
            switch (m_DataType)
            {
            case Q0:
                r = body->GetQuaternion();
                ref = r[0];
                break;
            case Q1:
                r = body->GetQuaternion();
                ref = r[1];
                break;
            case Q2:
                r = body->GetQuaternion();
                ref = r[2];
                break;
            case Q3:
                r = body->GetQuaternion();
                ref = r[3];
                break;
            case XP:
                r = body->GetPosition();
                ref = r[0];
                break;
            case YP:
                r = body->GetPosition();
                ref = r[1];
                break;
            case ZP:
                r = body->GetPosition();
                ref = r[2];
                break;
            case XV:
                r = body->GetLinearVelocity();
                ref = r[0];
                break;
            case YV:
                r = body->GetLinearVelocity();
                ref = r[1];
                break;
            case ZV:
                r = body->GetLinearVelocity();
                ref = r[2];
                break;
            case XRV:
                r = body->GetAngularVelocity();
                ref = r[0];
                break;
            case YRV:
                r = body->GetAngularVelocity();
                ref = r[1];
                break;
            case ZRV:
                r = body->GetAngularVelocity();
                ref = r[2];
                break;
            }
        }
        else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
        {
            hingeJoint->GetHingeAnchor(result);
            switch (m_DataType)
            {
            case XP:
                ref = result[0];
                break;
            case YP:
                ref = result[1];
                break;
            case ZP:
                ref = result[2];
                break;
            case Angle:
                ref = hingeJoint->GetHingeAngle();
            break;
            }
        }
        else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
        {
            ballJoint->GetBallAnchor(result);
            switch (m_DataType)
            {
            case XP:
                ref = result[0];
                break;
            case YP:
                ref = result[1];
                break;
            case ZP:
                ref = result[2];
                break;
            }
        }
        else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
        {
            switch (m_DataType)
            {
            case Q0:
                geom->GetQuaternion(q);
                ref  = q[0];
                break;
            case Q1:
                geom->GetQuaternion(q);
                ref = q[1];
                break;
            case Q2:
                geom->GetQuaternion(q);
                ref = q[2];
                break;
            case Q3:
                geom->GetQuaternion(q);
                ref = q[3];
                break;
            case XP:
                r = body->GetPosition();
                ref = r[0];
                break;
            case YP:
                r = body->GetPosition();
                ref = r[1];
                break;
            case ZP:
                r = body->GetPosition();
                ref = r[2];
                break;
            }
        }
        else if (m_Target == 0)
        {
            switch(m_DataType)
            {
            case MetabolicEnergy:
                ref = gSimulation->GetMetabolicEnergy();
                break;
            case MechanicalEnergy:
                ref = gSimulation->GetMechanicalEnergy();
                break;
            }
        }

        *m_DumpStream << gSimulation->GetTime() <<
                "\t" << m_ValueList[m_TargetTimeListIndex] <<
                "\t" << ref <<
                "\t" << ref - m_ValueList[m_TargetTimeListIndex] <<
                "\n";
    }
}

