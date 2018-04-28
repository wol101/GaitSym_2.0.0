/*
 *  NPointStrap.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 27/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include <math.h>
#include <string.h>
#include <iostream>

#include "NPointStrap.h"
#include "Body.h"
#include "PGDMath.h"
#include "Simulation.h"
#include "Util.h"
#include "DebugControl.h"

#ifdef USE_OPENGL
#include "FacetedPolyline.h"
extern int gDrawMuscles;
extern int gDrawMuscleForces;
#endif

// Simulation global
extern Simulation *gSimulation;

NPointStrap::NPointStrap(): TwoPointStrap()
{
}

NPointStrap::~NPointStrap()
{
    unsigned int i;
    for (i = 0; i < m_ViaPointList.size(); i++) delete [] m_ViaPointList[i];
    //for (i = 0; i < m_WorldViaPointList.size(); i++) delete m_WorldViaPointList[i];
}

void NPointStrap::SetViaPoints(std::vector<Body *> *bodyList, std::vector<dReal *> *pointList)
{
    if (pointList->size() != bodyList->size())
    {
        std::cerr << __FILE__ << " " << __LINE__ << " Error in SetViaPoints\n";
        return;
    }
    for (unsigned int i = 0; i < pointList->size(); i++)
    {
        PointForce *viaPointForce = new PointForce();
        viaPointForce->body = (*bodyList)[i];
        m_PointForceList.push_back(viaPointForce);
        m_ViaBodyList.push_back(viaPointForce->body);
        dReal *point = new dReal[sizeof(dVector3)];
        memcpy(point, (*pointList)[i], sizeof(dVector3));
        m_ViaPointList.push_back(point);
    }
}

// parses the position allowing a relative position specified by BODY IDk
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void NPointStrap::SetViaPoints(std::vector<Body *> *bodyList, std::vector<std::string *> *pointList)
{
    std::vector<dReal *> myPointList;
    std::vector<std::string> tokens;
    int i;
    unsigned int j;
    dVector3 pos, result;
    Body *body;
    dReal *point;

    if (pointList->size() != bodyList->size())
    {
        std::cerr << __FILE__ << " " << __LINE__ << " Error in SetViaPoints\n";
        return;
    }
    for (j = 0; j < pointList->size(); j++)
    {
        tokens.clear();
        Util::Tokenizer((*pointList)[j]->c_str(), tokens, "");
        body = (*bodyList)[j];

        if (tokens.size() < 3)
        {
            std::cerr << __FILE__ << " " << __LINE__ << " Error in SetViaPoints\n";
            return; // error condition
        }

        if (isalpha((int)tokens[0][0]) == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(tokens[i].c_str(), 0);
            point = new dReal[sizeof(dVector3)];
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], point); // convert from world to body
            myPointList.push_back(point);
            continue;
        }

        if (tokens.size() < 4)
        {
            std::cerr << __FILE__ << " " << __LINE__ << " Error in SetViaPoints\n";
            return; // error condition
        }
        Body *theBody = gSimulation->GetBody(tokens[0].c_str());
        if (theBody == 0)
        {
            if (tokens[0] == "World")
            {
                for (i = 0; i < 3; i++) pos[i] = strtod(tokens[i + 1].c_str(), 0);
                point = new dVector3();
                dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], point); // convert from world to body
                myPointList.push_back(point);
                continue;
            }
            else
            {
                std::cerr << __FILE__ << " " << __LINE__ << " Error in SetViaPoints\n";
                return; // error condition
            }
        }
        for (i = 0; i < 3; i++) pos[i] = strtod(tokens[i + 1].c_str(), 0);
        point = new dVector3();
        dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
        dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], point); // convert from world to body
        myPointList.push_back(point);
    }
    SetViaPoints(bodyList, &myPointList);
    for (j = 0; j < myPointList.size(); j++) delete [] myPointList[j];

}

void NPointStrap::Calculate(dReal deltaT)
{
    PointForce *theOrigin = m_PointForceList[0];
    PointForce *theInsertion = m_PointForceList[1];
    unsigned int i;
    dReal *ptr;

    // calculate the world positions
    dBodyGetRelPointPos(m_OriginBody->GetBodyID(), m_Origin[0], m_Origin[1], m_Origin[2],
                        theOrigin->point);
    dBodyGetRelPointPos(m_InsertionBody->GetBodyID(), m_Insertion[0], m_Insertion[1], m_Insertion[2],
                        theInsertion->point);
    for (i = 0; i < m_ViaPointList.size(); i++)
    {
        ptr = m_ViaPointList[i];
        dBodyGetRelPointPos(m_ViaBodyList[i]->GetBodyID(), ptr[0], ptr[1], ptr[2],
                        m_PointForceList[i + 2]->point);
    }

    unsigned int *mapping = new unsigned int[m_PointForceList.size()];
    for (i = 0; i < m_PointForceList.size(); i++)
    {
        if (i == 0) mapping[i] = 0;
        else
            if (i == m_PointForceList.size() - 1) mapping[i] = 1;
            else mapping[i] = i + 1;
    }

    m_LastLength = m_Length;
    pgd::Vector line, line2;
    m_Length = 0;
    dReal len;
    for (i = 0; i < m_PointForceList.size(); i++)
    {
        if (i == 0)
        {
            line.x = m_PointForceList[mapping[i + 1]]->point[0] - m_PointForceList[mapping[i]]->point[0];
            line.y = m_PointForceList[mapping[i + 1]]->point[1] - m_PointForceList[mapping[i]]->point[1];
            line.z = m_PointForceList[mapping[i + 1]]->point[2] - m_PointForceList[mapping[i]]->point[2];
            len = line.Magnitude();
            m_Length += len;
            line /= len;
        }
        else if (i == m_PointForceList.size() - 1)
        {
            line.x = m_PointForceList[mapping[i - 1]]->point[0] - m_PointForceList[mapping[i]]->point[0];
            line.y = m_PointForceList[mapping[i - 1]]->point[1] - m_PointForceList[mapping[i]]->point[1];
            line.z = m_PointForceList[mapping[i - 1]]->point[2] - m_PointForceList[mapping[i]]->point[2];
            line.Normalize();
        }
        else
        {
            line.x = m_PointForceList[mapping[i + 1]]->point[0] - m_PointForceList[mapping[i]]->point[0];
            line.y = m_PointForceList[mapping[i + 1]]->point[1] - m_PointForceList[mapping[i]]->point[1];
            line.z = m_PointForceList[mapping[i + 1]]->point[2] - m_PointForceList[mapping[i]]->point[2];
            len = line.Magnitude();
            m_Length += len;
            line /= len;
            line2.x = m_PointForceList[mapping[i - 1]]->point[0] - m_PointForceList[mapping[i]]->point[0];
            line2.y = m_PointForceList[mapping[i - 1]]->point[1] - m_PointForceList[mapping[i]]->point[1];
            line2.z = m_PointForceList[mapping[i - 1]]->point[2] - m_PointForceList[mapping[i]]->point[2];
            line2.Normalize();
            line += line2;
        }

        m_PointForceList[mapping[i]]->vector[0] = line.x;
        m_PointForceList[mapping[i]]->vector[1] = line.y;
        m_PointForceList[mapping[i]]->vector[2] = line.z;
    }

    if (deltaT != 0.0) m_Velocity = (m_Length - m_LastLength) / deltaT;
    else m_Velocity = 0;

    if (gDebug == StrapDebug)
    {
        pgd::Vector totalF(0, 0, 0);
        for (i = 0; i < m_PointForceList.size(); i++)
        {
            *gDebugStream << "NPointStrap::Calculate " <<
            *m_PointForceList[i]->body->GetName() << " " <<
            m_PointForceList[i]->point[0] << " " << m_PointForceList[i]->point[1] << " " << m_PointForceList[i]->point[2] << " " <<
            m_PointForceList[i]->vector[0] << " " << m_PointForceList[i]->vector[1] << " " << m_PointForceList[i]->vector[2] << "\n";
            totalF.x += m_PointForceList[i]->vector[0]; totalF.y += m_PointForceList[i]->vector[1]; totalF.z += m_PointForceList[i]->vector[2];
        }
        *gDebugStream << "Total F " << totalF.x << " " << totalF.y << " " << totalF.z << "\n";
    }
    delete [] mapping;
}

#ifdef USE_OPENGL
void
NPointStrap::Draw()
{
    if (m_Visible == false || gDrawMuscles == false) return;

    const int kSides = 128;
    FacetedPolyline *facetedPolyline;
    unsigned int i;

    if (gSimulation->GetTime() != m_LastDrawTime)
    {
        m_LastDrawTime = gSimulation->GetTime();
        std::vector<FacetedObject *>::const_iterator iterFO;
        for (iterFO = m_DrawList.begin(); iterFO != m_DrawList.end(); iterFO++)
            delete *iterFO;
        m_DrawList.clear();

        std::vector<pgd::Vector> polyline;
        polyline.push_back(pgd::Vector(m_PointForceList[0]->point[0], m_PointForceList[0]->point[1], m_PointForceList[0]->point[2]));
        for (i = 2; i < m_PointForceList.size(); i++)
        {
            polyline.push_back(pgd::Vector(m_PointForceList[i]->point[0], m_PointForceList[i]->point[1], m_PointForceList[i]->point[2]));
        }
        polyline.push_back(pgd::Vector(m_PointForceList[1]->point[0], m_PointForceList[1]->point[1], m_PointForceList[1]->point[2]));
        facetedPolyline = new FacetedPolyline(&polyline, m_Radius, kSides);
        facetedPolyline->SetColour(m_Colour);
        facetedPolyline->SetUseDisplayList(true);
        m_DrawList.push_back(facetedPolyline);

        if (gDrawMuscleForces)
        {
            for (i = 0; i < m_PointForceList.size(); i++)
            {
                polyline.clear();
                pgd::Vector f = pgd::Vector(m_PointForceList[i]->vector[0], m_PointForceList[i]->vector[1], m_PointForceList[i]->vector[2]) * m_Tension * m_ForceScale;
                polyline.push_back(pgd::Vector(m_PointForceList[i]->point[0], m_PointForceList[i]->point[1], m_PointForceList[i]->point[2]));
                polyline.push_back(pgd::Vector(m_PointForceList[i]->point[0], m_PointForceList[i]->point[1], m_PointForceList[i]->point[2]) + f);
                facetedPolyline = new FacetedPolyline(&polyline, m_ForceRadius, kSides);
                facetedPolyline->SetColour(m_Colour);
                facetedPolyline->SetUseDisplayList(true);
                m_DrawList.push_back(facetedPolyline);
            }
        }
    }

    for (i = 0; i < m_DrawList.size(); i++)
        m_DrawList[i]->Draw();
}
#endif


