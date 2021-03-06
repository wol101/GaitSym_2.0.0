/*
 *  CylinderWrapStrap.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include <math.h>
#include <string.h>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif

#include "CylinderWrapStrap.h"
#include "Body.h"
#include "PGDMath.h"
#include "DataFile.h"
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

static int CylinderWrap(pgd::Vector &origin, pgd::Vector &insertion, dReal radius, int nWrapSegments, dReal maxAngle,
                        pgd::Vector &originForce, pgd::Vector &insertionForce, pgd::Vector &cylinderForce, pgd::Vector &cylinderForcePosition,
                        dReal *pathLength, pgd::Vector *pathCoordinates, int *numPathCoordinates);



CylinderWrapStrap::CylinderWrapStrap()
{
    PointForce *origin = new PointForce();
    PointForce *insertion = new PointForce();
    PointForce *cylinder = new PointForce();
    m_PointForceList.push_back(origin);
    m_PointForceList.push_back(insertion);
    m_PointForceList.push_back(cylinder);

    m_CylinderRadius = 1;
    m_WrapStatus = -1;
    m_NumWrapSegments = 16;
    m_NumPathCoordinates = 0;

#ifdef USE_OPENGL
    m_PathCoordinates = new pgd::Vector[m_NumWrapSegments + 2];
#else
    m_PathCoordinates = 0;
#endif
}

CylinderWrapStrap::~CylinderWrapStrap()
{
    if (m_PathCoordinates) delete [] m_PathCoordinates;
}

void CylinderWrapStrap::SetOrigin(Body *body, dVector3 point)
{
    m_OriginBody = body;
    m_PointForceList[0]->body = m_OriginBody;
    m_OriginPosition.x = point[0];
    m_OriginPosition.y = point[1];
    m_OriginPosition.z = point[2];
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void CylinderWrapStrap::SetOrigin(Body *body, const char *buf)
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
        std::cerr << "Error in CylinderWrapStrap::SetOrigin\n";
        return; // error condition
    }


    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetOrigin(body, result);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in CylinderWrapStrap::SetOrigin\n";
        return; // error condition
    }
    Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetOrigin(body, result);
            return;
        }
        else
        {
            std::cerr << "Error in CylinderWrapStrap::SetOrigin\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetOrigin(body, pos);
}

void CylinderWrapStrap::SetInsertion(Body *body, dVector3 point)
{
    m_InsertionBody = body;
    m_PointForceList[1]->body = m_InsertionBody;
    m_InsertionPosition.x = point[0];
    m_InsertionPosition.y = point[1];
    m_InsertionPosition.z = point[2];
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void CylinderWrapStrap::SetInsertion(Body *body, const char *buf)
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
        std::cerr << "Error in CylinderWrapStrap::SetInsertion\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetInsertion(body, result);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in CylinderWrapStrap::SetInsertion\n";
        return; // error condition
    }
    Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetInsertion(body, result);
            return;
        }
        else
        {
            std::cerr << "Error in CylinderWrapStrap::SetInsertion\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetInsertion(body, pos);
}

void CylinderWrapStrap::SetCylinderBody(Body *body)
{
    m_CylinderBody = body;
    m_PointForceList[2]->body = m_CylinderBody;
}

void CylinderWrapStrap::SetCylinderPosition(dReal x, dReal y, dReal z)
{
    m_CylinderPosition.x = x;
    m_CylinderPosition.y = y;
    m_CylinderPosition.z = z;
}

void CylinderWrapStrap::GetCylinder(Body **body, dVector3 position, dReal *radius, dQuaternion q)
{
    *body = m_CylinderBody;
    position[0] = m_CylinderPosition.x;
    position[1] = m_CylinderPosition.y;
    position[2] = m_CylinderPosition.z;
    *radius = m_CylinderRadius;
    q[0] = m_CylinderQuaternion.n;
    q[1] = m_CylinderQuaternion.v.x;
    q[2] = m_CylinderQuaternion.v.y;
    q[3] = m_CylinderQuaternion.v.z;
}


// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void CylinderWrapStrap::SetCylinderPosition(const char *buf)
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
        std::cerr << "Error in CylinderWrapStrap::SetCylinderPosition\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(m_CylinderBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetCylinderPosition(result[0], result[1], result[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in CylinderWrapStrap::SetCylinderPosition\n";
        return; // error condition
    }
    Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(m_CylinderBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetCylinderPosition(result[0], result[1], result[2]);
            return;
        }
        else
        {
            std::cerr << "Error in CylinderWrapStrap::SetCylinderPosition\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(m_CylinderBody->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetCylinderPosition(pos[0], pos[1], pos[2]);
}

void CylinderWrapStrap::SetCylinderQuaternion(dReal q0, dReal q1, dReal q2, dReal q3)
{
    m_CylinderQuaternion.n = q0;
    m_CylinderQuaternion.v.x = q1;
    m_CylinderQuaternion.v.y = q2;
    m_CylinderQuaternion.v.z = q3;
    m_CylinderQuaternion.Normalize(); // this is the safest option
}

// parses the quaternion allowing a relative position specified by BODY ID
// note quaternion is (qs,qx,qy,qz)
// s x y z - world coordinates
// bodyName s x y z - position relative to bodyName local coordinate system
void CylinderWrapStrap::SetCylinderQuaternion(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dQuaternion quaternion;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 4)
    {
        std::cerr << "Error in CylinderWrapStrap::SetCylinderQuaternion\n";
        return; // error condition
    }


    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        const dReal *q = m_CylinderBody->GetQuaternion();
        pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
        Util::GetQuaternion(&lBufPtrs[0], quaternion);
        pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        pgd::Quaternion qLocal = ~qBody * qWorld;
        SetCylinderQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
        return;
    }

    if (count < 5)
    {
        std::cerr << "Error in CylinderWrapStrap::SetCylinderQuaternion\n";
        return; // error condition
    }
    Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            const dReal *q = m_CylinderBody->GetQuaternion();
            pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
            Util::GetQuaternion(&lBufPtrs[1], quaternion);
            pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            pgd::Quaternion qLocal = ~qBody * qWorld;
            SetCylinderQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
            return;
        }
        else
        {
            std::cerr << "Error in CylinderWrapStrap::SetCylinderQuaternion\n";
            return; // error condition
        }
    }

    // first get world quaternion
    const dReal *q2 = theBody->GetQuaternion();
    pgd::Quaternion qBody1(q2[0], q2[1], q2[2], q2[3]);
    Util::GetQuaternion(&lBufPtrs[1], quaternion);
    pgd::Quaternion qBody2(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    pgd::Quaternion qWorld = qBody1 * qBody2;

    // then set the local quaternion
    const dReal *q = m_CylinderBody->GetQuaternion();
    pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
    pgd::Quaternion qLocal = ~qBody * qWorld;
    SetCylinderQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);

}

void CylinderWrapStrap::Calculate(dReal deltaT)
{
    int i;
    // get the necessary body orientations and positions
    const dReal *q;
    q = dBodyGetQuaternion(m_OriginBody->GetBodyID());
    pgd::Quaternion qOriginBody(q[0], q[1], q[2], q[3]);
    q = dBodyGetPosition(m_OriginBody->GetBodyID());
    pgd::Vector vOriginBody(q[0], q[1], q[2]);
    q = dBodyGetQuaternion(m_InsertionBody->GetBodyID());
    pgd::Quaternion qInsertionBody(q[0], q[1], q[2], q[3]);
    q = dBodyGetPosition(m_InsertionBody->GetBodyID());
    pgd::Vector vInsertionBody(q[0], q[1], q[2]);
    q = dBodyGetQuaternion(m_CylinderBody->GetBodyID());
    pgd::Quaternion qCylinderBody(q[0], q[1], q[2], q[3]);
    q = dBodyGetPosition(m_CylinderBody->GetBodyID());
    pgd::Vector vCylinderBody(q[0], q[1], q[2]);

    // calculate some inverses
    pgd::Quaternion qCylinderBodyInv = ~qCylinderBody;
    pgd::Quaternion qCylinderQuaternionInv = ~m_CylinderQuaternion;

    // get the world coordinates of the origin and insertion
    pgd::Vector worldOriginPosition = QVRotate(qOriginBody, m_OriginPosition) + vOriginBody;
    pgd::Vector worldInsertionPosition = QVRotate(qInsertionBody, m_InsertionPosition) + vInsertionBody;

    // now calculate as cylinder coordinates
    pgd::Vector v;
    if (m_OriginBody->GetBodyID() == m_CylinderBody->GetBodyID()) v = m_OriginPosition;
    else v = QVRotate(qCylinderBodyInv, worldOriginPosition - vCylinderBody);
    pgd::Vector cylinderOriginPosition = QVRotate(qCylinderQuaternionInv, v - m_CylinderPosition);

    if (m_InsertionBody->GetBodyID() == m_CylinderBody->GetBodyID()) v = m_InsertionPosition;
    else v = QVRotate(qCylinderBodyInv, worldInsertionPosition - vCylinderBody);
    pgd::Vector cylinderInsertionPosition = QVRotate(qCylinderQuaternionInv, v - m_CylinderPosition);

    // std::cerr << "cylinderOriginPosition " << cylinderOriginPosition.x << " " << cylinderOriginPosition.y << " " << cylinderOriginPosition.z << " ";
    // std::cerr << "cylinderInsertionPosition " << cylinderInsertionPosition.x << " " << cylinderInsertionPosition.y << " " << cylinderInsertionPosition.z << "\n";

    pgd::Vector theOriginForce;
    pgd::Vector theInsertionForce;
    pgd::Vector theCylinderForce;
    pgd::Vector theCylinderForcePosition;
    m_LastLength = m_Length;

    m_WrapStatus = CylinderWrap(cylinderOriginPosition, cylinderInsertionPosition, m_CylinderRadius, m_NumWrapSegments, M_PI,
                                theOriginForce, theInsertionForce, theCylinderForce, theCylinderForcePosition,
                                &m_Length, m_PathCoordinates, &m_NumPathCoordinates);

    // calculate the length and velocity
    if (deltaT != 0.0) m_Velocity = (m_Length - m_LastLength) / deltaT;
    else m_Velocity = 0;

    if (gDebug == CylinderWrapStrapDebug)
    {
        *gDebugStream << "CylinderWrapStrap::Calculate m_Name " << m_Name <<
                "cylinderOriginPosition " << cylinderOriginPosition.x << " " << cylinderOriginPosition.y << " " << cylinderOriginPosition.z << " " <<
                "cylinderInsertionPosition " << cylinderInsertionPosition.x << " " << cylinderInsertionPosition.y << " " << cylinderInsertionPosition.z << "  " <<
                "theOriginForce " << theOriginForce.x << " " << theOriginForce.y << " " << theOriginForce.z << " " <<
                "theInsertionForce " << theInsertionForce.x << " " << theInsertionForce.y << " " << theInsertionForce.z << "  " <<
                "theCylinderForcePosition " << theCylinderForcePosition.x << " " << theCylinderForcePosition.y << " " << theCylinderForcePosition.z << "  " <<
                "theCylinderForce " << theCylinderForce.x << " " << theCylinderForce.y << " " << theCylinderForce.z << "  " <<
                "m_Length " << m_Length << " m_Velocity " << m_Velocity << " m_WrapStatus " << m_WrapStatus << "\n";
    }
    // now rotate back to word reference frame

    theOriginForce = QVRotate(qCylinderBody, QVRotate(m_CylinderQuaternion, theOriginForce));
    theInsertionForce = QVRotate(qCylinderBody, QVRotate(m_CylinderQuaternion, theInsertionForce));
    theCylinderForce = QVRotate(qCylinderBody, QVRotate(m_CylinderQuaternion, theCylinderForce));
    theCylinderForcePosition = QVRotate(m_CylinderQuaternion, theCylinderForcePosition) + m_CylinderPosition;
    theCylinderForcePosition = QVRotate(qCylinderBody, theCylinderForcePosition) + vCylinderBody;


    PointForce *theOrigin = m_PointForceList[0];
    PointForce *theInsertion = m_PointForceList[1];
    PointForce *theCylinder = m_PointForceList[2];
    theOrigin->vector[0] = theOriginForce.x; theOrigin->vector[1] = theOriginForce.y; theOrigin->vector[2] = theOriginForce.z;
    theOrigin->point[0] = worldOriginPosition.x; theOrigin->point[1] = worldOriginPosition.y; theOrigin->point[2] = worldOriginPosition.z;
    theInsertion->vector[0] = theInsertionForce.x; theInsertion->vector[1] = theInsertionForce.y; theInsertion->vector[2] = theInsertionForce.z;
    theInsertion->point[0] = worldInsertionPosition.x; theInsertion->point[1] = worldInsertionPosition.y; theInsertion->point[2] = worldInsertionPosition.z;
    theCylinder->vector[0] = theCylinderForce.x; theCylinder->vector[1] = theCylinderForce.y; theCylinder->vector[2] = theCylinderForce.z;
    theCylinder->point[0] = theCylinderForcePosition.x; theCylinder->point[1] = theCylinderForcePosition.y; theCylinder->point[2] = theCylinderForcePosition.z;

    // and handle the path coordinates
    for (i = 0; i < m_NumPathCoordinates; i++)
    {
        m_PathCoordinates[i] = QVRotate(m_CylinderQuaternion, m_PathCoordinates[i]) + m_CylinderPosition;
        m_PathCoordinates[i] = QVRotate(qCylinderBody, m_PathCoordinates[i]) + vCylinderBody;
    }

    if (gDebug == StrapDebug)
    {
        pgd::Vector totalF(0, 0, 0);
        for (i = 0; i < m_PointForceList.size(); i++)
        {
            *gDebugStream << "CylinderWrapStrap::Calculate " <<
                    *m_PointForceList[i]->body->GetName() << " " <<
                    m_PointForceList[i]->point[0] << " " << m_PointForceList[i]->point[1] << " " << m_PointForceList[i]->point[2] << " " <<
                    m_PointForceList[i]->vector[0] << " " << m_PointForceList[i]->vector[1] << " " << m_PointForceList[i]->vector[2] << "\n";
            totalF.x += m_PointForceList[i]->vector[0]; totalF.y += m_PointForceList[i]->vector[1]; totalF.z += m_PointForceList[i]->vector[2];
        }
        *gDebugStream << "Total F " << totalF.x << " " << totalF.y << " " << totalF.z << "\n";
    }
}

// function to wrap a line around a cylinder

// the cylinder is assumed to have its axis along the z axis and the wrapping is according
// to the right hand rule

// the coordinate system is right handed too

// returns -1 if a solution is impossible
// returns 0 if wrapping is unnecessary
// returns 1 if wrapping occurs
int CylinderWrap(pgd::Vector &origin, pgd::Vector &insertion, dReal radius, int nWrapSegments, dReal maxAngle,
                 pgd::Vector &originForce, pgd::Vector &insertionForce, pgd::Vector &cylinderForce, pgd::Vector &cylinderForcePosition,
                 dReal *pathLength, pgd::Vector *pathCoordinates, int *numPathCoordinates)
{
    // first of all calculate the planar case looking down the axis of the cylinder (i.e. xy plane)
    // this is standard tangent to a circle stuff

    // Let's define some variables:
    // origin.x: the x coordinate of the origin
    // origin.y: the y coordinate of the origin
    // d1: the distance of origin from the centre
    // l1: the distance of the origin from the circle along its tangent
    // theta1: the angle of the line from the centre to the origin
    // phi1:the angle at the centre formed by the origin triangle
    // radius: the radius of the circle

    dReal d1 = sqrt(origin.x*origin.x + origin.y*origin.y);
    // error condition
    if (d1 <= radius)
    {
        *numPathCoordinates = 0;
        return -1;
    }

    dReal theta1=atan2(origin.y, origin.x);

    dReal phi1=acos(radius/d1);

    dReal l1=d1 * sin(phi1);

    // The spherical coordinates of the tangent point are (radius, theta1+phi1)

    dReal tangentPointTheta1 = theta1 + phi1;

    dReal tangentPointX1 = radius * cos(tangentPointTheta1);

    dReal tangentPointY1 = radius * sin(tangentPointTheta1);

    // More variables:
    // insertion.x: the x coordinate of the insertion
    // insertion.y: the y coordinate of the inserttion
    // d2:the distance of the insertion from the centre
    // l2:the distance of the insertion from the centre
    // theta2:the angle of the line from the centre to the insertion
    // phi2:the angle at the centre formed by the insertion triangle

    dReal d2 = sqrt(insertion.x*insertion.x + insertion.y*insertion.y);
    // error condition
    if (d2 <= radius)
    {
        *numPathCoordinates = 0;
        return -1;
    }


    dReal theta2 = atan2(insertion.y, insertion.x);

    dReal phi2 = acos(radius/d2);

    dReal l2 = d2 * sin(phi2);

    // The spherical coordinates of the tangent point are (radius, theta2-phi2)

    dReal tangentPointTheta2 = theta2 - phi2;

    dReal tangentPointX2 = radius * cos(tangentPointTheta2);

    dReal tangentPointY2 = radius * sin(tangentPointTheta2);

    // rho: the angle around the circumference of the path in this plane
    // c:the distance around the circumference of the path in this plane
    dReal rho = tangentPointTheta2 - tangentPointTheta1;
    while (rho < 0)
        rho = rho + 2 * M_PI;
    while (rho > 2 * M_PI)
        rho = rho - 2 * M_PI;

    dReal c = radius * rho;

    // Finally we need to decide whether the path wraps at all.
    // We do this by seeing if the wrap angle is greater than a user specified limit.
    // Useful limits will be between M_PI and 2 M_PI

    if (rho > maxAngle)
    {
        // now calculate some forces
        originForce = insertion - origin;
        *pathLength = originForce.Magnitude();
        originForce.Normalize();

        insertionForce = -originForce;
        cylinderForce.x = cylinderForce.y = cylinderForce.z = 0;
        cylinderForcePosition.x = cylinderForcePosition.y = cylinderForcePosition.z = 0;

        // and a simple straight path
        if (pathCoordinates)
        {
            pathCoordinates[0] = origin;
            pathCoordinates[1] = insertion;
            *numPathCoordinates = 2;
        }

        return 0;
    }

    // OK, that's the x and y bits sorted. Now work out the z coordinates

    // The key point about the unwrapped problem is that the gradient is constant.
    // This means that the path around the cylinder is a helix that is continuous
    // with the line segments at each end.

    // origin.z: z coordinate of origin
    // insertion.z: z coordinate of origin
    // delz: height change
    // delz:horizontal distance
    // tangentPointZ1: origin tangent point
    // tangetPointZ2: insertion tangent point

    dReal delz = insertion.z - origin.z;

    dReal delx = l1 + c + l2;

    dReal tangentPointZ1 = origin.z + (l1/delx) * delz;

    dReal tangentPointZ2 = tangentPointZ1 + (c/delx) * delz;

    // now calculate some forces

    originForce.x = tangentPointX1 - origin.x;
    originForce.y = tangentPointY1 - origin.y;
    originForce.z = tangentPointZ1 - origin.z;
    originForce.Normalize();

    insertionForce.x = tangentPointX2 - insertion.x;
    insertionForce.y = tangentPointY2 - insertion.y;
    insertionForce.z = tangentPointZ2 - insertion.z;
    insertionForce.Normalize();

    cylinderForce = -originForce - insertionForce;

    cylinderForcePosition.x = 0;
    cylinderForcePosition.y = 0;
    cylinderForcePosition.z = (tangentPointZ1 + tangentPointZ2) / 2;

    *pathLength = sqrt(delx*delx + delz*delz);

    // that's the main calculations done.
    // Now check whether we need more points for drawing the path.
    int i;
    pgd::Vector *p = pathCoordinates;
    if (pathCoordinates)
    {
        *p = origin;
        p++;

        if (nWrapSegments > 0)
        {
            dReal delAngle = rho / (nWrapSegments);
            dReal angle = tangentPointTheta1;
            dReal delZ = (tangentPointZ2 - tangentPointZ1) / (nWrapSegments);
            dReal z = tangentPointZ1;
            for (i = 0; i < nWrapSegments; i++)
            {
                angle = angle + delAngle;
                p->x = radius * cos(angle);
                p->y = radius * sin(angle);
                z = z + delZ;
                p->z = z;
                p++;
            }
        }

        *p = insertion;
        *numPathCoordinates = nWrapSegments + 2;
    }

    return 1;
}

#ifdef USE_OPENGL
void CylinderWrapStrap::Draw()
{
    if (m_Visible == false || gDrawMuscles == false || m_NumPathCoordinates == 0) return;

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
        for (i = 0; i < m_NumPathCoordinates; i++)
        {
            polyline.push_back(m_PathCoordinates[i]);
        }
        facetedPolyline = new FacetedPolyline(&polyline, m_Radius, kSides);
        facetedPolyline->SetColour(m_Colour);
        facetedPolyline->SetUseDisplayList(true);
        m_DrawList.push_back(facetedPolyline);

        // calculate the quaternion that rotates from cylinder coordinates to world coordinates
        const dReal *q = dBodyGetQuaternion(m_CylinderBody->GetBodyID());
        pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
        pgd::Quaternion cylinderToWorldQuaternion =  qBody * m_CylinderQuaternion;
        pgd::Vector cylinderVecWorld = pgd::QVRotate(cylinderToWorldQuaternion, pgd::Vector(0, 0, m_CylinderLength / 2));
        // calculate the cylinder world position
        dVector3 position;
        dBodyGetRelPointPos(m_CylinderBody->GetBodyID(), m_CylinderPosition.x, m_CylinderPosition.y, m_CylinderPosition.z, position);
        // and draw it
        polyline.clear();
        polyline.push_back(pgd::Vector(position[0] - cylinderVecWorld.x, position[1] - cylinderVecWorld.y, position[2] - cylinderVecWorld.z));
        polyline.push_back(pgd::Vector(position[0] + cylinderVecWorld.x, position[1] + cylinderVecWorld.y, position[2] + cylinderVecWorld.z));
        facetedPolyline = new FacetedPolyline(&polyline, m_CylinderRadius, kSides);
        facetedPolyline->SetColour(m_CylinderColour);
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
