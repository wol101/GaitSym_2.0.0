/*
 *  Body.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// this class is a wrapper for the ODE body

#include <iostream>
#include <string>
#include <string.h>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif

#include <ode/ode.h>

#include "Body.h"
#include "Simulation.h"
#include "PGDMath.h"
#include "Util.h"
#include "Marker.h"

#ifdef USE_OPENGL
#include "GLUtils.h"
#include "FacetedObject.h"
extern int gDrawBonesFlag;
extern int gAxisFlag;
extern std::string gOBJName;
#endif

// Simulation global
extern Simulation *gSimulation;

// length of vector a
#define LENGTHOF(a) \
        sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

#define LENGTH2OF(a) \
        (a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

#define OUTSIDERANGE(x, minX, maxX) \
        ((x) < (minX) || (x) > (maxX))

#define PSWAP(a,b,t) {t tmp; tmp=a; a=b; b=tmp;}


Body::Body(dWorldID worldID)
{
    m_BodyID = dBodyCreate (worldID);
    dBodySetData(m_BodyID, this);
    m_WorldID = worldID;

    for (int i = 0; i < 3; i++)
    {
        m_PositionLowBound[i] = -DBL_MAX;
        m_PositionHighBound[i] = DBL_MAX;
        m_LinearVelocityLowBound[i] = -DBL_MAX;
        m_LinearVelocityHighBound[i] = DBL_MAX;
    }

    SetOffset(0, 0, 0);

#ifdef USE_OPENGL
    m_FacetedObject = 0;
#endif
}

Body::~Body()
{
    dBodyDestroy (m_BodyID);

#ifdef USE_OPENGL
    if (m_FacetedObject) delete m_FacetedObject;
#endif
}

void Body::SetPosition(dReal x, dReal y, dReal z)
{
    dBodySetPosition(m_BodyID, x, y, z);
}

void Body::SetQuaternion(dReal q0, dReal q1, dReal q2, dReal q3)
{
    dQuaternion q;
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
    dBodySetQuaternion(m_BodyID, q);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
// bodyName x1 y1 z1 x2 y2 z2 - position such that x1,y1,z1 on bodyName has same world coordinates as x2,y2,z2 on local body
void Body::SetPosition(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);

    if (count == 1) // it must be a marker
    {
        Marker *marker = gSimulation->GetMarker(lBufPtrs[0]);
        if (marker == 0)
        {
            std::cerr << "Error in Body::SetPosition: Cannot find marker " << lBufPtrs[0] << "\n";
            return; // error condition
        }
        pgd::Vector wp = marker->GetWorldPosition();
        dBodySetPosition(m_BodyID, wp.x, wp.y, wp.z);
        return;
    }

    if (count == 2) // two marker definition
    {
        Marker *marker1 = gSimulation->GetMarker(lBufPtrs[0]);
        Marker *marker2 = gSimulation->GetMarker(lBufPtrs[1]);
        if (marker1 == 0 || marker2 == 0)
        {
            std::cerr << "Error in Body::SetPosition: Both markers must be defined " << lBufPtrs[0] << " " << lBufPtrs[1] << "\n";
            return; // error condition
        }
        // find the marker that is relative to this body and make it marker1
        if (marker2->GetBody() == m_BodyID) PSWAP(marker1, marker2, Marker *);
        if (marker2->GetBody() == m_BodyID)
        {
            std::cerr << "Error in Body::SetPosition: Only one marker of pair must be relative to the body \n";
            return; // error condition
        }
        if (marker1->GetBody() != m_BodyID)
        {
            std::cerr << "Error in Body::SetPosition: One marker of pair must be relative to the body \n";
            return; // error condition
        }

        pgd::Vector target = marker2->GetWorldPosition();
        pgd::Vector current = marker2->GetWorldPosition();
        pgd::Vector difference = target - current;
        dBodySetPosition(m_BodyID, difference.x, difference.y, difference.z);
        return;
    }

    if (count < 3)
    {
        std::cerr << "Error in Body::SetPosition\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodySetPosition(m_BodyID, pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in Body::SetPosition\n";
        return; // error condition
    }
    Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodySetPosition(m_BodyID, pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in Body::SetPosition\n";
            return; // error condition
        }
    }
    if (count < 7)
    {
        const dReal *p = theBody->GetPosition();
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0) + p[i];
        dBodySetPosition(m_BodyID, pos[0], pos[1], pos[2]);
    }
    else
    {
        // get world coordinates of x1,y1,z1
        dVector3 local1, world1, local2, world2;
        for (i = 0; i < 3; i++)
        {
            local1[i] = strtod(lBufPtrs[i + 1], 0);
            local2[i] = strtod(lBufPtrs[i + 4], 0);
        }
        dBodyGetRelPointPos (theBody->GetBodyID(), local1[0], local1[1], local1[2], world1);
        dBodyGetRelPointPos (m_BodyID, local2[0], local2[1], local2[2], world2);
        // add the error to the current position
        const dReal *p = dBodyGetPosition(m_BodyID);
        for (i = 0; i < 3; i++) pos[i] = p[i] + world1[i] - world2[i];
        dBodySetPosition(m_BodyID, pos[0], pos[1], pos[2]);

        // for checking
        // dBodyGetRelPointPos (m_BodyID, local2[0], local2[1], local2[2], world2);
        // for (i = 0; i < 3; i++) std::cerr << world1[i] << " " << world2[i] << "\n";
    }
}

// parses the quaternion allowing a relative position specified by BODY ID
// note quaternion is (qs,qx,qy,qz)
// s x y z - world coordinates
// bodyName s x y z - position relative to bodyName local coordinate system
void Body::SetQuaternion(const char *buf)
{
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dQuaternion quaternion;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);

    if (count == 1) // it must be a marker
    {
        Marker *marker = gSimulation->GetMarker(lBufPtrs[0]);
        if (marker == 0)
        {
            std::cerr << "Error in Body::SetQuaternion: Cannot find marker " << lBufPtrs[0] << "\n";
            return; // error condition
        }
        pgd::Quaternion wq = marker->GetWorldQuaternion();
        SetQuaternion(wq.n, wq.v.x, wq.v.y, wq.v.z);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in Body::SetQuaternion\n";
        return; // error condition
    }


    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        Util::GetQuaternion(&lBufPtrs[0], quaternion);
        dBodySetQuaternion(m_BodyID, quaternion);
        return;
    }

    if (count < 5)
    {
        std::cerr << "Error in Body::SetQuaternion\n";
        return; // error condition
    }
    Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            Util::GetQuaternion(&lBufPtrs[1], quaternion);
            dBodySetQuaternion(m_BodyID, quaternion);
            return;
        }
        else
        {
            std::cerr << "Error in Body::SetQuaternion\n";
            return; // error condition
        }
    }
    const dReal *q = theBody->GetQuaternion();
    pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
    Util::GetQuaternion(&lBufPtrs[1], quaternion);
    pgd::Quaternion qIn(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    pgd::Quaternion qNew = qIn * qBody;
    quaternion[0] = qNew.n;
    quaternion[1] = qNew.v.x;
    quaternion[2] = qNew.v.y;
    quaternion[3] = qNew.v.z;
    dBodySetQuaternion(m_BodyID, quaternion);
}

void Body::SetLinearVelocity(dReal x, dReal y, dReal z)
{
    dBodySetLinearVel(m_BodyID, x, y, z);
}

dReal Body::GetLinearKineticEnergy()
{
    // linear KE = 0.5 m v^2
    dMass mass;
    dBodyGetMass(m_BodyID, &mass);

    const dReal *v = dBodyGetLinearVel(m_BodyID);
    dReal linearKE = 0.5 * mass.mass * LENGTH2OF(v);

    return linearKE;
}

void Body::GetLinearKineticEnergy(dVector3 ke)
{
    // linear KE = 0.5 m v^2
    dMass mass;
    dBodyGetMass(m_BodyID, &mass);

    const dReal *v = dBodyGetLinearVel(m_BodyID);
    ke[0] =  0.5 * mass.mass * v[0] * v[0];
    ke[1] =  0.5 * mass.mass * v[1] * v[1];
    ke[2] =  0.5 * mass.mass * v[2] * v[2];

    return;
}

dReal Body::GetRotationalKineticEnergy()
{

    // rotational KE = 0.5 * o(t) * I * o
    // where o is rotational velocity vector and o(t) is the same but transposed

    dMass mass;
    dBodyGetMass(m_BodyID, &mass);

    const dReal *ow = dBodyGetAngularVel(m_BodyID);
    dVector3 o;
    dBodyVectorFromWorld (m_BodyID, ow[0], ow[1], ow[2], o);
    dVector3 o1;
    dMULTIPLYOP0_331(o1, =, mass.I, o);
    dReal rotationalKE = 0.5 * (o[0]*o1[0] + o[1]*o1[1] + o[2]*o1[2]);

    return rotationalKE;
}

dReal Body::GetGravitationalPotentialEnergy()
{
    dMass mass;
    dBodyGetMass(m_BodyID, &mass);
    dVector3 g;
    dWorldGetGravity (m_WorldID, g);
    const dReal *p = dBodyGetPosition(m_BodyID);

    // gravitational PE = mgh
    dReal gravitationalPotentialEnergy = - mass.mass * (g[0]*p[0] + g[1]*p[1] + g[2]*p[2]);

    return gravitationalPotentialEnergy;
}

void Body::SetAngularVelocity(dReal x, dReal y, dReal z)
{
    dBodySetAngularVel(m_BodyID, x, y, z);
}

void Body::SetMass(const dMass *mass)
{
    dBodySetMass(m_BodyID, mass);
}

const dReal *Body::GetPosition()
{
    return dBodyGetPosition(m_BodyID);
}

const dReal *Body::GetQuaternion()
{
    return dBodyGetQuaternion(m_BodyID);
}

const dReal *Body::GetRotation()
{
    return dBodyGetRotation (m_BodyID);
}

const dReal *Body::GetLinearVelocity()
{
    return dBodyGetLinearVel(m_BodyID);
}

const dReal *Body::GetAngularVelocity()
{
    return dBodyGetAngularVel(m_BodyID);
}

const dReal Body::GetMass()
{
    dMass mass;
    dBodyGetMass(m_BodyID, &mass);
    return mass.mass;
}


LimitTestResult Body::TestLimits()
{
    const dReal *p = dBodyGetPosition(m_BodyID);
    if (OUTSIDERANGE(p[0], m_PositionLowBound[0], m_PositionHighBound[0])) return XPosError;
    if (OUTSIDERANGE(p[1], m_PositionLowBound[1], m_PositionHighBound[1])) return YPosError;
    if (OUTSIDERANGE(p[2], m_PositionLowBound[2], m_PositionHighBound[2])) return ZPosError;

    const dReal *v = dBodyGetLinearVel(m_BodyID);
    if (OUTSIDERANGE(v[0], m_LinearVelocityLowBound[0], m_LinearVelocityHighBound[0])) return XVelError;
    if (OUTSIDERANGE(v[1], m_LinearVelocityLowBound[1], m_LinearVelocityHighBound[1])) return YVelError;
    if (OUTSIDERANGE(v[2], m_LinearVelocityLowBound[2], m_LinearVelocityHighBound[2])) return ZVelError;

    return WithinLimits;
}

void Body::Dump()
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
            *m_DumpStream << "Time\tXP\tYP\tZP\tXV\tYV\tZV\tQW\tQX\tQY\tQZ\tRVX\tRVY\tRVZ\tLKEX\tLKEY\tLKEZ\tRKE\tGPE\n";
        }
    }


    if (m_DumpStream)
    {
        const dReal *p = GetPosition();
        const dReal *v = GetLinearVelocity();
        const dReal *q = GetQuaternion();
        const dReal *rv = GetAngularVelocity();
        dVector3 ke;
        GetLinearKineticEnergy(ke);

        *m_DumpStream << gSimulation->GetTime() << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] <<
                "\t" << v[0] << "\t" << v[1] << "\t" << v[2] <<
                "\t" << q[0] << "\t" << q[1] << "\t" << q[2] << "\t" << q[3] <<
                "\t" << rv[0] << "\t" << rv[1] << "\t" << rv[2] <<
                "\t" << ke[0] << "\t" << ke[1] << "\t" << ke[2] <<
                "\t" << GetGravitationalPotentialEnergy() << "\t" << GetRotationalKineticEnergy() <<
                "\n";
    }
}

// a utility function to calculate moments of interia given an arbitrary translation and rotation
// assumes starting point is the moment of inertia at the centre of mass
#define _I(i,j) I[(i)*4+(j)]
void Body::ParallelAxis(dMass *massProperties, const dReal *translation, const dReal *quaternion, dMass *newMassProperties)
{
    dReal x, y, z; // transformation from centre of mass to new location (m)
    dReal mass; // mass (kg)
    dReal ixx,  iyy,  izz,  ixy,  iyz,  izx; // moments of inertia kgm2
    dReal ang; // rotation angle (radians)
    dReal ax, ay, az; // axis of rotation
    dReal ixxp, iyyp, izzp, ixyp, iyzp, izxp; // transformed moments of inertia about new coordinate system)

    x = translation[0];
    y = translation[1];
    z = translation[2];
    mass = massProperties->mass;
    ixx = massProperties->_I(0,0);
    iyy = massProperties->_I(1,1);
    izz = massProperties->_I(2,2);
    ixy = massProperties->_I(0,1);
    izx = massProperties->_I(0,2);
    iyz = massProperties->_I(1,2);

    ang = 2*acos(quaternion[0]);
    dReal magnitude = sqrt(SQUARE(quaternion[1]) + SQUARE(quaternion[2]) + SQUARE(quaternion[3]));
    if (magnitude <= 1e-10)
    {
        std::cerr << "Vector magnitude too low in Body::ParallelAxis\n";
    }
    ax = quaternion[1] / magnitude;
    ay = quaternion[2] / magnitude;
    az = quaternion[3] / magnitude;

    ParallelAxis(x, y, z, mass, ixx, iyy, izz, ixy, iyz, izx, ang, ax, ay, az, &ixxp, &iyyp, &izzp, &ixyp, &iyzp, &izxp);

    dMassSetParameters (newMassProperties, mass, 0, 0, 0, ixxp, iyyp, izzp, ixyp, izxp, iyzp);
}

// a utility function to calculate moments of interia given an arbitrary translation and rotation
void Body::ParallelAxis(dReal x, dReal y, dReal z, // transformation from centre of mass to new location (m)
                        dReal mass, // mass (kg)
                        dReal ixx, dReal iyy, dReal izz, dReal ixy, dReal iyz, dReal izx, // moments of inertia kgm2
                        dReal ang, // rotation angle (radians)
                        dReal ax, dReal ay, dReal az, // axis of rotation - must be unit length
                        dReal *ixxp, dReal *iyyp, dReal *izzp, dReal *ixyp, dReal *iyzp, dReal *izxp) // transformed moments of inertia about new coordinate system
{
    dReal cosang = cos(ang);
    dReal sinang = sin(ang);

    *ixxp = -(mass*(-(y*y) - (z*z))) + ((ax*ax)*(1 - cosang) + cosang)*
            (ixx*((ax*ax)*(1 - cosang) + cosang) + izx*(ax*az*(1 - cosang) + ay*sinang) +
             ixy*(ax*ay*(1 - cosang) - az*sinang)) + (ax*ay*(1 - cosang) - az*sinang)*
            (ixy*((ax*ax)*(1 - cosang) + cosang) + iyz*(ax*az*(1 - cosang) + ay*sinang) +
             iyy*(ax*ay*(1 - cosang) - az*sinang)) + (ax*az*(1 - cosang) + ay*sinang)*
            (izx*((ax*ax)*(1 - cosang) + cosang) + izz*(ax*az*(1 - cosang) + ay*sinang) +
             iyz*(ax*ay*(1 - cosang) - az*sinang));

    *iyyp = -(mass*(-(x*x) - (z*z))) + (ax*ay*(1 - cosang) + az*sinang)*
            (ixy*((ay*ay)*(1 - cosang) + cosang) + izx*(ay*az*(1 - cosang) - ax*sinang) +
             ixx*(ax*ay*(1 - cosang) + az*sinang)) + ((ay*ay)*(1 - cosang) + cosang)*
            (iyy*((ay*ay)*(1 - cosang) + cosang) + iyz*(ay*az*(1 - cosang) - ax*sinang) +
             ixy*(ax*ay*(1 - cosang) + az*sinang)) + (ay*az*(1 - cosang) - ax*sinang)*
            (iyz*((ay*ay)*(1 - cosang) + cosang) + izz*(ay*az*(1 - cosang) - ax*sinang) +
             izx*(ax*ay*(1 - cosang) + az*sinang));

    *izzp = -(mass*(-(x*x) - (y*y))) + (ax*az*(1 - cosang) - ay*sinang)*
            (izx*((az*az)*(1 - cosang) + cosang) + ixy*(ay*az*(1 - cosang) + ax*sinang) +
             ixx*(ax*az*(1 - cosang) - ay*sinang)) + (ay*az*(1 - cosang) + ax*sinang)*
            (iyz*((az*az)*(1 - cosang) + cosang) + iyy*(ay*az*(1 - cosang) + ax*sinang) +
             ixy*(ax*az*(1 - cosang) - ay*sinang)) + ((az*az)*(1 - cosang) + cosang)*
            (izz*((az*az)*(1 - cosang) + cosang) + iyz*(ay*az*(1 - cosang) + ax*sinang) +
             izx*(ax*az*(1 - cosang) - ay*sinang));

    *ixyp = -(mass*x*y) + (ax*ay*(1 - cosang) + az*sinang)*
            (ixx*((ax*ax)*(1 - cosang) + cosang) + izx*(ax*az*(1 - cosang) + ay*sinang) +
             ixy*(ax*ay*(1 - cosang) - az*sinang)) + ((ay*ay)*(1 - cosang) + cosang)*
            (ixy*((ax*ax)*(1 - cosang) + cosang) + iyz*(ax*az*(1 - cosang) + ay*sinang) +
             iyy*(ax*ay*(1 - cosang) - az*sinang)) + (ay*az*(1 - cosang) - ax*sinang)*
            (izx*((ax*ax)*(1 - cosang) + cosang) + izz*(ax*az*(1 - cosang) + ay*sinang) +
             iyz*(ax*ay*(1 - cosang) - az*sinang));

    *iyzp = -(mass*y*z) + (ax*az*(1 - cosang) - ay*sinang)*
            (ixy*((ay*ay)*(1 - cosang) + cosang) + izx*(ay*az*(1 - cosang) - ax*sinang) +
             ixx*(ax*ay*(1 - cosang) + az*sinang)) + (ay*az*(1 - cosang) + ax*sinang)*
            (iyy*((ay*ay)*(1 - cosang) + cosang) + iyz*(ay*az*(1 - cosang) - ax*sinang) +
             ixy*(ax*ay*(1 - cosang) + az*sinang)) + ((az*az)*(1 - cosang) + cosang)*
            (iyz*((ay*ay)*(1 - cosang) + cosang) + izz*(ay*az*(1 - cosang) - ax*sinang) +
             izx*(ax*ay*(1 - cosang) + az*sinang));

    *izxp = -(mass*x*z) + (ax*az*(1 - cosang) - ay*sinang)*
            (ixx*((ax*ax)*(1 - cosang) + cosang) + izx*(ax*az*(1 - cosang) + ay*sinang) +
             ixy*(ax*ay*(1 - cosang) - az*sinang)) + (ay*az*(1 - cosang) + ax*sinang)*
            (ixy*((ax*ax)*(1 - cosang) + cosang) + iyz*(ax*az*(1 - cosang) + ay*sinang) +
             iyy*(ax*ay*(1 - cosang) - az*sinang)) + ((az*az)*(1 - cosang) + cosang)*
            (izx*((ax*ax)*(1 - cosang) + cosang) + izz*(ax*az*(1 - cosang) + ay*sinang) +
             iyz*(ax*ay*(1 - cosang) - az*sinang));
}

#ifdef USE_OPENGL

void Body::Draw()
{
    if (m_Visible == false) return;

    gOBJName = m_Name;

    // get into body local coordinates
    const dReal *pos = dBodyGetPosition(m_BodyID);
    const dReal *R = dBodyGetRotation(m_BodyID);

    // from DrawStuff.cpp setTransform
    GLfloat matrix[16];
    matrix[0]=R[0];
    matrix[1]=R[4];
    matrix[2]=R[8];
    matrix[3]=0;
    matrix[4]=R[1];
    matrix[5]=R[5];
    matrix[6]=R[9];
    matrix[7]=0;
    matrix[8]=R[2];
    matrix[9]=R[6];
    matrix[10]=R[10];
    matrix[11]=0;
    matrix[12]=pos[0];
    matrix[13]=pos[1];
    matrix[14]=pos[2];
    matrix[15]=1;
    glPushMatrix();
    glMultMatrixf (matrix);

    // draw axes at origin (CM)
    if (gAxisFlag) GLUtils::DrawAxes(m_AxisSize[0], m_AxisSize[1], m_AxisSize[2]);
    glPopMatrix();


    if (gDrawBonesFlag)
    {
        // now do the drawing in local coordinates
        m_FacetedObject->SetColour(m_Colour);
        m_FacetedObject->SetDisplayRotation(R);
        m_FacetedObject->SetDisplayPosition(pos[0], pos[1], pos[2]);
        m_FacetedObject->Draw();
    }

}

void Body::SetFacetedObject(FacetedObject *obj)
{
    if (m_FacetedObject) delete m_FacetedObject;
    m_FacetedObject = obj;
    m_FacetedObject->SetUseDisplayList(true);
}

#endif
