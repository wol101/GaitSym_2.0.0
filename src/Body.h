/*
 *  Body.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// this class is a wrapper for the ODE body

#ifndef Body_h
#define Body_h

#ifdef USE_OPENGL
#include <gl.h>
#endif

#include "NamedObject.h"

class FacetedObject;

enum LimitTestResult
{
    WithinLimits = 0,
    XPosError = 1,
    YPosError = 2,
    ZPosError = 3,
    XVelError = 4,
    YVelError = 5,
    ZVelError = 6
};

class Body: public NamedObject
{
public:

    Body(dWorldID worldID);
    ~Body();

    void SetPosition(dReal x, dReal y, dReal z);
    void SetQuaternion(dReal q0, dReal q1, dReal q2, dReal q3);
    void SetPosition(const char *buf);
    void SetQuaternion(const char *buf);
    void SetLinearVelocity(dReal x, dReal y, dReal z);
    void SetAngularVelocity(dReal x, dReal y, dReal z);

    void SetMass(const dMass *mass);

    void SetPositionLowBound(dReal x, dReal y, dReal z) { m_PositionLowBound[0] = x; m_PositionLowBound[1] = y; m_PositionLowBound[2] = z; };
    void SetPositionHighBound(dReal x, dReal y, dReal z) { m_PositionHighBound[0] = x; m_PositionHighBound[1] = y; m_PositionHighBound[2] = z; };
    void SetLinearVelocityLowBound(dReal x, dReal y, dReal z) { m_LinearVelocityLowBound[0] = x; m_LinearVelocityLowBound[1] = y; m_LinearVelocityLowBound[2] = z; };
    void SetLinearVelocityHighBound(dReal x, dReal y, dReal z) { m_LinearVelocityHighBound[0] = x; m_LinearVelocityHighBound[1] = y; m_LinearVelocityHighBound[2] = z; };

    const dReal *GetPosition();
    const dReal *GetQuaternion();
    const dReal *GetRotation();
    const dReal *GetLinearVelocity();
    const dReal *GetAngularVelocity();
    const dReal GetMass();
    dReal GetLinearKineticEnergy();
    void GetLinearKineticEnergy(dVector3 ke);
    dReal GetRotationalKineticEnergy();
    dReal GetGravitationalPotentialEnergy();

    dBodyID GetBodyID() { return m_BodyID; };

    LimitTestResult TestLimits();

    // Utility
    void ParallelAxis(dMass *massProperties, const dReal *translation, const dReal *quaternion, dMass *newMassProperties);
    static void ParallelAxis(dReal x, dReal y, dReal z, // transformation from centre of mass to new location (m)
                             dReal mass, // mass (kg)
                             dReal ixx, dReal iyy, dReal izz, dReal ixy, dReal iyz, dReal izx, // moments of inertia kgm2
                             dReal ang, // rotation angle (radians)
                             dReal ax, dReal ay, dReal az, // axis of rotation - must be unit length
                             dReal *ixxp, dReal *iyyp, dReal *izzp, dReal *ixyp, dReal *iyzp, dReal *izxp); // transformed moments of inertia about new coordinate system

    // these values are really only used for graphics
    void SetOffset(dReal x, dReal y, dReal z) {m_Offset[0] = x; m_Offset[1] = y; m_Offset[2] = z; };
    const dReal *GetOffset() { return m_Offset; };

    virtual void Dump();

#ifdef USE_OPENGL
    void Draw();
    void SetFacetedObject(FacetedObject *obj);
    FacetedObject *GetFacetedObject() { return m_FacetedObject; };
#endif

protected:

    dWorldID m_WorldID;
    dBodyID m_BodyID;

    dVector3 m_PositionLowBound;
    dVector3 m_PositionHighBound;
    dVector3 m_LinearVelocityLowBound;
    dVector3 m_LinearVelocityHighBound;

    dVector3 m_Offset;

#ifdef USE_OPENGL
    FacetedObject *m_FacetedObject;
#endif
};

#endif
