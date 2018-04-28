/*
 *  Marker.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 22/08/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

#ifndef Marker_h
#define Marker_h

#include "NamedObject.h"
#include "PGDMath.h"

struct MarkerInertia
{
    dReal I11;
    dReal I22;
    dReal I33;
    dReal I12;
    dReal I13;
    dReal I23;
};

class Marker: public NamedObject
{
public:

    Marker();
    virtual ~Marker();

    void SetBody(dBodyID body) { mBody = body; }
    dBodyID GetBody() { return mBody; }

    // these functions set the geom position relative to its body
    void SetPosition (dReal x, dReal y, dReal z)
    {
        mPosition.x = x; mPosition.y = y; mPosition.z = z;
    }
    void SetQuaternion(dReal q0, dReal q1, dReal q2, dReal q3)
    {
        mQuaternion.n = q0;
        mQuaternion.v.x = q1; mQuaternion.v.y = q2; mQuaternion.v.z = q3;
    }
    void SetPosition (const char *buf);
    void SetQuaternion(const char *buf);

    pgd::Vector GetPosition() { return mPosition; }
    pgd::Quaternion GetQuaternion() { return mQuaternion; }
    pgd::Vector GetWorldPosition();
    pgd::Quaternion GetWorldQuaternion();

    virtual void Dump();

#ifdef USE_OPENGL
    virtual void Draw();
    void SetRadius(dReal v) { mRadius = v; }
#endif

protected:

    dBodyID mBody;
    pgd::Vector mPosition;
    pgd::Quaternion mQuaternion;

#ifdef USE_OPENGL
    dReal mRadius;
#endif

};


#endif
