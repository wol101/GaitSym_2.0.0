/*
 *  DataTargetQuaternion.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#ifndef DATATARGETQUATERNION_H
#define DATATARGETQUATERNION_H

#include "DataTarget.h"
#include "PGDMath.h"

class DataTargetQuaternion : public DataTarget
{
public:
    DataTargetQuaternion();
    ~DataTargetQuaternion();

    void SetTargetValues(int size, dReal *values);
    void SetTargetValues(const char *buf);
    virtual dReal GetMatchValue();

    virtual void Dump();

protected:

    pgd::Quaternion *m_ValueList;
    int m_ValueListLength;

};

#endif // DATATARGETQUATERNION_H
