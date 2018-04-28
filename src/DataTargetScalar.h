/*
 *  DataTargetScalar.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#ifndef DATATARGETSCALAR_H
#define DATATARGETSCALAR_H

#include "DataTarget.h"

class Body;

class DataTargetScalar: public DataTarget
{
public:
    DataTargetScalar();
    ~DataTargetScalar();

    enum DataType
    {
        XP,
        YP,
        ZP,
        Q0,
        Q1,
        Q2,
        Q3,
        XV,
        YV,
        ZV,
        XRV,
        YRV,
        ZRV,
        Angle,
        MetabolicEnergy,
        MechanicalEnergy
    };

    void SetDataType(DataType dataType) { m_DataType = dataType; }
    DataType GetDataType() { return m_DataType; }

    void SetTargetValues(int size, dReal *values);
    virtual dReal GetMatchValue();

    virtual void Dump();

protected:

    dReal PositiveFunction(dReal v);

    dReal *m_ValueList;
    int m_ValueListLength;
    DataType m_DataType;
};

#endif // DATATARGETSCALAR_H
