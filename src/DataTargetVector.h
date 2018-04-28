/*
 *  DataTargetVector.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#ifndef DATATARGETVECTOR_H
#define DATATARGETVECTOR_H

#include "DataTarget.h"
#include "PGDMath.h"

class DataTargetVector : public DataTarget
{
public:
    DataTargetVector();
    ~DataTargetVector();

    void SetTargetValues(int size, dReal *values);
    void SetTargetValues(const char *buf);
    virtual dReal GetMatchValue();

    virtual void Dump();

protected:

    pgd::Vector *m_ValueList;
    int m_ValueListLength;

};

#endif // DATATARGETVECTOR_H
