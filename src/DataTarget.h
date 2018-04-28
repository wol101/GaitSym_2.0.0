/*
 *  DataTarget.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat May 22 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 */

#ifndef DataTarget_h
#define DataTarget_h

#include "NamedObject.h"

class Body;

class DataTarget: public NamedObject
{
public:
    DataTarget();
    ~DataTarget();

    enum MatchType { linear, square };

    void SetTarget(NamedObject *target) { m_Target = target; }
    NamedObject *GetTarget() { return m_Target; }

    void SetTargetTimes(int size, dReal *targetTimes);
    bool TargetMatch(dReal time);

    void SetWeight(dReal w) { m_Weight = w; }
    void SetSlope(dReal s) { m_Slope = s; }
    void SetMatchType(MatchType t) { m_MatchType = t; }
    void SetAbortThreshold(dReal a) { m_AbortThreshold = a; }

    virtual dReal GetMatchValue() = 0;

protected:

    NamedObject *m_Target;

    dReal m_Weight;
    dReal m_Slope;
    MatchType m_MatchType;
    dReal m_AbortThreshold;

    dReal *m_TargetTimeList;
    int m_TargetTimeListLength;
    int m_TargetTimeListIndex;


};

#endif

