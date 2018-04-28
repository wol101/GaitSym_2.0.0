/*
 *  DataTarget.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat May 22 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>
#include <float.h>

#include <ode/ode.h>

#include "DataTarget.h"
#include "Util.h"

DataTarget::DataTarget()
{
    m_TargetTimeListLength = -1;
    m_TargetTimeListIndex = -1;
    m_TargetTimeList = 0;
    m_Weight = 1;
    m_Slope = 1;
    m_MatchType = linear;
    m_AbortThreshold = -DBL_MAX;
    m_Target = 0;
}

DataTarget::~DataTarget()
{
    if (m_TargetTimeList) delete [] m_TargetTimeList;
}

void DataTarget::SetTargetTimes(int size, dReal *targetTimes)
{
    int i;
    if (size <= 0)
    {
        std::cerr << "DataTarget::SetTargetTimes error: size = " << size << "\n";
        return;
    }
    if (m_TargetTimeListLength != size)
    {
        if (m_TargetTimeList) delete [] m_TargetTimeList;
        m_TargetTimeListLength = size;
        m_TargetTimeList = new dReal[m_TargetTimeListLength];
    }
    for (i = 0 ; i < m_TargetTimeListLength; i++)
    {
        m_TargetTimeList[i] = targetTimes[i];
    }
    m_TargetTimeListIndex = 0;
}

// returns true if a target match time is triggered
bool DataTarget::TargetMatch(dReal time)
{
    // test whether over the end of the
    if (m_TargetTimeListIndex >= m_TargetTimeListLength || m_TargetTimeListIndex < 0)
    {
        return false;
    }
    if (time < m_TargetTimeList[m_TargetTimeListIndex])
    {
        return false;
    }
    return true;
}

