/*
 *  StepDriver.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat May 22 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 * Uses a pure stepped function to return a the value at a given time
 *
 */

#include <iostream>

#include <ode/ode.h>

#include "StepDriver.h"
#include "Util.h"

StepDriver::StepDriver()
{
    m_ValueList = 0;
    m_DurationList = 0;
    m_ListLength = -1;
    m_LastIndex = 0;
}

StepDriver::~StepDriver()
{
    if (m_DurationList) delete [] m_DurationList;
    if (m_ValueList) delete [] m_ValueList;
}

// Note list is t0, v0, t1, v1, t2, v2 etc
// times are absolute simulation times not intervals
void StepDriver::SetValueDurationPairs(int size, dReal *valueDurationPairs)
{
    int i;
    if (size <= 0)
    {
        std::cerr << "CyclicDriver::SetValueDurationPairs error: size = " << size << "\n";
        return;
    }
    if (m_ListLength != size / 2)
    {
        if (m_DurationList) delete [] m_DurationList;
        if (m_ValueList) delete [] m_ValueList;
        m_ListLength = size / 2;
        m_DurationList = new dReal[m_ListLength];
        m_ValueList = new dReal[m_ListLength];
    }
    for (i = 0 ; i < m_ListLength; i++)
    {
        m_DurationList[i] = valueDurationPairs[i * 2];
        m_ValueList[i] = valueDurationPairs[i * 2 + 1];
    }
    m_LastIndex = 0;
}

// optimised assuming the integration step is smaller than the time
// interval (which it always should be)
dReal StepDriver::GetValue(dReal time)
{
    if (m_LastIndex >= m_ListLength - 1) return m_ValueList[m_ListLength - 1];
    if (time < m_DurationList[m_LastIndex + 1]) return m_ValueList[m_LastIndex];
    
    while (true)
    {
        m_LastIndex++;
        if (m_LastIndex >= m_ListLength - 1) return m_ValueList[m_ListLength - 1];
        if (time < m_DurationList[m_LastIndex + 1]) return m_ValueList[m_LastIndex];
    }
}


