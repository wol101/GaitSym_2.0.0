/*
 *  Driver.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue Aug 04 2009.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 * Virtual class that all drivers descend from
 */

#include <ode/ode.h>

#include "Driver.h"
#include "Simulation.h"

// Simulation global
extern Simulation *gSimulation;

Driver::Driver()
{
    m_Target = 0;
}

Driver::~Driver()
{
}

void Driver::SetTarget(Muscle *muscle)
{
    if (muscle == 0)
    {
        std::cerr << "Error setting target for DRIVER " << m_Name << "\n";
        exit(1);
    }
    m_Target = muscle;
}

void Driver::Dump()
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
            *m_DumpStream << "Time\tValue\n";
        }
    }


    if (m_DumpStream)
    {
        *m_DumpStream << gSimulation->GetTime() << "\t" << GetValue(gSimulation->GetTime()) <<
                "\n";
    }
}
