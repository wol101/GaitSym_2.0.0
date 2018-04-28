/*
 *  NamedObject.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Root object that allows some basic data storage such as naming

#include "NamedObject.h"

NamedObject::NamedObject()
{
    m_Visible = true;
    m_Dump = false;
    m_FirstDump = true;
    m_DumpStream = 0;

#ifdef USE_OPENGL
    m_AxisSize[0] = m_AxisSize[1] = m_AxisSize[2] = 1;
    m_Colour.SetColour(1, 1, 1, 1);
#endif
}

NamedObject::~NamedObject()
{
    if (m_DumpStream) delete(m_DumpStream);
}

void NamedObject::Dump()
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
            *m_DumpStream << "Name\tm_Visible\n";
        }
    }

    if (m_DumpStream)
    {
        *m_DumpStream << m_Name << "\t" << m_Visible << "\n";
    }
}
