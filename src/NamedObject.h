/*
 *  NamedObject.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Root object that allows some basic data storage such as naming

#ifndef NamedObject_h
#define NamedObject_h

#include <string>
#include <iostream>
#include <fstream>

#ifdef USE_OPENGL
#include "GLUtils.h"
#endif

class NamedObject
{
public:

    NamedObject();
    virtual ~NamedObject();

    void SetName(const char* name) { m_Name = name; };
    void SetName(const std::string &name) { m_Name = name; };
    std::string *GetName() { return &m_Name; };

    bool GetVisible() { return m_Visible; };
    void SetVisible(bool v) { m_Visible = v; };

    bool GetDump() { return m_Dump; };
    void SetDump(bool v) { m_Dump = v; };

    virtual void Dump();

#ifdef USE_OPENGL
    void SetAxisSize(GLfloat axisSize[3]) {m_AxisSize[0] = axisSize[0]; m_AxisSize[1] = axisSize[1]; m_AxisSize[2] = axisSize[2]; };
    void SetColour(Colour &colour) { m_Colour = colour; };
    const GLfloat *GetAxisSize() { return m_AxisSize; };
    const Colour *GetColour() { return &m_Colour; };
#endif

protected:

    std::string m_Name;

    bool m_Visible;

    bool m_Dump;
    bool m_FirstDump;
    std::ofstream *m_DumpStream;

#ifdef USE_OPENGL
    GLfloat m_AxisSize[3];
    Colour m_Colour;
#endif
};

#endif
