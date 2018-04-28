/*
 *  Muscle.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#ifdef USE_OPENGL
#include "GLUtils.h"
extern int g_ActivationDisplay;
extern std::string gOBJName;
#endif

#include "Muscle.h"

Muscle::Muscle(Strap *strap)
{
    m_Strap = strap;
}


Muscle::~Muscle()
{
    delete m_Strap;
}

#ifdef USE_OPENGL
void Muscle::Draw()
{
    gOBJName = m_Name;
    if (g_ActivationDisplay)
    {
        Colour colour;
        GLUtils::SetColourFromMap(GetActivation(), SinColourMap, &colour);
        m_Strap->SetColour(colour);
        colour.alpha *= 0.5;
        m_Strap->SetForceColour(colour);
    }
    else
    {
        m_Strap->SetColour(m_Colour);
        m_Strap->SetForceColour(m_Colour);
    }
    m_Strap->Draw();
}
#endif
