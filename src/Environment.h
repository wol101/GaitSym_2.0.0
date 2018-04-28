/*
 *  MyEnvironment.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat Mar 26 2005.
 *  Copyright (c) 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Environment_h
#define Environment_h

#include <vector>
#include "NamedObject.h"

class Geom;

class Environment: public NamedObject
{
public:
    Environment();
    ~Environment();
    
    void AddGeom(Geom *geom);
    
#ifdef USE_OPENGL
    void Draw();
#endif
    
protected:
    
    std::vector<Geom *> m_GeomList;
    
};


#endif



