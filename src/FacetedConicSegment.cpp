/*
 *  FacetedConicSegment.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 06/01/2006.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

#ifdef USE_OPENGL

#include <ode/ode.h>
#include <sstream>
#include <iomanip>
#include <math.h>

#include "FacetedConicSegment.h"
#include "Face.h"
#include "Util.h"

// create a conic segment structure with axis along z axis
// radii are specified by r1 and r2 and the origin is the centre of r1
// if r2 == 0 then draw a cone
// if r1 == r2 then draw a cylinder
FacetedConicSegment::FacetedConicSegment(dReal l, dReal r1, dReal r2, int sides, dReal ox, dReal oy, dReal oz): FacetedObject()
{
    m_R1 = r1;
    m_R2 = r2;
    m_Length = l;
    m_OX = ox;
    m_OY = oy;
    m_OZ = oz;
    m_Sides = sides;
    
    int i;
    dReal theta = 2 * M_PI / sides;
    Vertex vertex;
    Vertex *vertexPtr;
    Face *face;
    
    vertex.x = ox; 
    vertex.y = oy; 
    vertex.z = oz;
    AddVertex(vertex, true);
    vertex.z = l + oz;
    AddVertex(vertex, true);

    if (r2 != 0)
    {
        vertex.z = oz;
        for (i = 0; i < sides; i++)
        {
            vertex.x = r1 * cos(theta * i) + ox;
            vertex.y = r1 * sin(theta * i) + oy;
            AddVertex(vertex, true);
        }
        vertex.z = l + oz;
        if (r1 == r2)
        {
            for (i = 0; i < sides; i++)
            {
                vertexPtr = GetVertex(i + 2);
                vertex.x = vertexPtr->x;
                vertex.y = vertexPtr->y;
                AddVertex(vertex, true);
            }
        }
        else
        {
            for (i = 0; i < sides; i++)
            {
                vertex.x = r2 * cos(theta * i) + ox;
                vertex.y = r2 * sin(theta * i) + oy;
                AddVertex(vertex, true);
            }
        }
    
        // first end cap
        for (i = 0; i < sides; i++)
        {
            face = new Face();
            face->SetNumVertices(3);
            if (i < sides - 1)
            {
                face->SetVertex(0, 0);
                face->SetVertex(1, i + 2);
                face->SetVertex(2, i + 3);
            }
            else
            {
                face->SetVertex(0, 0);
                face->SetVertex(1, i + 2);
                face->SetVertex(2, 2);
            }
            AddFace(face);
        }
        
        // sides
        for (i = 0; i < sides; i++)
        {
            if (i < sides - 1)
            {
                face = new Face();
                face->SetNumVertices(3);
                face->SetVertex(0, i + 2);
                face->SetVertex(1, i + 2 + sides);
                face->SetVertex(2, i + 3 + sides);
                AddFace(face);
                face = new Face();
                face->SetNumVertices(3);
                face->SetVertex(0, i + 3 + sides);
                face->SetVertex(1, i + 3);
                face->SetVertex(2, i + 2);
                AddFace(face);
            }
            else
            {
                face = new Face();
                face->SetNumVertices(3);
                face->SetVertex(0, i + 2);
                face->SetVertex(1, i + 2 + sides);
                face->SetVertex(2, 2 + sides);
                AddFace(face);
                face = new Face();
                face->SetNumVertices(3);
                face->SetVertex(0, 2 + sides);
                face->SetVertex(1, 2);
                face->SetVertex(2, i + 2);
                AddFace(face);
            }
        }
    
        // final end cap
        for (i = 0; i < sides; i++)
        {
            face = new Face();
            face->SetNumVertices(3);
            if (i < sides - 1)
            {
                face->SetVertex(0, 1);
                face->SetVertex(1, i + 3 + sides);
                face->SetVertex(2, i + 2 + sides);
            }
            else
            {
                face->SetVertex(0, 1);
                face->SetVertex(1, 2 + sides);
                face->SetVertex(2, i + 2 + sides);
            }
            AddFace(face);
        }
    
    }
    else
    {
        // draw a cone with base radius r1
        
        // base vertices
        vertex.z = oz;
        for (i = 0; i < sides; i++)
        {
            vertex.x = r1 * cos(theta * i) + ox;
            vertex.y = r1 * sin(theta * i) + oy;
            AddVertex(vertex, true);
        }
        
        // base
        for (i = 0; i < sides; i++)
        {
            face = new Face();
            face->SetNumVertices(3);
            if (i < sides - 1)
            {
                face->SetVertex(0, 0);
                face->SetVertex(1, i + 2);
                face->SetVertex(2, i + 3);
            }
            else
            {
                face->SetVertex(0, 0);
                face->SetVertex(1, i + 2);
                face->SetVertex(2, 2);
            }
            AddFace(face);
        }
    
    
        // sides
        for (i = 0; i < sides; i++)
        {
            face = new Face();
            face->SetNumVertices(3);
            if (i < sides - 1)
            {
                face->SetVertex(0, i + 2);
                face->SetVertex(1, 1);
                face->SetVertex(2, i + 3);
            }
            else
            {
                face->SetVertex(0, i + 2);
                face->SetVertex(1, 1);
                face->SetVertex(2, 2);
            }
            AddFace(face);
        }
    }
    
    CalculateNormals();
}

// write the object out as a POVRay string
void FacetedConicSegment::WritePOVRay(std::ostringstream &theString)
{
    bool drawDisc = false;
    if (m_Length / ((m_R1 + m_R2) / 2) < 0.001) drawDisc = true;
    
    dVector3 prel, p;
    prel[0] = m_OX;
    prel[1] = m_OY;
    prel[2] = m_OZ;
    prel[3] = 0;
    dMULTIPLY0_331(p, m_DisplayRotation, prel);
    dReal bpx = p[0] + m_DisplayPosition[0];
    dReal bpy = p[1] + m_DisplayPosition[1];
    dReal bpz = p[2] + m_DisplayPosition[2];

    prel[0] = m_OX;
    prel[1] = m_OY;
    prel[2] = m_OZ + m_Length;
    prel[3] = 0;
    dMULTIPLY0_331(p, m_DisplayRotation, prel);
    dReal cpx = p[0] + m_DisplayPosition[0];
    dReal cpy = p[1] + m_DisplayPosition[1];
    dReal cpz = p[2] + m_DisplayPosition[2];

    if ((finite(bpx) && finite(bpy) && finite(bpz) && finite(cpx) && finite(cpy) && finite(cpz)) == false) return;
     
    theString << "object {\n";
    if (drawDisc)
    {
        theString << "  disc {\n";
        theString << "    <" << bpx << "," << bpy << "," << bpz << ">, <" 
                << cpx - bpx << "," << cpy - bpy << "," << cpz - bpz << ">, " << MAX(m_R1, m_R2) << "\n";
    }
    else
    {
        theString << "  cone {\n";
        theString << std::setprecision(17) << "    <" << bpx << "," << bpy << "," << bpz << ">, " << m_R1 << ", <" 
                << cpx << "," << cpy << "," << cpz << ">, " << m_R2 << "\n";
    }
    
    // now colour
    theString << "    pigment {\n";
    theString << "      color rgbf<" << m_Colour.r << "," << m_Colour.g << "," << m_Colour.b <<"," << 1 - m_Colour.alpha << ">\n";
    theString << "    }\n";
    
    theString << "  }\n";
    theString << "}\n\n";
    
}

#endif

