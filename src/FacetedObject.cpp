/*
 *  FacettedObject.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 13/09/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <vector>
#include <string>
#include <sstream>

#include <ode/ode.h>

#if defined(USE_OPENGL)
#include <gl.h>
#endif

#include "FacetedObject.h"
#include "Face.h"
#include "DataFile.h"
#include "DebugControl.h"
#include "Util.h"

bool gDestinationOpenGL = true;
bool gDestinationPOVRay = false;
std::ofstream *gPOVRayFile = 0;
bool gDestinationOBJFile = false;
std::ofstream *gOBJFile = 0;

std::string gOBJName;
int gVertexOffset = 0;

// create object
FacetedObject::FacetedObject()
{
    mNumVertices = 0;
    mNumVerticesAllocated = 0;
    mNumFaces = 0;
    mNumFacesAllocated = 0;
    mVertexList = 0;
    mFaceList = 0;
    mDrawClockwise = false; // according to OpenGL red book anticlockwise winding is the default
    m_VertexMatchTolerance = 0.0000000001;
    mBadMesh = false;
    m_UseRelativeOBJ = false;

    memset(m_DisplayPosition, 0, sizeof(dVector3));
    dRSetIdentity(m_DisplayRotation);

#ifdef USE_OPENGL
    m_DisplayListIndex = 0;
    m_ValidDisplayList = false;
    m_UseDisplayList = false;
#endif
}

// destroy object
FacetedObject::~FacetedObject()
{
    ClearLists();
}

// parse an OBJ file to a FacetedObject
// returns true on error
bool FacetedObject::ParseOBJFile(const char *filename)
{
    DataFile theFile;
    if (theFile.ReadFile(filename)) return true;

    return (ParseOBJFile(theFile));
}

// parse an OBJ file to a FacetedObject
// returns true on error
bool FacetedObject::ParseOBJFile(DataFile &theFile)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    const int kBufferSize = 64000;
    char *line = new char[kBufferSize];
    char *buffer = new char[kBufferSize];
    char **tokens = new char *[kBufferSize];
    int numTokens;
    int count = 0;
    std::vector<Vertex *> vertexList;
    std::vector<Face *> faceList;
    Vertex *vertex;
    Face *face;
    int i;
    Vertex min = {DBL_MAX, DBL_MAX, DBL_MAX};
    Vertex max = {-DBL_MAX, -DBL_MAX, -DBL_MAX};

    // read the file
    while (theFile.ReadNextLine(line, kBufferSize, true, '#', '\\') == false)
    {
        strcpy(buffer, line);
        count++;
        numTokens = DataFile::ReturnTokens(buffer, tokens, kBufferSize);

        // vertices
        if (strcmp(tokens[0], "v") == 0)
        {
            if (numTokens > 3)
            {
                vertex = new Vertex();
                vertex->x = atof(tokens[1]);
                vertex->y = atof(tokens[2]);
                vertex->z = atof(tokens[3]);
                vertexList.push_back(vertex);

                if (gDebug == FacetedObjectDebug)
                {
                    min.x = MIN(min.x, vertex->x);
                    min.y = MIN(min.y, vertex->y);
                    min.z = MIN(min.z, vertex->z);
                    max.x = MAX(max.x, vertex->x);
                    max.y = MAX(max.y, vertex->y);
                    max.z = MAX(max.z, vertex->z);
                }
            }
        }

        // faces
        if (strcmp(tokens[0], "f") == 0)
        {
            if (numTokens > 3)
            {
                face = new Face();
                face->SetNumVertices(numTokens - 1);
                // note obj files start at 1 not zero
                for (i = 1; i < numTokens; i++)
                    face->SetVertex(i - 1, atoi(tokens[i]) - 1);
                faceList.push_back(face);
            }
        }
    }

    if (gDebug == FacetedObjectDebug)
        std::cerr << "ParseOBJFile:\tmin.x\t" << min.x << "\tmax.x\t" << max.x <<
                "\tmin.y\t" << min.y << "\tmax.y\t" << max.y <<
                "\tmin.z\t" << min.z << "\tmax.z\t" << max.z << "\n";

    // fill out the display object
    ClearLists();
    for (i = 0; i < (int)vertexList.size(); i++)
    {
        AddVertex(*vertexList[i]);
    }

    for (i = 0; i < (int)faceList.size(); i++)
    {
        AddFace(faceList[i]);
    }

    // calculate normals
    CalculateNormals();

    // clear memory
    for (i = 0; i < (int)vertexList.size(); i++)
        delete vertexList[i];

    delete [] line;
    delete [] buffer;
    delete [] tokens;

    return false;
}

// write the object out as a POVRay string
// currently assumes all faces are triangles (call Triangulate if conversion is necessary)
void FacetedObject::WritePOVRay(std::ostringstream &theString)
{
    int i, j;
    int nVertices;
    Vertex *vPtr;
    dVector3 prel, p, result;

    theString.precision(7); // should be plenty

    theString << "object {\n";
    theString << "  mesh {\n";

    // first faces
    for (i = 0; i < mNumFaces; i++)
    {
        nVertices = mFaceList[i]->GetNumVertices();
        if (nVertices == 3)
        {
            theString << "    triangle {\n";
            for (j = 0; j < nVertices; j++)
            {
                vPtr = mVertexList[mFaceList[i]->GetVertex(j)];
                prel[0] = vPtr->x;
                prel[1] = vPtr->y;
                prel[2] = vPtr->z;
                prel[3] = 0;
                dMULTIPLY0_331(p, m_DisplayRotation, prel);
                result[0] = p[0] + m_DisplayPosition[0];
                result[1] = p[1] + m_DisplayPosition[1];
                result[2] = p[2] + m_DisplayPosition[2];

                theString << "      <" << result[0] << "," << result[1] << "," << result[2] << ">\n";
            }
            theString << "    }\n";
        }
    }
#ifdef USE_OPENGL
    // now colour
    theString << "    pigment {\n";
    theString << "      color rgbf<" << m_Colour.r << "," << m_Colour.g << "," << m_Colour.b <<"," << 1 - m_Colour.alpha << ">\n";
    theString << "    }\n";
#endif
    theString << "  }\n";
    theString << "}\n\n";
}

// Write a FacetedObject out as a OBJ
void FacetedObject::WriteOBJFile(std::ostringstream &out)
{
    int i, j;
    Vertex *vPtr;
    dVector3 prel, p, result;
    static unsigned long counter = 0;

    out.precision(7); // should be plenty

    for (i = 0; i < gOBJName.size(); i++)
        if (gOBJName[i] <= ' ') gOBJName[i] = '_';
    out << "o " << gOBJName << counter << "\n";
    counter++;

    if (m_UseRelativeOBJ)
    {
        // write out the vertices, faces, groups and objects
        // this is the relative version - inefficient but allows concatenation of objects
        for (i = 0; i < this->GetNumFaces(); i++)
        {
            for (j = 0; j < this->GetFace(i)->GetNumVertices(); j++)
            {
                vPtr = mVertexList[mFaceList[i]->GetVertex(j)];
                prel[0] = vPtr->x;
                prel[1] = vPtr->y;
                prel[2] = vPtr->z;
                prel[3] = 0;
                dMULTIPLY0_331(p, m_DisplayRotation, prel);
                result[0] = p[0] + m_DisplayPosition[0];
                result[1] = p[1] + m_DisplayPosition[1];
                result[2] = p[2] + m_DisplayPosition[2];
                out << "v " << result[0] << " " << result[1] << " " << result[2] << "\n";
            }

            out << "f ";
            for (j = 0; j < this->GetFace(i)->GetNumVertices(); j++)
            {
                if (j == this->GetFace(i)->GetNumVertices() - 1)
                    out << j - this->GetFace(i)->GetNumVertices() << "\n";
                else
                    out << j - this->GetFace(i)->GetNumVertices() << " ";
            }
        }
    }
    else
    {
        for (i = 0; i < this->GetNumVertices(); i++)
        {
            vPtr = this->GetVertex(i);
            prel[0] = vPtr->x;
            prel[1] = vPtr->y;
            prel[2] = vPtr->z;
            prel[3] = 0;
            dMULTIPLY0_331(p, m_DisplayRotation, prel);
            result[0] = p[0] + m_DisplayPosition[0];
            result[1] = p[1] + m_DisplayPosition[1];
            result[2] = p[2] + m_DisplayPosition[2];
            out << "v " << result[0] << " " << result[1] << " " << result[2] << "\n";
        }

        for (i = 0; i < this->GetNumFaces(); i++)
        {
            out << "f ";
            for (j = 0; j < this->GetFace(i)->GetNumVertices(); j++)
            {
                // note this files vertex list start at 1 not zero
                if (j == this->GetFace(i)->GetNumVertices() - 1)
                    out << this->GetFace(i)->GetVertex(j) + 1 + gVertexOffset << "\n";
                else
                    out << this->GetFace(i)->GetVertex(j) + 1 + gVertexOffset << " ";
            }
        }
        gVertexOffset += this->GetNumVertices();
    }
}

// clear the vertex and face lists
void FacetedObject::ClearLists()
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
    if (m_DisplayListIndex) glDeleteLists(m_DisplayListIndex, 1);
#endif
    int i;
    if (mVertexList)
    {
        for (i = 0; i < mNumVertices; i++) delete mVertexList[i];
        delete [] mVertexList;
        mVertexList = 0;
        mNumVertices = 0;
        mNumVerticesAllocated = 0;
    }
    if (mFaceList)
    {
        for (i = 0; i < mNumFaces; i++) delete mFaceList[i];
        delete [] mFaceList;
        mFaceList = 0;
        mNumFaces = 0;
        mNumFacesAllocated = 0;
    }
}

void FacetedObject::Draw()
{
    if (m_Visible == false) return;

    if (gDestinationOpenGL)
    {
#ifdef USE_OPENGL
        int i, j;
        int nVertices;
        GLfloat normal[3];
        Vertex *vPtr;

        GLfloat matrix[16];
        matrix[0]=m_DisplayRotation[0];
        matrix[1]=m_DisplayRotation[4];
        matrix[2]=m_DisplayRotation[8];
        matrix[3]=0;
        matrix[4]=m_DisplayRotation[1];
        matrix[5]=m_DisplayRotation[5];
        matrix[6]=m_DisplayRotation[9];
        matrix[7]=0;
        matrix[8]=m_DisplayRotation[2];
        matrix[9]=m_DisplayRotation[6];
        matrix[10]=m_DisplayRotation[10];
        matrix[11]=0;
        matrix[12]=m_DisplayPosition[0];
        matrix[13]=m_DisplayPosition[1];
        matrix[14]=m_DisplayPosition[2];
        matrix[15]=1;
        glPushMatrix();
        glMultMatrixf(matrix);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // from DrawStuff.cpp setColor
        GLfloat r = m_Colour.r;
        GLfloat g = m_Colour.g;
        GLfloat b = m_Colour.b;
        GLfloat alpha = m_Colour.alpha;
        GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
        light_ambient[0] = r*0.3f;
        light_ambient[1] = g*0.3f;
        light_ambient[2] = b*0.3f;
        light_ambient[3] = alpha;
        light_diffuse[0] = r*0.7f;
        light_diffuse[1] = g*0.7f;
        light_diffuse[2] = b*0.7f;
        light_diffuse[3] = alpha;
        light_specular[0] = r*0.2f;
        light_specular[1] = g*0.2f;
        light_specular[2] = b*0.2f;
        light_specular[3] = alpha;
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
        glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
        glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
        glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 5.0f);

        // from DrawStuff.cpp setupDrawingMode
        if (alpha < 1.0)
        {
            glEnable (GL_BLEND);
            glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        }
        else
        {
            glDisable (GL_BLEND);
        }

        if (m_ValidDisplayList && m_UseDisplayList)
        {
            glCallList(m_DisplayListIndex);
        }
        else
        {
            if (m_UseDisplayList)
            {
                if (m_DisplayListIndex) glDeleteLists(m_DisplayListIndex, 1);
                m_DisplayListIndex = glGenLists(1);
                if (m_DisplayListIndex != 0)
                {
                    glNewList(m_DisplayListIndex, GL_COMPILE);
                }
            }



            for (i = 0; i < mNumFaces; i++)
            {

                if (mDrawClockwise)
                {
                    nVertices = mFaceList[i]->GetNumVertices();
                    if (nVertices < 3) throw (__LINE__);

                    glBegin(GL_POLYGON);
                    mFaceList[i]->GetNormal(normal);
                    glNormal3fv(normal);

                    for (j = nVertices - 1; j >= 0; j--)
                    {
                        vPtr = mVertexList[mFaceList[i]->GetVertex(j)];
                        glVertex3f(vPtr->x, vPtr->y, vPtr->z);
                    }
                    glEnd();
                    if (mBadMesh)
                    {
                        glBegin(GL_POLYGON);
                        mFaceList[i]->GetNormal(normal);
                        normal[0] = -normal[0]; normal[1] = -normal[1]; normal[2] = -normal[2];
                        glNormal3fv(normal);

                        for (j = 0; j < nVertices; j++)
                        {
                            vPtr = mVertexList[mFaceList[i]->GetVertex(j)];
                            glVertex3f(vPtr->x, vPtr->y, vPtr->z);
                        }
                        glEnd();
                    }
                }
                else
                {
                    nVertices = mFaceList[i]->GetNumVertices();
                    if (nVertices < 3) throw (__LINE__);

                    glBegin(GL_POLYGON);
                    mFaceList[i]->GetNormal(normal);
                    glNormal3fv(normal);

                    for (j = 0; j < nVertices; j++)
                    {
                        vPtr = mVertexList[mFaceList[i]->GetVertex(j)];
                        glVertex3f(vPtr->x, vPtr->y, vPtr->z);
                    }
                    glEnd();
                    if (mBadMesh)
                    {
                        glBegin(GL_POLYGON);
                        mFaceList[i]->GetNormal(normal);
                        normal[0] = -normal[0]; normal[1] = -normal[1]; normal[2] = -normal[2];
                        glNormal3fv(normal);

                        for (j = nVertices - 1; j >= 0; j--)
                        {
                            vPtr = mVertexList[mFaceList[i]->GetVertex(j)];
                            glVertex3f(vPtr->x, vPtr->y, vPtr->z);
                        }
                        glEnd();
                    }
                }
            }

            if (m_UseDisplayList)
            {
                if (m_DisplayListIndex != 0)
                {
                    glEndList();
                    glCallList(m_DisplayListIndex);
                    m_ValidDisplayList = true;
                }
            }
        }

        glPopMatrix();
#endif
    }

    else if (gDestinationPOVRay)
    {
        std::ostringstream theString;
        WritePOVRay(theString);
        if (gPOVRayFile) (*gPOVRayFile) << theString.str();
        else std::cout << theString.str();
    }

    else if (gDestinationOBJFile)
    {
        std::ostringstream theString;
        WriteOBJFile(theString);
        if (gOBJFile) (*gOBJFile) << theString.str();
        else std::cout << theString.str();
    }
}

void FacetedObject::SetDisplayPosition(dReal x, dReal y, dReal z)
{
    m_DisplayPosition[0] = x;
    m_DisplayPosition[1] = y;
    m_DisplayPosition[2] = z;
}

void FacetedObject::SetDisplayRotation(const dMatrix3 R, bool fast)
{
    if (fast)
    {
        memcpy(m_DisplayRotation, R, sizeof(dMatrix3));
    }
    else
    {
        dQuaternion q;
        dRtoQ (R, q);
        dNormalize4 (q);
        dQtoR (q, m_DisplayRotation);
    }
}

void FacetedObject::SetDisplayRotationFromQuaternion(const dQuaternion q, bool fast)
{
    if (fast == false)
    {
        dQuaternion qq;
        memcpy(qq, q, sizeof(dQuaternion));
        dNormalize4 (qq);
        dQtoR(qq, m_DisplayRotation);
    }
    else
        dQtoR(q, m_DisplayRotation);
}

void FacetedObject::SetDisplayRotationFromAxis(dReal x, dReal y, dReal z, bool fast)
{
    // calculate the rotation needed to get the axis pointing the right way
    dVector3 axis;
    axis[0] = x;
    axis[1] = y;
    axis[2] = z;
    if (fast == false) dNormalize3(axis);
    dVector3 p, q;
    // calculate 2 perpendicular vectors
    dPlaneSpace(axis, p, q);
    // assemble the matrix
    m_DisplayRotation[3] = m_DisplayRotation[7] = m_DisplayRotation[11] = 0;

    m_DisplayRotation[0] =    p[0]; m_DisplayRotation[4] =    p[1]; m_DisplayRotation[8] =     p[2];
    m_DisplayRotation[1] =    q[0]; m_DisplayRotation[5] =    q[1]; m_DisplayRotation[9] =     q[2];
    m_DisplayRotation[2] = axis[0]; m_DisplayRotation[6] = axis[1]; m_DisplayRotation[10] = axis[2];
}

// utility to calculate a face normal
// this assumes anticlockwise winding
void FacetedObject::ComputeFaceNormal(const Vertex *v1,
                                      const Vertex *v2, const Vertex *v3, dReal normal[3])
{
    dReal a[3], b[3];

    // calculate in plane vectors
    a[0] = v2->x - v1->x;
    a[1] = v2->y - v1->y;
    a[2] = v2->z - v1->z;
    b[0] = v3->x - v1->x;
    b[1] = v3->y - v1->y;
    b[2] = v3->z - v1->z;

    // cross(a, b, normal);
    normal[0] = a[1] * b[2] - a[2] * b[1];
    normal[1] = a[2] * b[0] - a[0] * b[2];
    normal[2] = a[0] * b[1] - a[1] * b[0];

    // normalize(normal);
    dReal norm = sqrt(normal[0] * normal[0] +
                      normal[1] * normal[1] +
                      normal[2] * normal[2]);

    if (norm > 0.0)
    {
        normal[0] /= norm;
        normal[1] /= norm;
        normal[2] /= norm;
    }
}

// Calculate all the normals
void FacetedObject::CalculateNormals()
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;
    Face *face;
    dReal normal[3];

    for (i = 0; i < mNumFaces; i++)
    {
        face = mFaceList[i];

        if (face->GetNumVertices() > 2)
        {
            ComputeFaceNormal(mVertexList[face->GetVertex(0)],
                              mVertexList[face->GetVertex(1)],
                              mVertexList[face->GetVertex(2)],
                              normal);
            if (mDrawClockwise)
                face->SetNormal(normal[0], normal[1], normal[2]);
            else
                face->SetNormal(-normal[0], -normal[1], -normal[2]);
        }
    }
}

// concatenate another faceted object
void FacetedObject::Concatenate(FacetedObject *object)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;
    Vertex **oldVertexList = mVertexList;
    Face **oldFaceList = mFaceList;
    int oldNumVertices = mNumVertices;
    int oldNumFaces = mNumFaces;
    int objectNumVertices = object->GetNumVertices();
    int objectNumFaces = object->GetNumFaces();

    // copy over old lists
    mNumVertices = oldNumVertices + objectNumVertices;
    mVertexList = new Vertex *[mNumVertices];
    for (i = 0; i < oldNumVertices; i++) mVertexList[i] = oldVertexList[i];
    if (oldVertexList) delete [] oldVertexList;

    mNumFaces = oldNumFaces + objectNumFaces;
    mFaceList = new Face *[mNumFaces];
    for (i = 0; i < oldNumFaces; i++) mFaceList[i] = oldFaceList[i];
    if (oldFaceList) delete [] oldFaceList;

    // copy vertices
    for (i = oldNumVertices; i < mNumVertices; i++)
    {
        mVertexList[i] = new Vertex();
        *mVertexList[i] = *object->GetVertex(i - oldNumVertices);
    }

    // copy faces offsetting vertex numbers
    for (i = oldNumFaces; i < mNumFaces; i++)
    {
        mFaceList[i] = new Face();
        *mFaceList[i] = *object->GetFace(i - oldNumFaces);
        mFaceList[i]->OffsetVertices(oldNumVertices);
    }
}

// move the object
void FacetedObject::Move(dReal x, dReal y, dReal z)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;

    for (i = 0; i < mNumVertices; i++)
    {
        mVertexList[i]->x += x;
        mVertexList[i]->y += y;
        mVertexList[i]->z += z;
    }
}

// scale the object
void FacetedObject::Scale(dReal x, dReal y, dReal z)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;

    for (i = 0; i < mNumVertices; i++)
    {
        mVertexList[i]->x *= x;
        mVertexList[i]->y *= y;
        mVertexList[i]->z *= z;
    }
}

// swap axes of the object
void FacetedObject::SwapAxes(int axis1, int axis2)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;
    if (axis1 < 0 || axis1 > 2 ||
        axis2 < 0 || axis2 > 2)
    {
        std::cerr << "FacetedObject::SwapAxes axis out of range\n";
        exit(1);
    }

    for (i = 0; i < mNumVertices; i++)
    {
        if (axis1 == 0 && axis2 == 1) SWAP(mVertexList[i]->x, mVertexList[i]->y);
        if (axis1 == 0 && axis2 == 2) SWAP(mVertexList[i]->x, mVertexList[i]->z);
        if (axis1 == 1 && axis2 == 2) SWAP(mVertexList[i]->y, mVertexList[i]->z);
        if (axis1 == 1 && axis2 == 0) SWAP(mVertexList[i]->y, mVertexList[i]->x);
        if (axis1 == 2 && axis2 == 0) SWAP(mVertexList[i]->z, mVertexList[i]->x);
        if (axis1 == 2 && axis2 == 1) SWAP(mVertexList[i]->z, mVertexList[i]->y);
    }
}

// rotate axes of the object
void FacetedObject::RotateAxes(int axis0, int axis1, int axis2)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;
    dReal t0, t1, t2;
    if (axis0 < 0 || axis0 > 2 ||
        axis1 < 0 || axis1 > 2 ||
        axis2 < 0 || axis2 > 2)
    {
        std::cerr << "FacetedObject::RotateAxes axis out of range\n";
        exit(1);
    }

    for (i = 0; i < mNumVertices; i++)
    {
        if (axis0 == 0) t0 = mVertexList[i]->x;
        if (axis0 == 1) t0 = mVertexList[i]->y;
        if (axis0 == 2) t0 = mVertexList[i]->z;
        if (axis1 == 0) t1 = mVertexList[i]->x;
        if (axis1 == 1) t1 = mVertexList[i]->y;
        if (axis1 == 2) t1 = mVertexList[i]->z;
        if (axis2 == 0) t2 = mVertexList[i]->x;
        if (axis2 == 1) t2 = mVertexList[i]->y;
        if (axis2 == 2) t2 = mVertexList[i]->z;
        mVertexList[i]->x = t0;
        mVertexList[i]->y = t1;
        mVertexList[i]->z = t2;
    }
}

// mirror the object
void FacetedObject::Mirror(bool x, bool y, bool z)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;
    int actions = 0;

    if (x)
    {
        for (i = 0; i < mNumVertices; i++) mVertexList[i]->x =  -mVertexList[i]->x;
        actions++;
    }

    if (y)
    {
        for (i = 0; i < mNumVertices; i++) mVertexList[i]->y =  -mVertexList[i]->y;
        actions++;
    }

    if (z)
    {
        for (i = 0; i < mNumVertices; i++) mVertexList[i]->z =  -mVertexList[i]->z;
        actions++;
    }

    // if we have an odd number of mirroring operations we need to
    // reverse the order that the faces are defined
    if (ODD(actions))
    {
        for (i = 0; i < mNumFaces; i++) mFaceList[i]->ReverseVertexOrder();
    }

    // if we've done anything then we need to recalculate the normals
    if (actions) CalculateNormals();
}

// print out stats on the object
void FacetedObject::Stats()
{
    int i;
    Vertex min = {DBL_MAX, DBL_MAX, DBL_MAX};
    Vertex max = {-DBL_MAX, -DBL_MAX, -DBL_MAX};

    std::cout << "FacettedObject: " << m_Name << "\n";
    std::cout << "Vertices: " << mNumVertices << "\n";
    std::cout << "Faces: " << mNumFaces << "\n";

    for (i = 0; i < mNumVertices; i++)
    {
        min.x = MIN(min.x, mVertexList[i]->x);
        min.y = MIN(min.y, mVertexList[i]->y);
        min.z = MIN(min.z, mVertexList[i]->z);
        max.x = MAX(max.x, mVertexList[i]->x);
        max.y = MAX(max.y, mVertexList[i]->y);
        max.z = MAX(max.z, mVertexList[i]->z);
    }
    std::cout << "Vertex extents (" << min.x << ", " << min.y << ", " << min.z <<
            ") to (" << max.x << ", " << max.y << ", " << max.z << ")\n";

    std::cout.flush();
}

// simple routine to convert convex polygons to triangles
void FacetedObject::Triangulate()
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;
    int numVertices;
    int numNewFaces = 0;
    int currentNewFace = 0;
    int v0, v1, v2;
    Face **newFaceList;

    if (mNumFaces == 0) return; // nothing to do

    // count the number of faces after triangulation
    for (i = 0; i < mNumFaces; i++)
    {
        numVertices = mFaceList[i]->GetNumVertices();
        if (numVertices >= 3) numNewFaces += (numVertices - 2);
    }
    if (numNewFaces == mNumFaces) return; // nothing to do

    if (numNewFaces == 0) // no triangular faces
    {
        for (i = 0; i < mNumFaces; i++) delete mFaceList[i];
        delete [] mFaceList;
        mFaceList = 0;
        mNumFaces = 0;
        return;
    }

    // allocate the new faces
    newFaceList = new Face *[numNewFaces];
    for (i = 0; i < numNewFaces; i++) newFaceList[i] = new Face();

    // do the triangulation
    for (i = 0; i < mNumFaces; i++)
    {
        numVertices = mFaceList[i]->GetNumVertices();
        if (numVertices == 3)
        {
            *newFaceList[currentNewFace] = *mFaceList[i];
            currentNewFace++;
        }
        else
        {
            if (numVertices > 3)
            {
                v0 = 0;
                for (v2 = 2; v2 < numVertices; v2++)
                {
                    v1 = v2 - 1;
                    newFaceList[currentNewFace]->SetNumVertices(3);
                    newFaceList[currentNewFace]->SetVertex(0, mFaceList[i]->GetVertex(v0));
                    newFaceList[currentNewFace]->SetVertex(1, mFaceList[i]->GetVertex(v1));
                    newFaceList[currentNewFace]->SetVertex(2, mFaceList[i]->GetVertex(v2));
                    currentNewFace++;
                }
            }
        }
    }

    // and change over the face list
    for (i = 0; i < mNumFaces; i++) delete mFaceList[i];
    delete [] mFaceList;
    mFaceList = newFaceList;
    mNumFaces = numNewFaces;
}

// add a face to the object
// adds vertices
void FacetedObject::AddFace(Vertex vertex[], int numVertices, bool fast)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;
    int *vertexIndices = new int[numVertices];
    Face *face = new Face();

    for (i = 0; i < numVertices; i++)
        vertexIndices[i] = AddVertex(vertex[i], fast);

    face->SetNumVertices(numVertices);
    for (i = 0; i < numVertices; i++)
        face->SetVertex(i, vertexIndices[i]);

    AddFace(face);
    delete [] vertexIndices;
}

// add a face to the object
// assumes all vertex indices are correct
// takes over ownership of the pointer
void FacetedObject::AddFace(Face *face)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;
    if (mNumFaces >= mNumFacesAllocated)
    {
        mNumFacesAllocated += kAllocationIncrement;
        Face **newFaceList = new Face *[mNumFacesAllocated];
        if (mFaceList)
        {
            for (i = 0; i < mNumFaces; i++)
                newFaceList[i] = mFaceList[i];
            delete [] mFaceList;
        }
        mFaceList = newFaceList;
    }
    mFaceList[mNumFaces] = face;
    mNumFaces++;
}

// add a vertex to an object
// if fast is true it doesn' try to match with existing vertices
int FacetedObject::AddVertex(Vertex vertex, bool fast)
{
#ifdef USE_OPENGL
    m_ValidDisplayList = false;
#endif
    int i;
    // try to match existing vertex
    if (fast == false)
    {
        for (i = 0; i < mNumVertices; i++)
        {
            if (fabs(vertex.x - mVertexList[i]->x) < m_VertexMatchTolerance &&
                fabs(vertex.y - mVertexList[i]->y) < m_VertexMatchTolerance &&
                fabs(vertex.z - mVertexList[i]->z) < m_VertexMatchTolerance ) return i;
        }
    }

    Vertex *newVertex = new Vertex();
    *newVertex = vertex;
    if (mNumVertices >= mNumVerticesAllocated)
    {
        mNumVerticesAllocated += kAllocationIncrement;
        Vertex **newVertexList = new Vertex *[mNumVerticesAllocated];
        if (mVertexList)
        {
            for (i = 0; i < mNumVertices; i++)
                newVertexList[i] = mVertexList[i];
            delete [] mVertexList;
        }
        mVertexList = newVertexList;
    }
    mVertexList[mNumVertices] = newVertex;
    mNumVertices++;
    return mNumVertices - 1;
}

// return an ODE style trimesh
// note memory is allocated by this routine and will need to be released elsewhere
void FacetedObject::CalculateTrimesh(double **vertices, int *numVertices, int *vertexStride, dTriIndex **triIndexes, int *numTriIndexes, int *triStride)
{
    int i;
    *vertexStride = 3 * sizeof(double);
    *triStride = 3 * sizeof(dTriIndex);

    Triangulate();
    *numVertices = mNumVertices;
    *numTriIndexes = mNumFaces * 3;

    *vertices = new double[mNumVertices * 3];
    *triIndexes = new dTriIndex[mNumFaces * 3];

    for (i = 0; i < mNumVertices; i++)
    {
        (*vertices)[i * 3] = mVertexList[i]->x;
        (*vertices)[i * 3 + 1] = mVertexList[i]->y;
        (*vertices)[i * 3 + 2] = mVertexList[i]->z;
    }

    for (i = 0; i < mNumFaces; i++)
    {
        (*triIndexes)[i * 3] = mFaceList[i]->GetVertex(0);
        (*triIndexes)[i * 3 + 1] = mFaceList[i]->GetVertex(1);
        (*triIndexes)[i * 3 + 2] = mFaceList[i]->GetVertex(2);
    }
}

// return an ODE style trimesh
// note memory is allocated by this routine and will need to be released elsewhere
void FacetedObject::CalculateTrimesh(float **vertices, int *numVertices, int *vertexStride, dTriIndex **triIndexes, int *numTriIndexes, int *triStride)
{
    int i;
    *vertexStride = 3 * sizeof(float);
    *triStride = 3 * sizeof(dTriIndex);

    Triangulate();
    *numVertices = mNumVertices;
    *numTriIndexes = mNumFaces * 3;

    *vertices = new float[mNumVertices * 3];
    *triIndexes = new dTriIndex[mNumFaces * 3];

    for (i = 0; i < mNumVertices; i++)
    {
        (*vertices)[i * 3] = mVertexList[i]->x;
        (*vertices)[i * 3 + 1] = mVertexList[i]->y;
        (*vertices)[i * 3 + 2] = mVertexList[i]->z;
    }

    for (i = 0; i < mNumFaces; i++)
    {
        (*triIndexes)[i * 3] = mFaceList[i]->GetVertex(0);
        (*triIndexes)[i * 3 + 1] = mFaceList[i]->GetVertex(1);
        (*triIndexes)[i * 3 + 2] = mFaceList[i]->GetVertex(2);
    }
}

// calculate mass properties
// based on dMassSetTrimesh
/*
 * dMassSetTrimesh, implementation by Gero Mueller.
 * Based on Brian Mirtich, "Fast and Accurate Computation of
 * Polyhedral Mass Properties," journal of graphics tools, volume 1,
 * number 2, 1996.
 */

#define	SQR(x)			((x)*(x))						//!< Returns x square
#define	CUBE(x)			((x)*(x)*(x))					//!< Returns x cube

#define _I(i,j) I[(i)*4+(j)]

void FacetedObject::CalculateMassProperties(dMass *m, dReal density, bool clockwise)
{
    dMassSetZero (m);

    Triangulate();

    // assumes anticlockwise winding so need to reverse if anticlockwise
    if (clockwise == true) for (int index = 0; index < mNumFaces; index++) mFaceList[index]->ReverseVertexOrder();

    unsigned int triangles = mNumFaces;

    dReal nx, ny, nz;
    unsigned int i, j, A, B, C;
    // face integrals
    dReal Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

    // projection integrals
    dReal P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

    dReal T0 = 0;
    dReal T1[3] = {0., 0., 0.};
    dReal T2[3] = {0., 0., 0.};
    dReal TP[3] = {0., 0., 0.};

    dVector3 v[3];
    for( i = 0; i < triangles; i++ )
    {
        for (j = 0; j < 3; j++)
        {
            v[j][0] = mVertexList[mFaceList[i]->GetVertex(j)]->x;
            v[j][1] = mVertexList[mFaceList[i]->GetVertex(j)]->y;
            v[j][2] = mVertexList[mFaceList[i]->GetVertex(j)]->z;
        }

        dVector3 n, a, b;
        dOP( a, -, v[1], v[0] );
        dOP( b, -, v[2], v[0] );
        dCROSS( n, =, b, a );
        nx = fabs(n[0]);
        ny = fabs(n[1]);
        nz = fabs(n[2]);

        if( nx > ny && nx > nz )
            C = 0;
        else
            C = (ny > nz) ? 1 : 2;

        // Even though all triangles might be initially valid,
        // a triangle may degenerate into a segment after applying
        // space transformation.
        if (n[C] != REAL(0.0))
        {
            A = (C + 1) % 3;
            B = (A + 1) % 3;

            // calculate face integrals
            {
                dReal w;
                dReal k1, k2, k3, k4;

                //compProjectionIntegrals(f);
                {
                    dReal a0, a1, da;
                    dReal b0, b1, db;
                    dReal a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
                    dReal a1_2, a1_3, b1_2, b1_3;
                    dReal C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
                    dReal Cab, Kab, Caab, Kaab, Cabb, Kabb;

                    P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

                    for( j = 0; j < 3; j++)
                    {
                        switch(j)
                        {
                                                        case 0:
                            a0 = v[0][A];
                            b0 = v[0][B];
                            a1 = v[1][A];
                            b1 = v[1][B];
                            break;
                                                        case 1:
                            a0 = v[1][A];
                            b0 = v[1][B];
                            a1 = v[2][A];
                            b1 = v[2][B];
                            break;
                                                        case 2:
                            a0 = v[2][A];
                            b0 = v[2][B];
                            a1 = v[0][A];
                            b1 = v[0][B];
                            break;
                        }
                        da = a1 - a0;
                        db = b1 - b0;
                        a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
                        b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
                        a1_2 = a1 * a1; a1_3 = a1_2 * a1;
                        b1_2 = b1 * b1; b1_3 = b1_2 * b1;

                        C1 = a1 + a0;
                        Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
                        Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
                        Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
                        Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
                        Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
                        Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

                        P1 += db*C1;
                        Pa += db*Ca;
                        Paa += db*Caa;
                        Paaa += db*Caaa;
                        Pb += da*Cb;
                        Pbb += da*Cbb;
                        Pbbb += da*Cbbb;
                        Pab += db*(b1*Cab + b0*Kab);
                        Paab += db*(b1*Caab + b0*Kaab);
                        Pabb += da*(a1*Cabb + a0*Kabb);
                    }

                    P1 /= 2.0;
                    Pa /= 6.0;
                    Paa /= 12.0;
                    Paaa /= 20.0;
                    Pb /= -6.0;
                    Pbb /= -12.0;
                    Pbbb /= -20.0;
                    Pab /= 24.0;
                    Paab /= 60.0;
                    Pabb /= -60.0;
                }

                w = - dDOT(n, v[0]);

                k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

                Fa = k1 * Pa;
                Fb = k1 * Pb;
                Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

                Faa = k1 * Paa;
                Fbb = k1 * Pbb;
                Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb +
                            w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

                Faaa = k1 * Paaa;
                Fbbb = k1 * Pbbb;
                Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab
                              + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
                              + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
                              + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

                Faab = k1 * Paab;
                Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
                Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
                             + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
            }


            T0 += n[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

            T1[A] += n[A] * Faa;
            T1[B] += n[B] * Fbb;
            T1[C] += n[C] * Fcc;
            T2[A] += n[A] * Faaa;
            T2[B] += n[B] * Fbbb;
            T2[C] += n[C] * Fccc;
            TP[A] += n[A] * Faab;
            TP[B] += n[B] * Fbbc;
            TP[C] += n[C] * Fcca;
        }
    }

    T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
    T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
    TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;

    m->mass = density * T0;
    m->_I(0,0) = density * (T2[1] + T2[2]);
    m->_I(1,1) = density * (T2[2] + T2[0]);
    m->_I(2,2) = density * (T2[0] + T2[1]);
    m->_I(0,1) = - density * TP[0];
    m->_I(1,0) = - density * TP[0];
    m->_I(2,1) = - density * TP[1];
    m->_I(1,2) = - density * TP[1];
    m->_I(2,0) = - density * TP[2];
    m->_I(0,2) = - density * TP[2];

    m->c[0] = T1[0] / T0;
    m->c[1] = T1[1] / T0;
    m->c[2] = T1[2] / T0;

    // assumes anticlockwise winding so need to reverse if clockwise
    // puts everything back as we found it
    if (clockwise == true) for (int index = 0; index < mNumFaces; index++) mFaceList[index]->ReverseVertexOrder();


}

// reverse the face winding
void FacetedObject::ReverseWinding()
{
    for (int index = 0; index < mNumFaces; index++) mFaceList[index]->ReverseVertexOrder();
}

