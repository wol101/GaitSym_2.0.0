/*
 *  FacetedPolyline.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 10/08/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

// Uses algorithms described in Geometric Tools for Computer Graphics, Scheider & Eberly 2003

#ifdef USE_OPENGL

#include <ode/ode.h>
#include <iostream>
#include <math.h>

#include "FacetedPolyline.h"
#include "Face.h"

FacetedPolyline::FacetedPolyline(std::vector<pgd::Vector> *polyline, dReal radius, int n)
{
    unsigned int i, j;
    std::vector<pgd::Vector> profile;
    std::vector<FPPolygon *> faces;

    // need to add extra tails to the polyline for direction padding
    std::vector<pgd::Vector> newPolyline;
    pgd::Vector v0 = (*polyline)[1] - (*polyline)[0];
    pgd::Vector v1 = (*polyline)[0] - v0;
    newPolyline.push_back(v1);
    for (i = 0; i < polyline->size(); i++) newPolyline.push_back((*polyline)[i]);
    v0 = (*polyline)[polyline->size() - 1] - (*polyline)[polyline->size() - 2];
    v1 = (*polyline)[polyline->size() - 1] + v0;
    newPolyline.push_back(v1);

    // create the profile
    dReal delTheta = 2 * M_PI / n;
    dReal theta = M_PI / 2;
    for (i = 0; i < n; i++)
    {
        v0.x = cos(theta) * radius;
        v0.y = sin(theta) * radius;
        v0.z = 0;
        theta -= delTheta;
        profile.push_back(v0);
    }

    Extrude(&newPolyline, &profile, &faces);

    // faces are mostly quadrilateral
    Vertex quadrilateral[4];
    Vertex *v;
    for (i = 0; i < faces.size(); i++)
    {
        if (faces[i]->vertices.size() == 4)
        {
            for (j = 0; j < 4; j++)
            {
                quadrilateral[j].x = faces[i]->vertices[j].x;
                quadrilateral[j].y = faces[i]->vertices[j].y;
                quadrilateral[j].z = faces[i]->vertices[j].z;
            }
            AddFace(quadrilateral, 4, true);
        }
        else
        {
            v = new Vertex[faces[i]->vertices.size()];
            for (j = 0; j < faces[i]->vertices.size(); j++)
            {
                v[j].x = faces[i]->vertices[j].x;
                v[j].y = faces[i]->vertices[j].y;
                v[j].z = faces[i]->vertices[j].z;
            }
            AddFace(v, faces[i]->vertices.size(), true);
            delete [] v;
        }
    }

    // and we need triangles
    Triangulate();

    SetDrawClockwise(false);
    CalculateNormals();

    for (i = 0; i < faces.size(); i++) delete faces[i];

}


// extrude profile along a poly line using sharp corners
// profile is a 2D shape with z = 0 for all values.
// polyline needs to have no parallel neighbouring segements
// clockwise winding assumed
// first and last point of polyline used for direction only!
void FacetedPolyline::Extrude(std::vector<pgd::Vector> *polyline, std::vector<pgd::Vector> *profile, std::vector<FPPolygon *> *faces)
{
    unsigned int i, j;
    Line3D line;
    pgd::Vector v1, v2, p1, p2;
    dReal epsilon = 0.000001;

    // define the planes of the joins
    std::vector<Plane3D> joinPlanes;
    Plane3D plane;
    for (i = 1; i < (*polyline).size() - 1; i++)
    {
        v1 = (*polyline)[i] - (*polyline)[i - 1];
        v2 = (*polyline)[i + 1] - (*polyline)[i];
        v1.Normalize();
        v2.Normalize();
        p1 = v1 - v2;
        if (p1.Magnitude() > epsilon) // not parallel so use two vector form of plane
        {
            p2 = v1 ^ v2;
            plane = Plane3D(&(*polyline)[i], &p1, &p2);
        }
        else // parallel so use normal-point form
        {
            plane = Plane3D(&(*polyline)[i], &v1);
        }

        joinPlanes.push_back(plane);
    }

    // define the rotated (*profile) for the first line segment
    // note for a truly generic routine you need two rotations to allow an up vector to be defined
    pgd::Vector zVec(0, 0, 1);
    v1 = (*polyline)[1] - (*polyline)[0];
    pgd::Quaternion q = pgd::FindRotation(zVec, v1);
    std::vector<pgd::Vector> rotatedProfile;
    for (i = 0; i < (*profile).size(); i++)
    {
        pgd::Vector v = pgd::QVRotate(q, (*profile)[i]);
        rotatedProfile.push_back(v);
    }

    // find the intersections on the join planes
    std::vector<pgd::Vector> vertexList;
    for (i = 0; i < rotatedProfile.size(); i++)
    {
        v2 = (*polyline)[0] + rotatedProfile[i];
        line = Line3D(&v2, &v1);
        for (j = 0; j < joinPlanes.size(); j++)
        {
            if (Intersection(&line, &joinPlanes[j], &v2))
            {
                vertexList.push_back(v2);
                if (j < joinPlanes.size() - 1)
                {
                    p1 = (*polyline)[j + 2] - (*polyline)[j + 1];
                    line = Line3D(&v2, &p1);
                }
            }
            else
            {
                std::cerr << "Error finding line & plane intersection" << std::endl;
                return;
            }
        }
    }

    // now construct faces
    FPPolygon *face;
    for (i = 0; i < rotatedProfile.size(); i++)
    {
        for (j = 0; j < joinPlanes.size() - 1; j++)
        {
            face = new FPPolygon();
            if (i < rotatedProfile.size() - 1)
            {
                face->vertices.push_back(vertexList[j + joinPlanes.size() * i]);
                face->vertices.push_back(vertexList[1 + j + joinPlanes.size() * i]);
                face->vertices.push_back(vertexList[1 + j + joinPlanes.size() * (i + 1)]);
                face->vertices.push_back(vertexList[j + joinPlanes.size() * (i + 1)]);
            }
            else
            {
                face->vertices.push_back(vertexList[j + joinPlanes.size() * i]);
                face->vertices.push_back(vertexList[1 + j + joinPlanes.size() * i]);
                face->vertices.push_back(vertexList[1 + j]);
                face->vertices.push_back(vertexList[j]);
            }

            faces->push_back(face);
        }
    }

    // end caps
    face = new FPPolygon();
    for (i = 0; i < rotatedProfile.size(); i++) face->vertices.push_back(vertexList[joinPlanes.size() * i]);
    faces->push_back(face);
    face = new FPPolygon();
    for (i = 0; i < rotatedProfile.size(); i++)
        face->vertices.push_back(vertexList[joinPlanes.size() - 1 + joinPlanes.size() * (rotatedProfile.size() - i - 1)]);
    faces->push_back(face);

}

// find intersection of line and plane
// returns true on success, false if no intersection
bool FacetedPolyline::Intersection(Line3D *line, Plane3D *plane, pgd::Vector *intersection)
{
    dReal denominator = line->direction * plane->GetNormal();
    dReal epsilon = 0.000001;
    dReal t;

    if (fabs(denominator) < epsilon)
    {
        // line and plane very close to parallel so they probably don't meet
        // but perhaps the origin is in the plane
        if (fabs(line->origin.x * plane->a + line->origin.y * plane->b + line->origin.z * plane->c + plane->d) > epsilon)
        {
            t = 0;
            *intersection = line->origin;
            return true;
        }
        else
        {
            return false;
        }
    }

    // compute intersection

    t = -(plane->a * line->origin.x + plane->b * line->origin.y + plane->c * line->origin.z + plane->d);
    t = t / denominator;
    *intersection = line->origin + t * line->direction;

    return true;

}

#endif

