/*
 *  FacetedSphere.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 03/01/2006.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

// this code largely based on code by Jon Leech from:
// http://www.gamedev.net/reference/articles/article427.asp
/*
 * sphere - generate a polygon mesh approximating a sphere by
 *  recursive subdivision. First approximation is an octahedron;
 *  each level of refinement increases the number of polygons by
 *  a factor of 4.
 * Level 3 (128 polygons) is a good tradeoff if gouraud
 *  shading is used to render the database.
 *
 * Usage: sphere [level]
 *      level is an integer >= 1 setting the recursion level (default 1).
 *
 * Notes:
 *
 *  The triangles are generated with vertices in clockwise order as
 *  viewed from the outside in a right-handed coordinate system.
 *  To reverse the order, compile with COUNTERCLOCKWISE defined.
 *
 *  Shared vertices are not retained, so numerical errors may produce
 *  cracks between polygons at high subdivision levels.
 *
 *  The subroutines print_object() and print_triangle() should
 *  be changed to generate whatever the desired database format is.
 *  If UNC is defined, a PPHIGS text archive is generated.
 *
 * Jon Leech 3/24/89
 */

#ifdef USE_OPENGL

//#define COUNTERCLOCKWISE

#include <stdio.h>
#include <math.h>
#include <ode/ode.h>
#include <sstream>

#include "FacetedSphere.h"
#include "Face.h"

typedef struct 
{
    double  x, y, z;
} point;

typedef struct 
{
    point     pt[3];    /* Vertices of triangle */
    double    area;     /* Unused; might be used for adaptive subdivision */
} triangle;

typedef struct 
{
    int       npoly;    /* # of polygons in object */
    triangle *poly;     /* Polygons in no particular order */
} object;

/* Six equidistant points lying on the unit sphere */
#define XPLUS {  1,  0,  0 }    /*  X */
#define XMIN  { -1,  0,  0 }    /* -X */
#define YPLUS {  0,  1,  0 }    /*  Y */
#define YMIN  {  0, -1,  0 }    /* -Y */
#define ZPLUS {  0,  0,  1 }    /*  Z */
#define ZMIN  {  0,  0, -1 }    /* -Z */

/* Forward declarations */
static point *normalize( point *p );
static point *midpoint( point *a, point *b );
static void print_object( object *obj, int level );
static void print_triangle( triangle *t );
static double sqr( double x );
static double area_of_triangle( triangle *t );

FacetedSphere::FacetedSphere(dReal radius, int maxlevels, bool fast): FacetedObject()
{
    /* Vertices of a unit octahedron */
    triangle octahedron[] = 
    {
        {{ XPLUS, ZPLUS, YPLUS }, 0.0},
        {{ YPLUS, ZPLUS, XMIN  }, 0.0},
        {{ XMIN , ZPLUS, YMIN  }, 0.0},
        {{ YMIN , ZPLUS, XPLUS }, 0.0},
        {{ XPLUS, YPLUS, ZMIN  }, 0.0},
        {{ YPLUS, XMIN , ZMIN  }, 0.0},
        {{ XMIN , YMIN , ZMIN  }, 0.0},
        {{ YMIN , XPLUS, ZMIN  }, 0.0}
    };

    /* An octahedron */
    object oct = 
    {
        sizeof(octahedron) / sizeof(octahedron[0]),
        &octahedron[0]
    };

    object *oldObject, *newObject;
    int     i, level;
    
    if (maxlevels < 1) maxlevels = 1;
    m_Level = maxlevels;
    m_Radius = radius;
    
#ifdef COUNTERCLOCKWISE
    /* Reverse order of points in each triangle */
    for (i = 0; i < oct.npoly; i++) {
        point tmp;
        tmp = oct.poly[i].pt[0];
        oct.poly[i].pt[0] = oct.poly[i].pt[2];
        oct.poly[i].pt[2] = tmp;
    }
    SetDrawClockwise(false);
#else
    SetDrawClockwise(true);
#endif
    
    oldObject = &oct;
    
    /* Subdivide each starting triangle (maxlevels - 1) times */
    for (level = 1; level < maxlevels; level++) 
    {
        /* Allocate a new object */
        newObject = (object *)malloc(sizeof(object));
        if (newObject == NULL) 
        {
            fprintf(stderr, "FacetedSphere: Out of memory on subdivision level %d\n",
                    level);
            exit(1);
        }
        newObject->npoly = oldObject->npoly * 4;
        
        /* Allocate 4* the number of points in the current approximation */
        newObject->poly  = (triangle *)malloc(newObject->npoly * sizeof(triangle));
        if (newObject->poly == NULL) 
        {
            fprintf(stderr, "FacetedSphere: Out of memory on subdivision level %d\n",
                    level);
            exit(1);
        }
        
        /* Subdivide each polygon in the old approximation and normalize
            *  the new points thus generated to lie on the surface of the unit
            *  sphere.
            * Each input triangle with vertices labelled [0,1,2] as shown
            *  below will be turned into four new triangles:
            *
            *                      Make new points
            *                          a = (0+2)/2
            *                          b = (0+1)/2
            *                          c = (1+2)/2
            *        1
            *       /\             Normalize a, b, c
            *      /  \
            *    b/____\ c         Construct new triangles
            *    /\    /\              [0,b,a]
            *   /  \  /  \             [b,1,c]
            *  /____\/____\            [a,b,c]
            * 0      a     2           [a,c,2]
            */
        for (i = 0; i < oldObject->npoly; i++) 
        {
            triangle *oldt = &oldObject->poly[i];
            triangle *newt = &newObject->poly[i*4];
            point a, b, c;
            
            a = *normalize(midpoint(&oldt->pt[0], &oldt->pt[2]));
            b = *normalize(midpoint(&oldt->pt[0], &oldt->pt[1]));
            c = *normalize(midpoint(&oldt->pt[1], &oldt->pt[2]));
            
            newt->pt[0] = oldt->pt[0];
            newt->pt[1] = b;
            newt->pt[2] = a;
            newt++;
            
            newt->pt[0] = b;
            newt->pt[1] = oldt->pt[1];
            newt->pt[2] = c;
            newt++;
            
            newt->pt[0] = a;
            newt->pt[1] = b;
            newt->pt[2] = c;
            newt++;
            
            newt->pt[0] = a;
            newt->pt[1] = c;
            newt->pt[2] = oldt->pt[2];
        }
        
        if (level > 1) {
            free(oldObject->poly);
            free(oldObject);
        }
        
        /* Continue subdividing new triangles */
        oldObject = newObject;
    }
    
    /* Print out resulting approximation */
    //print_object(oldObject, maxlevels);
    
    // add the faces to the Faceted Object
    Vertex vertex[3];
    for (i = 0; i < oldObject->npoly; i++)
    {
        vertex[0].x = oldObject->poly[i].pt[0].x; vertex[0].y = oldObject->poly[i].pt[0].y; vertex[0].z = oldObject->poly[i].pt[0].z;
        vertex[1].x = oldObject->poly[i].pt[1].x; vertex[1].y = oldObject->poly[i].pt[1].y; vertex[1].z = oldObject->poly[i].pt[1].z;
        vertex[2].x = oldObject->poly[i].pt[2].x; vertex[2].y = oldObject->poly[i].pt[2].y; vertex[2].z = oldObject->poly[i].pt[2].z;
        AddFace(vertex, 3, fast);
    }
    
    free(oldObject->poly);
    free(oldObject);
    
    CalculateNormals();
    Scale(radius, radius, radius);
}

// write the object out as a POVRay string
void FacetedSphere::WritePOVRay(std::ostringstream &theString)
{
    theString << "object {\n";
    theString << "  sphere {\n";
    theString << "    <" << m_DisplayPosition[0] << "," << m_DisplayPosition[1] << "," << m_DisplayPosition[2] << ">, " << m_Radius << "\n";
    
    // now colour
    theString << "    pigment {\n";
    theString << "      color rgbf<" << m_Colour.r << "," << m_Colour.g << "," << m_Colour.b <<"," << 1 - m_Colour.alpha << ">\n";
    theString << "    }\n";
    
    theString << "  }\n";
    theString << "}\n\n";
}

/* Normalize a point p */
point *normalize(point *p)
{
    static point r;
    double mag;
    
    r = *p;
    mag = r.x * r.x + r.y * r.y + r.z * r.z;
    if (mag != 0.0) 
    {
        mag = 1.0 / sqrt(mag);
        r.x *= mag;
        r.y *= mag;
        r.z *= mag;
    }
    
    return &r;
}

/* Return the average of two points */
point *midpoint(point *a, point *b)
{
    static point r;
    
    r.x = (a->x + b->x) * 0.5;
    r.y = (a->y + b->y) * 0.5;
    r.z = (a->z + b->z) * 0.5;
    
    return &r;
}

/* Write out all polygons in an object */
void print_object(object *obj, int level)
{
    int i;
    
#ifdef UNC  /* Generate PPHIGS archive format */
    int dx, dy, dz;
    
    printf("structure sphere%d posted {\n", level);
    printf("\tcolor polygon {\n");
    printf("\t\t200 100  50   0     50 100 200   0\n");
    printf("\t};\n");
    
    switch (level) 
    {
        case 1:
            dx = -2000; dy =  2000; dz = 0;
            break;
        case 2:
            dx =  2000; dy =  2000; dz = 0;
            break;
        case 3:
            dx = -2000; dy = -2000; dz = 0;
            break;
        case 4:
            dx =  2000; dy = -2000; dz = 0;
            break;
        case 5:
            dx =     0; dy =     0; dz = 0;
            break;
        default:
            dx = dy = dz = 0;
            break;
    }
    
    printf("\tmatrix Pre scale 1000 1000 1000;\n");
    printf("\tmatrix Pre translate %d %d %d ;\n", dx, dy, dz);
#endif  /* UNC */
    
    /* Spit out coordinates for each triangle */
    for (i = 0; i < obj->npoly; i++)
        print_triangle(&obj->poly[i]);
    
#ifdef UNC
    printf("};\n");
#endif  /* UNC */
}

/* Write a single polygon */
void print_triangle(triangle *t)
{
    int i;
    static int vertexCount = 1;
    
#ifdef UNC
    printf("\tpolygon 3 {\n");
    for (i = 0; i < 3; i++)
        printf("\t\t%g\t%g\t%g %g\t%g\t%g ;\n",
               t->pt[i].x, t->pt[i].y, t->pt[i].z,     /* Point */
               t->pt[i].x, t->pt[i].y, t->pt[i].z);    /* Normal */
    printf("\t};\n");
#else
    /* Insert code for your output format here.
    * Triangle vertices are in t->pt[0..2].{x,y,z}
    */
    for (i = 0; i < 3; i++)
    {
       printf("v %.17g %.17g %.17g\n", t->pt[i].x, t->pt[i].y, t->pt[i].z);
    }
    printf("f %d %d %d\n", vertexCount++, vertexCount++, vertexCount++);     
               
#endif
}

double sqr(double x)
{ 
    return x * x ; 
}

/* Returns area of triangle (currently unused) */
double area_of_triangle(triangle *t)
{
    double a;
    
    a  = sqr(t->pt[0].y * (t->pt[1].z - t->pt[2].z) -
             t->pt[1].y * (t->pt[0].z - t->pt[2].z) +
             t->pt[2].y * (t->pt[0].z - t->pt[1].z));
    a += sqr(t->pt[0].z * (t->pt[1].x - t->pt[2].x) -
             t->pt[1].z * (t->pt[0].x - t->pt[2].x) +
             t->pt[2].z * (t->pt[0].x - t->pt[1].x));
    a += sqr(t->pt[0].x * (t->pt[1].y - t->pt[2].y) -
             t->pt[1].x * (t->pt[0].y - t->pt[2].y) +
             t->pt[2].x * (t->pt[0].y - t->pt[1].y));
    
    return 0.5 * sqrt(a);
}

#endif
