#include <QtGui>
#include <QtOpenGL>
#include <iostream>
#include <string>
#include <QClipboard>
#include <QWheelEvent>

#include <math.h>
#include <float.h>

#include "glwidget.h"
#include "trackball.h"
#include "Simulation.h"
#include "PGDMath.h"
#include "TIFFWrite.h"
#include "RayGeom.h"
#include "Contact.h"
#include "FacetedSphere.h"

// Simulation global
extern Simulation *gSimulation;

// output globals
extern bool gDestinationOpenGL;
extern bool gDestinationPOVRay;
extern std::ofstream *gPOVRayFile;
extern bool gDestinationOBJFile;
extern std::ofstream *gOBJFile;
extern int gVertexOffset;

// lights

static GLfloat gLight0Ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
static GLfloat gLight0Diffuse[] =  {.6f, .6f, 1.0f, 1.0f};
static GLfloat gLight0Specular[] =  {.6f, .6f, 1.0f, 1.0f};
static GLfloat gLight0Position[] = {.5f, .5f, 1.0f, 0.0f};

static GLfloat gLight1Ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
static GLfloat gLight1Diffuse[] =  {.9f, .6f, 0.0f, 1.0f};
static GLfloat gLight1Specular[] =  {.9f, .6f, 0.0f, 1.0f};
static GLfloat gLight1Position[] = {-1.0f, -1.0f, 1.0f, 0.0f};

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    XYAspect = XYAspect = static_cast<GLfloat>(width()) / static_cast<GLfloat>(height());
    cameraDistance = 50;
    frontBackDistance = 20;
    FOV = 5;
    COIx = COIy = COIz = 0;
    cameraVecX = 0;
    cameraVecY = 1;
    cameraVecZ = 0;
    upX = 0;
    upY = 0;
    upZ = 1;
    overlayFlag = false;
    movieFormat = GLWidget::TIFF;
    yUp = false;
    frontClip = 0;
    backClip = 1;
    cursorRadius = 0.001;
    m3DCursorNudge = 0.001;

    SetWhiteBackground(false);

    trackball = new Trackball();
    mTrackball = false;
    mPan = false;
    mZoom = false;

    setCursor(Qt::CrossCursor);
}

GLWidget::~GLWidget()
{
    makeCurrent();
    delete trackball;
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(400, 400);
}

void GLWidget::initializeGL()
{
}

void GLWidget::paintGL()
{
    //Set up OpenGL lights
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);

    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, gLight0Ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, gLight0Diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, gLight0Specular);

    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, gLight1Ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, gLight1Diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, gLight1Specular);

    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_FLAT);
    glFrontFace(GL_CCW); // counter clockwise is front facing
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK); // note culling has nothing to do with normals, it is done on winding order
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);

    // antialiasing is very slow
    //glEnable(GL_POINT_SMOOTH);
    //glEnable(GL_LINE_SMOOTH);
    //glEnable(GL_POLYGON_SMOOTH);

    glClearColor( backRed, backGreen, backBlue, backAlpha );
    if (overlayFlag == false) glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // set up GL_PROJECTION matrix to describe the attributes of the camera,
    // such as field of view, focal length, fish eye lens
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    frontClip = fabs(cameraDistance) - frontBackDistance / 2;
    backClip = fabs(cameraDistance) + frontBackDistance / 2;
    if (frontClip / backClip < 0.01) frontClip = backClip * 0.01;
    gluPerspective(FOV, XYAspect, frontClip, backClip);

    // set up GL_MODELVIEW matrix as where you stand with the camera and the direction you point it.
    glMatrixMode( GL_MODELVIEW );

    glLoadIdentity();
    gluLookAt(COIx - cameraVecX * cameraDistance, COIy - cameraVecY * cameraDistance, COIz - cameraVecZ * cameraDistance,
               COIx, COIy, COIz,
               upX, upY, upZ);

    // draw the lights
    glLightfv(GL_LIGHT0, GL_POSITION, gLight0Position);
    glLightfv(GL_LIGHT1, GL_POSITION, gLight1Position);

    // now draw things
    if (gSimulation) gSimulation->Draw();

    // the 3d cursor
    FacetedSphere sphere(cursorRadius, 3);
    sphere.SetDisplayPosition(m3DCursor.x, m3DCursor.y, m3DCursor.z);
    sphere.Draw();

    // any manipulation feedback
    if (mZoom)
    {
        // line drawing lighting mode
        glDisable(GL_LIGHTING);
        glColor4f(1, 1, 1, 1);

        // raster mode positioning
        // with origin at top left
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width(), height(), 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        double centreX = double(width()) / 2;
        double centreY = double(height()) / 2;
        double xMouse = double(mMouseX) - centreX;
        double yMouse = double(mMouseY) - centreY;
        double radius = sqrt(xMouse * xMouse + yMouse * yMouse);

        glBegin(GL_LINE_LOOP);
        GLfloat x, y;
        for (int angle = 0; angle < 360; angle += 2)
        {
                x = centreX + radius * cos ((double)angle * M_PI / 180);
                y = centreY + radius * sin ((double)angle * M_PI / 180);
                glVertex2f(x, y);
        }
        glEnd();

    }

    if (mTrackball  && trackball->GetOutsideRadius())
    {
        // line drawing lighting mode
        glDisable(GL_LIGHTING);
        glColor4f(1, 1, 1, 1);

        // raster mode positioning
        // with origin at top left
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width(), height(), 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        double centreX = double(width()) / 2;
        double centreY = double(height()) / 2;
        double xMouse = double(mMouseX) - centreX;
        double yMouse = double(mMouseY) - centreY;
        double radius = trackball->GetTrackballRadius();

        glBegin(GL_LINE_LOOP);
        GLfloat x, y;
        for (int angle = 0; angle < 360; angle += 2)
        {
            x = centreX + radius * cos ((double)angle * M_PI / 180);
            y = centreY + radius * sin ((double)angle * M_PI / 180);
            glVertex2f(x, y);
        }
        glEnd();
    }
}

void GLWidget::resizeGL(int width, int height)
{
    glViewport( 0, 0, width, height );
    XYAspect = static_cast<GLfloat>(width) / static_cast<GLfloat>(height);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    if (gSimulation == 0) return;

    mMouseX = event->pos().x();
    mMouseY = event->pos().y();

    mTrackball = false;
    if (event->buttons() & Qt::LeftButton)
    {
        if (event->modifiers() == Qt::NoModifier)
        {
            int trackballRadius;
            if (width() < height()) trackballRadius = width() / 2.2;
            else trackballRadius = height() / 2.2;
            mTrackballStartCameraVec = pgd::Vector(cameraVecX, cameraVecY, cameraVecZ);
            mTrackballStartUp = pgd::Vector(upX, upY, upZ);
            trackball->StartTrackball(event->pos().x(), event->pos().y(), width() / 2, height() / 2, trackballRadius, mTrackballStartUp, -mTrackballStartCameraVec);
            mTrackball = true;
            emit EmitStatusString(tr("Rotate"));
            updateGL();
        }
        else if (event->modifiers() & Qt::ShiftModifier)
        {
            // create the collision ray
            GLdouble objX, objY, objZ;
            GLdouble modelMatrix[16];
            glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
            GLdouble projMatrix[16];
            glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
            int viewport[4];
            glGetIntegerv(GL_VIEWPORT,viewport);

            GLfloat winX, winY, winZ;
            winX = event->pos().x();
            winY = event->pos().y();
            winY = (GLfloat)viewport[3] - winY;

            pgd::Vector cameraPosition = pgd::Vector(COIx, COIy, COIz) - cameraDistance * pgd::Vector(cameraVecX, cameraVecY, cameraVecZ);
            dReal length = fabs(cameraDistance) + frontBackDistance / 2;

#ifdef USE_EYE_BASED_PICKING // picking based on the coordinate of the surface under the mouse and the eye position
            glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
            gluUnProject(winX, winY, winZ,
                    modelMatrix, projMatrix, viewport,
                    &objX, &objY, &objZ );

            pgd::Vector rayDirection = pgd::Vector(objX, objY, objZ) - cameraPosition;
            rayDirection.Normalize();

            RayGeom rayGeom(0, length,
                            cameraPosition.x, cameraPosition.y, cameraPosition.z,
                            rayDirection.x, rayDirection.y, rayDirection.z);

#else // picking based on the near and far clipping planes
            winZ = frontClip;
            gluUnProject(winX, winY, winZ,
                    modelMatrix, projMatrix, viewport,
                    &objX, &objY, &objZ );
            pgd::Vector nearPoint(objX, objY, objZ);
            winZ = backClip;
            gluUnProject(winX, winY, winZ,
                    modelMatrix, projMatrix, viewport,
                    &objX, &objY, &objZ );
            pgd::Vector farPoint(objX, objY, objZ);

            pgd::Vector rayDirection = farPoint - nearPoint;
            rayDirection.Normalize();

            RayGeom rayGeom(0, length,
                            nearPoint.x, nearPoint.y, nearPoint.z,
                            rayDirection.x, rayDirection.y, rayDirection.z);
#endif
            rayGeom.SetParams(0, 0, 0); // firstcontact=0, backfacecull=0, closestHit = 0

            // code for collision detection
            pgd::Vector closestContact;
            std::vector<Geom *> *pickGeomList = gSimulation->GetPickGeomList();
            const int maxContacts = 128;
            dContactGeom contacts[maxContacts];
            int numCollisions;
            dReal distance2;
            pgd::Vector cameraRelVector;
            dReal minDistance2 = DBL_MAX;

            for (unsigned int j = 0; j < pickGeomList->size(); j++)
            {
                // std::cerr << *(*pickGeomList)[j]->GetName() << "\n";
                numCollisions = dCollide (rayGeom.GetGeomID(), (*pickGeomList)[j]->GetGeomID(), maxContacts, contacts, sizeof(dContactGeom));
                for (int i = 0; i < numCollisions; i++)
                {
                    cameraRelVector = pgd::Vector(contacts[i].pos[0], contacts[i].pos[1], contacts[i].pos[2]) - cameraPosition;
                    distance2 = cameraRelVector.Magnitude2();
                    if (distance2 < minDistance2)
                    {
                        minDistance2 = distance2;
                        closestContact = pgd::Vector(contacts[i].pos[0], contacts[i].pos[1], contacts[i].pos[2]);
                    }
                }
             }

            if (minDistance2 < DBL_MAX)
            {
                Move3DCursor(closestContact.x, closestContact.y, closestContact.z);
            }

        }
    }
    else if (event->buttons() & Qt::RightButton)
    {
        mPan = true;

        // get pick in world coordinates
        GLdouble objX, objY, objZ;
        // get and store the matrices from the start of panning
        glGetDoublev(GL_MODELVIEW_MATRIX,mPanModelMatrix);
        glGetDoublev(GL_PROJECTION_MATRIX,mPanProjMatrix);
        int viewport[4];
        glGetIntegerv(GL_VIEWPORT,viewport);

        GLfloat winX, winY, winZ;
        winX = event->pos().x();
        winY = event->pos().y();
        winY = (GLfloat)viewport[3] - winY;
        winZ = (backClip - frontClip) / 2;
        gluUnProject(winX, winY, winZ,
                     mPanModelMatrix, mPanProjMatrix, viewport,
                     &objX, &objY, &objZ );

        mPanStartVec = pgd::Vector(objX, objY, objZ);
        mPanStartCOI = pgd::Vector(COIx, COIy, COIz);

        emit EmitStatusString(tr("Pan"));
        updateGL();
    }
    else if (event->buttons() & Qt::MidButton)
    {
        mZoom = true;
        // centred -1 to -1 normalised values
        double x = (double)(2 * event->pos().x()) / (double)width() - 1.0;
        double y = (double)(2 * (height() - event->pos().y())) / (double)height() - 1.0;
        mZoomDistance = sqrt(x * x + y * y);
        if (mZoomDistance < 0.05) mZoomDistance = 0.05;
        mZoomStartFOV = FOV;

        emit EmitStatusString(tr("Zoom"));
        updateGL();
    }

}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    mMouseX = event->pos().x();
    mMouseY = event->pos().y();

    if (event->buttons() & Qt::LeftButton)
    {
        if (mTrackball)
        {
            pgd::Quaternion rotation;
            trackball->RollTrackballToClick(event->pos().x(), event->pos().y(), &rotation);
            pgd::Vector newCameraVec = pgd::QVRotate(~rotation, mTrackballStartCameraVec);
            cameraVecX = newCameraVec.x;
            cameraVecY = newCameraVec.y;
            cameraVecZ = newCameraVec.z;
            pgd::Vector newUp = pgd::QVRotate(~rotation, mTrackballStartUp);
            upX = newUp.x;
            upY = newUp.y;
            upZ = newUp.z;
            updateGL();

            emit EmitStatusString(QString("Camera %1 %2 %3 Up %4 %5 %6").arg(cameraVecX).arg(cameraVecY).arg(cameraVecZ).arg(upX).arg(upY).arg(upZ));
        }
    }
    else if (event->buttons() & Qt::RightButton)
    {
        if (mPan)
        {
            // get pick in world coordinates
            GLdouble objX, objY, objZ;
            int viewport[4];
            glGetIntegerv(GL_VIEWPORT,viewport);

            GLfloat winX, winY, winZ;
            winX = event->pos().x();
            winY = event->pos().y();
            winY = (GLfloat)viewport[3] - winY;
            winZ = (backClip - frontClip) / 2;
            gluUnProject(winX, winY, winZ,
                         mPanModelMatrix, mPanProjMatrix, viewport,
                         &objX, &objY, &objZ );
            // and move centre of interest by move
            //COIx -= (objX - mPanStartVec.x);
            //COIy -= (objY - mPanStartVec.y);
            //COIz -= (objZ - mPanStartVec.z);
            // and reset last position
            //mPanStartVec = pgd::Vector(objX, objY, objZ);
            COIx = mPanStartCOI.x + (objX - mPanStartVec.x);
            COIy = mPanStartCOI.y + (objY - mPanStartVec.y);
            COIz = mPanStartCOI.z + (objZ - mPanStartVec.z);
            updateGL();

            emit EmitStatusString(QString("COI %1 %2 %3").arg(COIx).arg(COIy).arg(COIz));
            emit EmitCOI(COIx, COIy, COIz);
        }
    }
    else if (event->buttons() & Qt::MidButton)
    {
        if (mZoom)
        {
            // centred -1 to -1 normalised values
            double x = (double)(2 * event->pos().x()) / (double)width() - 1.0;
            double y = (double)(2 * (height() - event->pos().y())) / (double)height() - 1.0;
            double zoomDistance = sqrt(x * x + y * y);
            FOV = mZoomStartFOV * mZoomDistance / zoomDistance;
            if (FOV > 170) FOV = 170;
            else if (FOV < 0.001) FOV = 0.001;
            updateGL();

            emit EmitStatusString(QString("FOV %1").arg(FOV));
            emit EmitFoV(FOV);
        }
    }
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    mTrackball = false;
    mPan = false;
    mZoom = false;
    updateGL();
}

void GLWidget::wheelEvent(QWheelEvent * event)
{
    // assume each ratchet of the wheel gives a score of 120 (8 * 15 degrees)
    double sensitivity = 2400;
    double scale = 1.0 + double(event->delta()) / sensitivity;
    FOV *= scale;
    if (FOV > 170) FOV = 170;
    else if (FOV < 0.001) FOV = 0.001;
    updateGL();
}

void GLWidget::SetCameraRight()
{
    if (yUp)
    {
        cameraVecX = 0;
        cameraVecY = 0;
        cameraVecZ = -1;
        upX = 0;
        upY = 1;
        upZ = 0;
    }
    else
    {
        cameraVecX = 0;
        cameraVecY = 1;
        cameraVecZ = 0;
        upX = 0;
        upY = 0;
        upZ = 1;
    }
    updateGL();
}

void GLWidget::SetCameraTop()
{
    if (yUp)
    {
        cameraVecX = 0;
        cameraVecY = 1;
        cameraVecZ = 0;
        upX = 0;
        upY = 0;
        upZ = 1;
    }
    else
    {
        cameraVecX = 0;
        cameraVecY = 0;
        cameraVecZ = -1;
        upX = 0;
        upY = 1;
        upZ = 0;
    }
    updateGL();
}

void GLWidget::SetCameraFront()
{
    if (yUp)
    {
        cameraVecX = -1;
        cameraVecY = 0;
        cameraVecZ = 0;
        upX = 0;
        upY = 1;
        upZ = 0;
    }
    else
    {
        cameraVecX = -1;
        cameraVecY = 0;
        cameraVecZ = 0;
        upX = 0;
        upY = 0;
        upZ = 1;
    }
    updateGL();
}

void GLWidget::SetWhiteBackground(bool v)
{
    if (v)
    {
        backRed = backGreen = backBlue = backAlpha = 1.0;
    }
    else
    {
        backRed = backGreen = backBlue = 0.0;
        backAlpha = 1.0;
    }
    updateGL();
}

void GLWidget::SetCameraVec(double x, double y, double z)
{
    if (yUp)
    {
        cameraVecX = x;
        cameraVecY = z;
        cameraVecZ = -y;
        if (z > 0.999 || z < -0.999)
        {
            upX = 0;
            upY = 0;
            upZ = 1;
        }
        else
        {
            upX = 0;
            upY = 1;
            upZ = 0;
        }
   }
    else
    {
        cameraVecX = x;
        cameraVecY = y;
        cameraVecZ = z;
        if (z > 0.999 || z < -0.999)
        {
            upX = 0;
            upY = 1;
            upZ = 0;
        }
        else
        {
            upX = 0;
            upY = 0;
            upZ = 1;
        }
    }
    updateGL();
}

// write the current frame out to a file
int GLWidget::WriteFrame(QString filename)
{
    std::string pathname(filename.toAscii());
    if (movieFormat == PPM)
    {
        unsigned char *rgb = new unsigned char[width() * height() * 3];
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width(), height(), GL_RGB, GL_UNSIGNED_BYTE, rgb);
        std::string ppmpathname = pathname + ".ppm";
        WritePPM(ppmpathname.c_str(), width(), height(), (unsigned char *)rgb);
        delete [] rgb;
    }
    if (movieFormat == TIFF)
    {
        unsigned char *rgb = new unsigned char[width() * height() * 3];
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width(), height(), GL_RGB, GL_UNSIGNED_BYTE, rgb);
        std::string tiffpathname = pathname + ".tif";
        WriteTIFF(tiffpathname.c_str(), width(), height(), (unsigned char *)rgb);
        delete [] rgb;
    }
    if (movieFormat == POVRay)
    {
        gDestinationOpenGL = false;
        gDestinationPOVRay = true;
        gDestinationOBJFile = false;
        std::string povpathname = pathname + ".pov";
        gPOVRayFile = new std::ofstream(povpathname.c_str());
        (*gPOVRayFile) << "#declare gCameraX = " << -cameraVecX * cameraDistance << " ;\n";
        (*gPOVRayFile) << "#declare gCameraY = " << -cameraVecY * cameraDistance << " ;\n";
        (*gPOVRayFile) << "#declare gCameraZ = " << -cameraVecZ * cameraDistance << " ;\n";
        (*gPOVRayFile) << "#declare gCOIx = " << COIx << " ;\n";
        (*gPOVRayFile) << "#declare gCOIy = " << COIy << " ;\n";
        (*gPOVRayFile) << "#declare gCOIz = " << COIz << " ;\n";
        (*gPOVRayFile) << "#declare gUpX = " << upX << " ;\n";
        (*gPOVRayFile) << "#declare gUpY = " << upY << " ;\n";
        (*gPOVRayFile) << "#declare gUpZ = " << upZ << " ;\n";
        (*gPOVRayFile) << "#declare gTime = " << gSimulation->GetTime() << " ;\n";

        (*gPOVRayFile) << "\n#include \"camera.pov\"\n\n";
        gSimulation->Draw();
        delete gPOVRayFile;
        gPOVRayFile = 0;
        gDestinationOpenGL = true;
        gDestinationPOVRay = false;
        gDestinationOBJFile = false;
    }
    if (movieFormat == OBJ)
    {
        gVertexOffset = 0;
        gDestinationOpenGL = false;
        gDestinationPOVRay = false;
        gDestinationOBJFile = true;
        std::string objpathname = pathname + ".obj";
        gOBJFile = new std::ofstream(objpathname.c_str());
        gSimulation->Draw();
        delete gOBJFile;
        gOBJFile = 0;
        gDestinationOpenGL = true;
        gDestinationPOVRay = false;
        gDestinationOBJFile = false;
    }
    return 0;
}

// write a PPM file (need to invert the y axis)
void GLWidget::WritePPM(const char *pathname, int width, int height, unsigned char *rgb)
{
    FILE *out;
    int i;

    out = fopen(pathname, "wb");

    // need to invert write order
    fprintf(out, "P6\n%d %d\n255\n", width, height);
    for (i = height - 1; i >= 0; i--)
        fwrite(rgb + (i * width * 3), width * 3, 1, out);

    fclose(out);
}

// write a TIFF file
void GLWidget::WriteTIFF(const char *pathname, int width, int height, unsigned char *rgb)
{
    TIFFWrite tiff;
    int i;

    tiff.initialiseImage(width, height, 72, 72, 3);
    // need to invert write order
    for (i = 0; i < height; i ++)
        tiff.copyRow(height - i - 1, rgb + (i * width * 3));

    tiff.writeToFile((char *)pathname);
}

// handle key presses
void GLWidget::keyPressEvent( QKeyEvent *e )
{
    switch( e->key() )
    {

        // X, Y and Z move the cursor
    case Qt::Key_X:
        if (e->modifiers() == Qt::NoModifier)
            m3DCursor.x += m3DCursorNudge;
        else
            m3DCursor.x -= m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        break;

    case Qt::Key_Y:
        if (e->modifiers() == Qt::NoModifier)
            m3DCursor.y += m3DCursorNudge;
        else
            m3DCursor.y -= m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        break;

    case Qt::Key_Z:
        if (e->modifiers() == Qt::NoModifier)
            m3DCursor.z += m3DCursorNudge;
        else
            m3DCursor.z -= m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        break;

        // S snaps the cursor to the nearest whole number multiple of the nudge value
    case Qt::Key_S:
        m3DCursor.x = round(m3DCursor.x / m3DCursorNudge) * m3DCursorNudge;
        m3DCursor.y = round(m3DCursor.y / m3DCursorNudge) * m3DCursorNudge;
        m3DCursor.z = round(m3DCursor.z / m3DCursorNudge) * m3DCursorNudge;
        Move3DCursor(m3DCursor.x, m3DCursor.y, m3DCursor.z);
        break;

    default:
        QGLWidget::keyPressEvent( e );
    }
}

// use enter events to grab the keyboard
void GLWidget::enterEvent ( QEvent * event )
{
    grabKeyboard();
}

// use leave events to release the keyboard
void GLWidget::leaveEvent ( QEvent * event )
{
    releaseKeyboard();
}

// set the 3D cursor position
void GLWidget::Move3DCursor(double x, double y, double z)
{
    m3DCursor = pgd::Vector(x, y, z);
    QClipboard *clipboard = QApplication::clipboard();
    clipboard->setText(QString("%1\t%2\t%3").arg(m3DCursor.x).arg(m3DCursor.y).arg(m3DCursor.z), QClipboard::Clipboard);
    emit EmitStatusString(QString("3D Cursor %1\t%2\t%3").arg(m3DCursor.x).arg(m3DCursor.y).arg(m3DCursor.z));
    updateGL();
}
