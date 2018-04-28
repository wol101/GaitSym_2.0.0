#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>

#include "PGDMath.h"

class Trackball;
class RayGeom;

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

    enum MovieFormat {TIFF, PPM, POVRay, OBJ};

    void SetCameraRight();
    void SetCameraTop();
    void SetCameraFront();
    void SetCameraDistance(GLfloat v) { if (cameraDistance != v) { cameraDistance = v; updateGL(); } }
    void SetCameraFoV(GLfloat v) { if (FOV != v) { FOV = v; updateGL(); } }
    void SetCameraCOIX(GLfloat v) { if (COIx != v) { COIx = v; updateGL(); } }
    void SetCameraCOIY(GLfloat v) { if (COIy != v) { COIy = v; updateGL(); } }
    void SetCameraCOIZ(GLfloat v) { if (COIz != v) { COIz = v; updateGL(); } }
    void SetOverlay(bool v) { overlayFlag = v; }
    void SetWhiteBackground(bool v);
    void SetMovieFormat(MovieFormat v) { movieFormat = v; }
    void SetYUp(bool v) { yUp = v; }
    void Set3DCursorRadius(double v) { if (v >= 0) { cursorRadius = v; } }
    void Set3DCursorNudge(double v) { if (v >= 0) { m3DCursorNudge = v; } }
    int WriteFrame(QString filename);

public slots:
    void SetCameraVec(double x, double y, double z);

signals:
     void EmitStatusString(QString s);
     void EmitCOI(double x, double y, double z);
     void EmitFoV(double v);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void enterEvent(QEvent * event);
    void leaveEvent(QEvent * event);
    void wheelEvent(QWheelEvent * event);

    void WritePPM(const char *pathname, int width, int height, unsigned char *rgb);
    void WriteTIFF(const char *pathname, int width, int height, unsigned char *rgb);
    void Move3DCursor(double x, double y, double z);


private:

    GLfloat XYAspect;
    GLfloat backRed, backGreen, backBlue, backAlpha;
    GLfloat cameraDistance;
    GLfloat frontBackDistance;
    GLfloat FOV;
    GLfloat cameraVecX, cameraVecY, cameraVecZ;
    GLfloat COIx, COIy, COIz;
    GLfloat upX, upY, upZ;
    GLfloat frontClip;
    GLfloat backClip;

    bool overlayFlag;
    MovieFormat movieFormat;
    bool yUp;

    Trackball *trackball;
    bool mTrackball;
    pgd::Vector mTrackballStartCameraVec;
    pgd::Vector mTrackballStartUp;

    bool mPan;
    pgd::Vector mPanStartVec;
    pgd::Vector mPanStartCOI;
    GLdouble mPanModelMatrix[16];
    GLdouble mPanProjMatrix[16];

    bool mZoom;
    double mZoomDistance;
    double mZoomStartFOV;

    int mMouseX;
    int mMouseY;

    pgd::Vector m3DCursor;
    double cursorRadius;
    double m3DCursorNudge;
    //RayGeom *mRay;
};

#endif // GLWIDGET_H
