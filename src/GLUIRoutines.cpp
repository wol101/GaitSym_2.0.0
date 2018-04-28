/*
 *  GLUIRoutines.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// GLUIRoutines.cc - routine for displaying the GLUT and GLUI windows

#if !defined(USE_CARBON) && !defined(USE_QT)
#ifdef USE_OPENGL

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>

#include <gl.h>
#include <glu.h>
#include <glut.h>
#include <glui/glui.h>

#include <ode/ode.h>

#include "GLUIRoutines.h"
#include "Simulation.h"
#include "DebugControl.h"
#include "PGDMath.h"
#include "TIFFWrite.h"
#include "Body.h"
#include "GLUtils.h"
#include "Util.h"

#include <glui/algebra3.h>

#ifdef USE_OSMESA
#include <osmesa.h>
static int OSMesaFrameGrab(int WIDTH, int HEIGHT, char *filename);
void OSMesaWritePPM(const char *filename, const GLubyte *buffer, int width, int height, int binary);
#endif

// external functions
int ReadModel();
void WriteModel();
void OutputProgramState(std::ostream &out);
void OutputKinetics();

// various globals

// Simulation global
extern Simulation *gSimulation;

extern int gWindowWidth;
extern int gWindowHeight;
extern int gFrameGrabWindowWidth;
extern int gFrameGrabWindowHeight;
extern bool gFinishedFlag;
extern bool gUseSeparateWindow;
extern int gDisplaySkip;
extern float gFOV;
extern float gCameraDistance;
extern long gSleepTime;

// output globals
extern bool gDestinationOpenGL;
extern bool gDestinationPOVRay;
extern std::ofstream *gPOVRayFile;
extern bool gDestinationOBJFile;
extern std::ofstream *gOBJFile;
extern int gVertexOffset;

// graphic control globals
extern int gAxisFlag;
extern int gDrawMuscleForces;
extern int gDrawContactForces;
extern int gDrawMuscles;
extern int gDrawBonesFlag;
extern int gDrawGeomsFlag;
extern int gWhiteBackground;
extern int gBadMesh;
extern int g_ActivationDisplay;


// movie
static GLUI_String gMovieDirectory = "movie";
static int gWriteMovie = false;
static int gUsePPM = false;
static int gUseTIFF = false;
static int gUsePOV = true;
static int gUseOBJ = false;

static int gOverlayFlag = false;

static int gTrackingFlag = false;
static float gCOIx = 0;
static float gCOIy = 0;
static float gCOIz = 0;
static GLUI_Rotation *gViewRotationControl;
static int gBallDamping = true;

static float gCameraX;
static float gCameraY;
static float gCameraZ;

static int gWindowPositionX = 10;
static int gWindowPositionY = 20;

static int gMainWindow; // stores the id of the main display window
static int gStepFlag = false;
static int gSlowFlag = false;
static int gRunFlag = false;

static GLfloat gLight0Ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
static GLfloat gLight0Diffuse[] =  {.6f, .6f, 1.0f, 1.0f};
static GLfloat gLight0Specular[] =  {.6f, .6f, 1.0f, 1.0f};
static GLfloat gLight0Position[] = {.5f, .5f, 1.0f, 0.0f};

static GLfloat gLight1Ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
static GLfloat gLight1Diffuse[] =  {.9f, .6f, 0.0f, 1.0f};
static GLfloat gLight1Specular[] =  {.9f, .6f, 0.0f, 1.0f};
static GLfloat gLight1Position[] = {-1.0f, -1.0f, 1.0f, 0.0f};

static GLfloat gLightsRotation[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };

static float gXYAspect;
static GLUI *gGLUIControlWindow;
static float gViewRotation[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
static float gScale = 1.0;

// limits
static GLUI_Spinner *gTimeLimitSpinner;
static GLUI_Spinner *gMechanicalEnergyLimitSpinner;
static GLUI_Spinner *gMetabolicEnergyLimitSpinner;

// info panel
static GLUI_EditText *gGLUIEditTextCameraX;
static GLUI_EditText *gGLUIEditTextCameraY;
static GLUI_EditText *gGLUIEditTextCameraZ;
static GLUI_EditText *gGLUIEditTextSimulationTime;
static GLUI_EditText *gGLUIEditTextMechanicalEnergy;
static GLUI_EditText *gGLUIEditTextMetabolicEnergy;
static GLUI_EditText *gGLUIEditTextFitness;

// debug panel
static int gDebugListItem;
static int gDebugWrite = false;

// gloabl widow shape storage
static int gGlutReshapeX;
static int gGlutReshapeY;

static void myGlutKeyboard(unsigned char Key, int x, int y);
static void myGlutIdle( void );
static void myGlutMouse(int button, int button_state, int x, int y );
static void myGlutMotion(int x, int y );
static void myGlutReshape( int x, int y );
static void myGlutDisplay( void );
static void WritePPM(const char *pathname, int gWidth, int gHeight, unsigned char *rgb);
static void WriteTIFF(const char *pathname, int gWidth, int gHeight, unsigned char *rgb);   
static void ButtonCallback( int control );
static void WriteOutput(const char *subpath);

// UI ID defines
const int STEP_BUTTON = 1;
const int STOP_BUTTON = 2;
const int START_BUTTON = 3;
const int BALL_RIGHT_BUTTON = 5;
const int BALL_FRONT_BUTTON = 6;
const int BALL_TOP_BUTTON = 7;
const int SNAPSHOT_BUTTON = 8;
const int MOVIE_CHECKBOX = 11;
const int BALL_DAMP_CHECKBOX = 12;
const int DEBUG_CHECKBOX = 21;

const int DEFAULT_TEXT_WIDTH = 40;  


/**************************************** myGlutKeyboard() **********/

void myGlutKeyboard(unsigned char Key, int x, int y)
{
    switch(Key)
    {
        case 27:
        case 'q':
            exit(0);
            break;
    }
    
    glutPostRedisplay();
}


/***************************************** myGlutIdle() ***********/

void myGlutIdle( void )
{
    /* According to the GLUT specification, the current window is
    undefined during an idle callback.  So we need to explicitly change
    it if necessary */
    if ( glutGetWindow() != gMainWindow )
        glutSetWindow(gMainWindow);
    
    int i;
    static double cameraCOIX = 0;
    bool newPicture = false;
    DebugControl debugStore;
    
    // before we do anything else, check that we have a model
    
    if (gFinishedFlag)
    {
        if (ReadModel() == 0)
        {
            gFinishedFlag = false;
#if defined(USE_SOCKETS)  || defined(USE_UDP)
            gRunFlag = true;
#else
            gTimeLimitSpinner->set_float_val((float)gSimulation->GetTimeLimit());
            gMechanicalEnergyLimitSpinner->set_float_val((float)gSimulation->GetMechanicalEnergyLimit());
            gMetabolicEnergyLimitSpinner->set_float_val((float)gSimulation->GetMetabolicEnergyLimit());
#endif
        }
        else
        {
#if defined(USE_SOCKETS)  || defined(USE_UDP)
            usleep(gSleepTime); // slight pause on read failure
            glutPostRedisplay();
            return;
#else
            exit(1);
#endif
        }
    }
    
    if (gRunFlag)
    {
        for (i = 0; i < gDisplaySkip; i++)
        {
            if (gSimulation->ShouldQuit() == true)
            {
                gFinishedFlag = true;
                gRunFlag = false;
                break;
            }
            if (i == 0)
            {
                gSimulation->UpdateSimulation();
            }
            else
            {
                debugStore = gDebug;
                gDebug = NoDebug;
                gSimulation->UpdateSimulation();
                gDebug = debugStore;
            }
            
            newPicture = true;
            
            if (gSimulation->TestForCatastrophy())
            {
                gFinishedFlag = true;
                gRunFlag = false;
                break;
            }
        }
        
        if (gSlowFlag) usleep(100000); // 0.1 s
    }
    else
    {
        if (gStepFlag)
        {
            gStepFlag = false;
            for (i = 0; i < gDisplaySkip; i++)
            {
                if (i == 0)
                {
                   gSimulation->UpdateSimulation();
                }
                else
                {
                    debugStore = gDebug;
                    gDebug = NoDebug;
                    gSimulation->UpdateSimulation();
                    gDebug = debugStore;
                }
            }
            newPicture = true;
        }
        
    }
    
    // update the live variables
    if (gTrackingFlag)
    {
        Body *body = gSimulation->GetBody(gSimulation->GetInterface()->TrackBodyID.c_str());
        if (body)
        {
            const dReal *position = dBodyGetPosition(body->GetBodyID());
            if (position[0] != cameraCOIX)
            {
                cameraCOIX = position[0];
                gCOIx = cameraCOIX;
            }
        }
    }
    
    // set the info values by hand
    gGLUIEditTextCameraX->set_float_val(gCameraX);
    gGLUIEditTextCameraY->set_float_val(gCameraY);
    gGLUIEditTextCameraZ->set_float_val(gCameraZ);
    gGLUIEditTextSimulationTime->set_float_val((float)gSimulation->GetTime());
    gGLUIEditTextMechanicalEnergy->set_float_val((float)gSimulation->GetMechanicalEnergy());
    gGLUIEditTextMetabolicEnergy->set_float_val((float)gSimulation->GetMetabolicEnergy());
    gGLUIEditTextFitness->set_float_val((float)gSimulation->CalculateInstantaneousFitness());
    
    // get any new limits
#if ! defined(USE_SOCKETS) && ! defined(USE_UDP)
    gSimulation->SetTimeLimit((double)gTimeLimitSpinner->get_float_val());
    gSimulation->SetMechanicalEnergyLimit((double)gMechanicalEnergyLimitSpinner->get_float_val());
    gSimulation->SetMetabolicEnergyLimit((double)gMetabolicEnergyLimitSpinner->get_float_val());
#endif
    
    gGLUIControlWindow->sync_live();
    glutPostRedisplay();
    
    static unsigned int frameCount = 1;
    if (newPicture && gWriteMovie)
    {
        char filename[256];
        sprintf(filename, "Frame%05d", frameCount);
        std::string pathname = gMovieDirectory;
        pathname.append("/");
        pathname.append(filename);
        WriteOutput(pathname.c_str());
        frameCount++;
    }
    
    // enforce a control update
    // shouldn't be necessary but I'm not getting proper update of live controls on a Mac
    // but I get flickering on Linux and SGI - sigh!
#ifdef UPDATE_CONTROLS_ON_IDLE
    glutSetWindow(gGLUIControlWindow->get_glut_window_id());
    glutPostRedisplay();
#endif
    
    if (gFinishedFlag)
    {
        WriteModel();
    }
}


/***************************************** myGlutMouse() **********/

void myGlutMouse(int button, int button_state, int x, int y )
{
}


/***************************************** myGlutMotion() **********/

void myGlutMotion(int x, int y )
{
    glutPostRedisplay(); 
}

/**************************************** myGlutReshape() *************/

void myGlutReshape( int x, int y )
{
    gGlutReshapeX = x;
    gGlutReshapeY = y;
    
    if (gUseSeparateWindow)
    {
        glViewport( 0, 0, x, y );
        gXYAspect = (float)x / (float)y;
    }
    else
    {
        int tx, ty, tw, th;
        GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
        glViewport( tx, ty, tw, th );
        gXYAspect = (float)tw / (float)th;
    }
    
    glutPostRedisplay();
}

/**************************************** StartGLUT() *************/

void StartGlut(void)
{
    /****************************************/
    /*   Initialize GLUT and create window  */
    /****************************************/
    
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    glutInitWindowPosition( gWindowPositionX, gWindowPositionY);
    glutInitWindowSize( gWindowWidth, gWindowHeight );
    
    gMainWindow = glutCreateWindow( "ODE Simulation" );
    
    glutDisplayFunc( myGlutDisplay );
    glutMotionFunc( myGlutMotion );
    GLUI_Master.set_glutReshapeFunc( myGlutReshape );
    GLUI_Master.set_glutKeyboardFunc( myGlutKeyboard );
    GLUI_Master.set_glutSpecialFunc( NULL );
    GLUI_Master.set_glutMouseFunc( myGlutMouse );
        
    /*** Create the side subwindow ***/
    if (gUseSeparateWindow)
    {
        gGLUIControlWindow = GLUI_Master.create_glui("Controls", 0,
                                                     gWindowPositionX + gWindowWidth + 20, gWindowPositionY);
    }
    else
    {
        gGLUIControlWindow = GLUI_Master.create_glui_subwindow(gMainWindow, GLUI_SUBWINDOW_RIGHT);
    }
    
    
    /*** Add the view controls ***/
    
    // column 1
    
    // Camera Panel
    GLUI_Panel *cameraPanel = gGLUIControlWindow->add_panel("Camera");
    gViewRotationControl =
        gGLUIControlWindow->add_rotation_to_panel(cameraPanel, "Rotate", gViewRotation);
    gViewRotationControl->set_spin( 0 );
    /* GLUI_Checkbox *dampingCheckBox = */ gGLUIControlWindow->add_checkbox_to_panel(cameraPanel, "Damping", &gBallDamping,
                                                                                     BALL_DAMP_CHECKBOX, ButtonCallback);
    /* GLUI_Button *rightButton = */ gGLUIControlWindow->add_button_to_panel(cameraPanel, "Right", BALL_RIGHT_BUTTON, ButtonCallback);
    /* GLUI_Button *topButton = */ gGLUIControlWindow->add_button_to_panel(cameraPanel, "Top", BALL_TOP_BUTTON, ButtonCallback);
    /* GLUI_Button *frontButton = */ gGLUIControlWindow->add_button_to_panel(cameraPanel, "Front", BALL_FRONT_BUTTON, ButtonCallback);
    
#ifdef USE_TRANSLATION_WIDGETS
    GLUI_Translation *dist =
        gGLUIControlWindow->add_translation_to_panel(cameraPanel,
                                                     "Distance", GLUI_TRANSLATION_Z, &gCameraDistance );
    dist->set_speed( .1 );
#endif
    GLUI_Spinner *cameraDistanceSpinner = gGLUIControlWindow->add_spinner_to_panel(cameraPanel,
                                                                                   "Distance", GLUI_SPINNER_FLOAT, &gCameraDistance);
    cameraDistanceSpinner->set_w(DEFAULT_TEXT_WIDTH);
    cameraDistanceSpinner->set_alignment(GLUI_ALIGN_RIGHT);
    GLUI_Spinner *fovSpiner = gGLUIControlWindow->add_spinner_to_panel(cameraPanel,
                                                                       "FoV", GLUI_SPINNER_FLOAT, &gFOV);
    fovSpiner->set_float_limits(1.0, 170.0);
    fovSpiner->set_w(DEFAULT_TEXT_WIDTH);
    fovSpiner->set_alignment(GLUI_ALIGN_RIGHT);
    
    // Centre of Interest Panel
    GLUI_Panel *coiPanel = gGLUIControlWindow->add_panel("COI");
#ifdef USE_TRANSLATION_WIDGETS
    GLUI_Translation *trans_x =
        gGLUIControlWindow->add_translation_to_panel(coiPanel,
                                                     "X", GLUI_TRANSLATION_X, &gCOIx );
    trans_x->set_speed( .1 );
    GLUI_Translation *trans_y =
        gGLUIControlWindow->add_translation_to_panel(coiPanel,
                                                     "Y", GLUI_TRANSLATION_Z, &gCOIy );
    trans_y->set_speed( .1 );
    GLUI_Translation *trans_z =
        gGLUIControlWindow->add_translation_to_panel(coiPanel,
                                                     "Z", GLUI_TRANSLATION_Y, &gCOIz );
    trans_z->set_speed( .1 );
#endif
    GLUI_Spinner *coiXSpinner = gGLUIControlWindow->add_spinner_to_panel(coiPanel,
                                                                         "COI X", GLUI_SPINNER_FLOAT, &gCOIx);
    coiXSpinner->set_w(DEFAULT_TEXT_WIDTH);
    coiXSpinner->set_alignment(GLUI_ALIGN_RIGHT);
    GLUI_Spinner *coiYSpinner = gGLUIControlWindow->add_spinner_to_panel(coiPanel,
                                                                         "COI Y", GLUI_SPINNER_FLOAT, &gCOIy);
    coiYSpinner->set_w(DEFAULT_TEXT_WIDTH);
    coiYSpinner->set_alignment(GLUI_ALIGN_RIGHT);
    GLUI_Spinner *coiZSpinner = gGLUIControlWindow->add_spinner_to_panel(coiPanel,
                                                                         "COI Z", GLUI_SPINNER_FLOAT, &gCOIz);
    coiZSpinner->set_w(DEFAULT_TEXT_WIDTH);
    coiZSpinner->set_alignment(GLUI_ALIGN_RIGHT);
    
    
#if defined USE_TRANSLATION_WIDGETS
    // column 2
    gGLUIControlWindow->add_column( true );
#endif
    
    // Display Panel
    GLUI_Panel *displayPanel = gGLUIControlWindow->add_panel("Display");
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Tracking", &gTrackingFlag);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Overlay", &gOverlayFlag);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Draw Contact Forces", &gDrawContactForces);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Draw Muscle Forces", &gDrawMuscleForces);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Draw Muscles", &gDrawMuscles);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Draw Bones", &gDrawBonesFlag);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Draw Geoms", &gDrawGeomsFlag);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Draw Axes", &gAxisFlag);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "White Background", &gWhiteBackground);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Bad Mesh", &gBadMesh);
    gGLUIControlWindow->add_checkbox_to_panel(displayPanel, "Activation Colours", &g_ActivationDisplay);
    //GLUI_Spinner *forceScaleSpiner = gGLUIControlWindow->add_spinner_to_panel(displayPanel,
    //                                                                          "Force Scale", GLUI_SPINNER_FLOAT, &gForceLineScale);
    //forceScaleSpiner->set_float_limits(0.0, 0.1);
    //forceScaleSpiner->set_w(DEFAULT_TEXT_WIDTH);
    //forceScaleSpiner->set_alignment(GLUI_ALIGN_RIGHT);
    
#if ! defined USE_TRANSLATION_WIDGETS && ! defined(USE_SOCKETS) && ! defined(USE_UDP)
    // column 2
    gGLUIControlWindow->add_column( true );
#endif
    
#if ! defined(USE_SOCKETS) && ! defined(USE_UDP)
    // Movie Panel
    GLUI_Panel *moviePanel = gGLUIControlWindow->add_panel("Movie");
    GLUI_EditText *directoryNameEditText = gGLUIControlWindow->add_edittext_to_panel(moviePanel, "Directory Name", gMovieDirectory);
    directoryNameEditText->set_w(DEFAULT_TEXT_WIDTH);
    directoryNameEditText->set_alignment(GLUI_ALIGN_RIGHT);
    gGLUIControlWindow->add_checkbox_to_panel(moviePanel, "Record Movie", &gWriteMovie,
                                              MOVIE_CHECKBOX, ButtonCallback);
    gGLUIControlWindow->add_checkbox_to_panel(moviePanel, "PPM Output", &gUsePPM);
    gGLUIControlWindow->add_checkbox_to_panel(moviePanel, "TIFF Output", &gUseTIFF);
    gGLUIControlWindow->add_checkbox_to_panel(moviePanel, "POVRay Output", &gUsePOV);
    gGLUIControlWindow->add_checkbox_to_panel(moviePanel, "OBJ Output", &gUseOBJ);
    gGLUIControlWindow->add_button_to_panel(moviePanel, "Snapshot", SNAPSHOT_BUTTON, ButtonCallback);
    
    // Animation Panel
    GLUI_Panel *animatePanel = gGLUIControlWindow->add_panel("Animate");
    gGLUIControlWindow->add_button_to_panel(animatePanel, "Start", START_BUTTON, ButtonCallback);
    gGLUIControlWindow->add_button_to_panel(animatePanel, "Stop", STOP_BUTTON, ButtonCallback);
    gGLUIControlWindow->add_button_to_panel(animatePanel, "Step", STEP_BUTTON, ButtonCallback);
    gGLUIControlWindow->add_statictext_to_panel(animatePanel, "" );
    gGLUIControlWindow->add_checkbox_to_panel(animatePanel, "Slow", &gSlowFlag);
    GLUI_Spinner *frameSkipSpiner = gGLUIControlWindow->add_spinner_to_panel(animatePanel,
                                                                             "Frame Skip", GLUI_SPINNER_INT, &gDisplaySkip);
    frameSkipSpiner->set_int_limits(1, INT_MAX);
    frameSkipSpiner->set_w(DEFAULT_TEXT_WIDTH);
    frameSkipSpiner->set_alignment(GLUI_ALIGN_RIGHT);
    gTimeLimitSpinner = gGLUIControlWindow->add_spinner_to_panel(animatePanel,
                                                                 "Time Limit", GLUI_SPINNER_FLOAT);
    gTimeLimitSpinner->set_w(DEFAULT_TEXT_WIDTH);
    gTimeLimitSpinner->set_alignment(GLUI_ALIGN_RIGHT);
    gMechanicalEnergyLimitSpinner = gGLUIControlWindow->add_spinner_to_panel(animatePanel,
                                                                             "Mech. Energy Limit", GLUI_SPINNER_FLOAT);
    gMechanicalEnergyLimitSpinner->set_w(DEFAULT_TEXT_WIDTH);
    gMechanicalEnergyLimitSpinner->set_alignment(GLUI_ALIGN_RIGHT);
    gMetabolicEnergyLimitSpinner = gGLUIControlWindow->add_spinner_to_panel(animatePanel,
                                                                            "Met. Energy Limit", GLUI_SPINNER_FLOAT);
    gMetabolicEnergyLimitSpinner->set_w(DEFAULT_TEXT_WIDTH);
    gMetabolicEnergyLimitSpinner->set_alignment(GLUI_ALIGN_RIGHT);
#endif
    
    // Information Panel
    GLUI_Panel *infoPanel = gGLUIControlWindow->add_panel("Info Only");
    gGLUIEditTextCameraX = gGLUIControlWindow->add_edittext_to_panel(infoPanel,
                                                                     "Camera X", GLUI_EDITTEXT_FLOAT);
    gGLUIEditTextCameraX->set_w(DEFAULT_TEXT_WIDTH);
    gGLUIEditTextCameraX->set_alignment(GLUI_ALIGN_RIGHT);
    gGLUIEditTextCameraY = gGLUIControlWindow->add_edittext_to_panel(infoPanel,
                                                                     "Camera Y", GLUI_EDITTEXT_FLOAT);
    gGLUIEditTextCameraY->set_w(DEFAULT_TEXT_WIDTH);
    gGLUIEditTextCameraY->set_alignment(GLUI_ALIGN_RIGHT);
    gGLUIEditTextCameraZ = gGLUIControlWindow->add_edittext_to_panel(infoPanel,
                                                                     "Camera Z", GLUI_EDITTEXT_FLOAT);
    gGLUIEditTextCameraZ->set_w(DEFAULT_TEXT_WIDTH);
    gGLUIEditTextCameraZ->set_alignment(GLUI_ALIGN_RIGHT);
    gGLUIControlWindow->add_statictext_to_panel(infoPanel, "" );
    gGLUIEditTextSimulationTime = gGLUIControlWindow->add_edittext_to_panel(infoPanel, "Sim. Time", GLUI_EDITTEXT_FLOAT);
    gGLUIEditTextSimulationTime->set_w(DEFAULT_TEXT_WIDTH);
    gGLUIEditTextSimulationTime->set_alignment(GLUI_ALIGN_RIGHT);
    gGLUIEditTextMechanicalEnergy = gGLUIControlWindow->add_edittext_to_panel(infoPanel, "Mech. Energy", GLUI_EDITTEXT_FLOAT);
    gGLUIEditTextMechanicalEnergy->set_w(DEFAULT_TEXT_WIDTH);
    gGLUIEditTextMechanicalEnergy->set_alignment(GLUI_ALIGN_RIGHT);
    gGLUIEditTextMetabolicEnergy = gGLUIControlWindow->add_edittext_to_panel(infoPanel, "Met. Energy", GLUI_EDITTEXT_FLOAT);
    gGLUIEditTextMetabolicEnergy->set_w(DEFAULT_TEXT_WIDTH);
    gGLUIEditTextMetabolicEnergy->set_alignment(GLUI_ALIGN_RIGHT);
    gGLUIEditTextFitness = gGLUIControlWindow->add_edittext_to_panel(infoPanel, "Fitness", GLUI_EDITTEXT_FLOAT);
    gGLUIEditTextFitness->set_w(DEFAULT_TEXT_WIDTH);
    gGLUIEditTextFitness->set_alignment(GLUI_ALIGN_RIGHT);
    
    // debug panel
    if (gDebug == GLUIDebug)
    {
        GLUI_Panel *debugPanel = gGLUIControlWindow->add_panel("Debug");
        GLUI_Listbox *debugListBox =
            gGLUIControlWindow->add_listbox_to_panel
            (debugPanel, "Level", &gDebugListItem);
        for (int i = 0; i < 13; i++)
            debugListBox->add_item(i , (char *)gDebugLabels[i]);
        gGLUIControlWindow->add_checkbox_to_panel(debugPanel, "Debug to File", &gDebugWrite,
                                                  DEBUG_CHECKBOX, ButtonCallback);
    }
    
#ifdef QUIT_BUTTON_REQUIRED
    gGLUIControlWindow->add_statictext( "" );
    
    /****** A 'quit' button *****/
    gGLUIControlWindow->add_button( "Quit", 0,(GLUI_Update_CB)exit );
#endif
    
    // tell the gGLUIControlWindow window which the main window is
    gGLUIControlWindow->set_main_gfx_window(gMainWindow);
    
    /**** We register the idle callback with GLUI, *not* with GLUT ****/
    GLUI_Master.set_glutIdleFunc( myGlutIdle );
    
}

/***************************************** myGlutDisplay() *****************/

void myGlutDisplay( void )
{
    /****************************************/
    /*       Set up OpenGL lights           */
    /****************************************/
    
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, gLight0Ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, gLight0Diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, gLight0Specular);
    glLightfv(GL_LIGHT0, GL_POSITION, gLight0Position);
    
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, gLight1Ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, gLight1Diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, gLight1Specular);
    glLightfv(GL_LIGHT1, GL_POSITION, gLight1Position);
    
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
        
    static bool neverClearedFlag = true;
    
    if (gWhiteBackground) glClearColor( 1.0f, 1.0f, 1.0f, 1.0f );
    else glClearColor( .0f, .0f, .0f, 1.0f );
    
    if (neverClearedFlag)
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        neverClearedFlag = false;
    }
    else
    {
        if (gOverlayFlag == false)
            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    }
    
    
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    //gluPerspective(gFOV, gXYAspect, 0.01, 1000);
    // this almost certainly won't work in the general case
    GLfloat frontBackDistance = 10;
    GLfloat frontClip = ABS(gCameraDistance) - frontBackDistance / 2;
    GLfloat backClip = ABS(gCameraDistance) + frontBackDistance / 2;
    if (frontClip < 0.1)
    {
        frontClip = 0.1;
        backClip = frontBackDistance;
    }
    gluPerspective(gFOV, gXYAspect, frontClip, backClip);
    
    glMatrixMode( GL_MODELVIEW );
    
    glLoadIdentity();
    glMultMatrixf( gLightsRotation );
    glLightfv(GL_LIGHT0, GL_POSITION, gLight0Position);
    
    glLoadIdentity();
    
    // note rotation - need to swap Y and Z because of different up vector
    // for the main view and the rotation widget
    mat4 myRotation(
                    gViewRotation[0], gViewRotation[1], gViewRotation[2], gViewRotation[3],
                    gViewRotation[4], gViewRotation[5], gViewRotation[6], gViewRotation[7],
                    gViewRotation[8], gViewRotation[9], gViewRotation[10], gViewRotation[11],
                    gViewRotation[12], gViewRotation[13], gViewRotation[14], gViewRotation[15]);
    
    vec4 myEye(0.0, 0.0, gCameraDistance, 1.0);
    vec3 myEyeTransformed = (vec3)(myRotation * myEye);
    
    gCameraX = gCOIx + myEyeTransformed[0];
    gCameraY = gCOIy + myEyeTransformed[2];
    gCameraZ = gCOIz + myEyeTransformed[1];
    pgd::Vector viewDirection(gCOIx - gCameraX, gCOIy - gCameraY, gCOIz - gCameraZ);
    viewDirection.Normalize();
    double angle = fabs(pgd::RadiansToDegrees(acos(viewDirection * pgd::Vector(0, 0, 1))));
    if (angle < 5 || angle > 175)
        gluLookAt( gCameraX, gCameraY, gCameraZ,
                   gCOIx, gCOIy, gCOIz,
                   0.0, 1.0, 0);
    else
        gluLookAt( gCameraX, gCameraY, gCameraZ,
                   gCOIx, gCOIy, gCOIz,
                   0.0, 0.0, 1.0);
    
    glScalef( gScale, gScale, gScale );
    
    // ===============================================================
    
    if (gFinishedFlag == false)
    {
        gSimulation->Draw();
    }
    
    
    glEnable(GL_LIGHTING);
    
    glutSwapBuffers();
}  


// write a PPM file (need to invert the y axis)
void WritePPM(const char *pathname, int width, int height, unsigned char *rgb)
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
void WriteTIFF(const char *pathname, int width, int height, unsigned char *rgb)
{
    TIFFWrite tiff;
    int i;
    
    tiff.initialiseImage(width, height, 72, 72, 3);
    // need to invert write order
    for (i = 0; i < height; i ++)
        tiff.copyRow(height - i - 1, rgb + (i * width * 3));
    
    tiff.writeToFile((char *)pathname);
}

// button callback function
void ButtonCallback( int control )
{
    int i;
    switch (control)
    {
        case STEP_BUTTON:
            gStepFlag = true;
            gRunFlag = false;
            break;
        case START_BUTTON:
            gRunFlag = true;
            break;
        case STOP_BUTTON:
            gRunFlag = false;
            break;
        case SNAPSHOT_BUTTON:
#ifdef USE_OSMESA
            OSMesaFrameGrab(gFrameGrabWindowWidth, gFrameGrabWindowHeight, "snapshot-hires.ppm");
#endif
            WriteOutput("snapshot");
            break;
        case BALL_RIGHT_BUTTON:
            gViewRotationControl->reset();
            break;
        case BALL_TOP_BUTTON:
            const static float topRotation[16] = {1,0,0,0,  0,0,-1,0,  0,1,0,0,  0,0,0,1};
            for (i = 0; i < 16; i++) gViewRotation[i] = topRotation[i];
                break;
        case BALL_FRONT_BUTTON:
            const static float frontRotation[16] = {0,0,-1,0,  0,1,0,0,  1,0,0,0,  0,0,0,1};
            for (i = 0; i < 16; i++) gViewRotation[i] = frontRotation[i];
                break;
        case BALL_DAMP_CHECKBOX:
            if (gBallDamping)
                gViewRotationControl->set_spin( 0 );
            else
                gViewRotationControl->set_spin( 1.0 );
            break;
        case MOVIE_CHECKBOX:
            if (gWriteMovie)
            {
                struct stat sb;
                int err = stat(gMovieDirectory.c_str(), &sb);
                if (err) // file doesn't exist
                {
                    std::string command = "mkdir ";
                    command.append(gMovieDirectory);
                    system(command.c_str());
                    err = stat(gMovieDirectory.c_str(), &sb);
                }
                if (err) // something wrong
                {
                    gWriteMovie = false;
                    break;
                }
                if ((sb.st_mode & S_IFDIR) == 0)
                {
                    gWriteMovie = false;
                    break;
                }
            }
            break;
        case DEBUG_CHECKBOX:
        {
            if (gDebugWrite)
            {
                char filename[256];
                time_t theTime = time(0);
                struct tm *theLocalTime = localtime(&theTime);
                sprintf(filename, "%s_%04d-%02d-%02d_%02d.%02d.%02d.log",
                        gDebugLabels[gDebugListItem],
                        theLocalTime->tm_year + 1900, theLocalTime->tm_mon + 1,
                        theLocalTime->tm_mday,
                        theLocalTime->tm_hour, theLocalTime->tm_min,
                        theLocalTime->tm_sec);
                std::ofstream *file = new std::ofstream();
                file->open(filename, std::ofstream::out | std::ofstream::app);
                gDebugStream = file;
                gDebug = (DebugControl)gDebugListItem;
            }
            else
            {
                std::ofstream *file = dynamic_cast<std::ofstream *>(gDebugStream);
                if (file != (std::ostream *)&std::cerr && file != 0)
                {
                    file->close();
                    delete file;
                }
                gDebugStream = &std::cerr;
                gDebug = NoDebug; // we don't want to debug to stderr
            }
        }
            break;
    }
}

// write the current display out to a file
// filename is without extension
void WriteOutput(const char *subpath)
{
    std::string pathname = subpath;
    if (gUsePPM)
    {
        int tx, ty, tw, th;
        GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
        unsigned char *rgb = new unsigned char[tw * th * 3];
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, tw, th, GL_RGB, GL_UNSIGNED_BYTE, rgb);
        std::string ppmpathname = pathname + ".ppm";
        WritePPM(ppmpathname.c_str(), tw, th, (unsigned char *)rgb);
        delete [] rgb;
    }
    if (gUseTIFF)
    {
        int tx, ty, tw, th;
        GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
        unsigned char *rgb = new unsigned char[tw * th * 3];
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, tw, th, GL_RGB, GL_UNSIGNED_BYTE, rgb);
        std::string tiffpathname = pathname + ".tif";
        WriteTIFF(tiffpathname.c_str(), tw, th, (unsigned char *)rgb);
        delete [] rgb;
    }
    if (gUsePOV) 
    {
        gDestinationOpenGL = false;
        gDestinationPOVRay = true;
        gDestinationOBJFile = false;
        std::string povpathname = pathname + ".pov";
        gPOVRayFile = new std::ofstream(povpathname.c_str());
        (*gPOVRayFile) << "#declare gCameraX = " << gCameraX << " ;\n";
        (*gPOVRayFile) << "#declare gCameraY = " << gCameraY << " ;\n";
        (*gPOVRayFile) << "#declare gCameraZ = " << gCameraZ << " ;\n";
        (*gPOVRayFile) << "#declare gCOIx = " << gCOIx << " ;\n";
        (*gPOVRayFile) << "#declare gCOIy = " << gCOIy << " ;\n";
        (*gPOVRayFile) << "#declare gCOIz = " << gCOIz << " ;\n";
        (*gPOVRayFile) << "#declare gTime = " << gSimulation->GetTime() << " ;\n";
 
        (*gPOVRayFile) << "\n#include \"camera.pov\"\n\n";
        gSimulation->Draw();
        delete gPOVRayFile;
        gPOVRayFile = 0;
        gDestinationOpenGL = true;
        gDestinationPOVRay = false;
        gDestinationOBJFile = false;
    }
    if (gUseOBJ)    
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
}

#ifdef USE_OSMESA

// OSMesa code adapted from the Mesa 5 demo osdemo.c

// returns zero on success
int OSMesaFrameGrab(int WIDTH, int HEIGHT, char *filename)
{
    printf("Starting osmesa frame grab (width = %d height = %d)\n", WIDTH, HEIGHT);
    
    OSMesaContext ctx = OSMesaCreateContextExt( OSMESA_RGBA, 16, 0, 0, NULL );
    if (!ctx) 
    {
        printf("OSMesaCreateContext failed!\n");
        return 1;
    }

    /* Allocate the image buffer */
    void *buffer = malloc( WIDTH * HEIGHT * 4 * sizeof(GLubyte) );
    if (!buffer) 
    {
        printf("Alloc image buffer failed!\n");
        return 1;
    }
    
    /* Bind the buffer to the context and make it current */
    if (!OSMesaMakeCurrent( ctx, buffer, GL_UNSIGNED_BYTE, WIDTH, HEIGHT )) 
    {
        printf("OSMesaMakeCurrent failed!\n");
        return 1;
    }
    
    {
        int z, s, a;
        glGetIntegerv(GL_DEPTH_BITS, &z);
        glGetIntegerv(GL_STENCIL_BITS, &s);
        glGetIntegerv(GL_ACCUM_RED_BITS, &a);
        printf("Depth=%d Stencil=%d Accum=%d\n", z, s, a);
    }
    
    glViewport( 0, 0, WIDTH, HEIGHT );
    gXYAspect = (float)WIDTH / (float)HEIGHT;

    myGlutDisplay();
    OSMesaWritePPM(filename, (const GLubyte *)buffer, WIDTH, HEIGHT, 1);
    
    /* free the image buffer */
    free( buffer );
    
    /* destroy the context */
    OSMesaDestroyContext( ctx );
    
    // and back to the old window
    glutSetWindow(gMainWindow);
    myGlutReshape(gGlutReshapeX, gGlutReshapeY);
    
    printf("all done\n");
    
    return 0;
}

void OSMesaWritePPM(const char *filename, const GLubyte *buffer, int width, int height, int binary)
{
    FILE *f = fopen( filename, "w" );
    if (f) 
    {
        int i, x, y;
        const GLubyte *ptr = buffer;
        if (binary) 
        {
            fprintf(f,"P6\n");
            fprintf(f,"# ppm file created by OSMesaWritePPM in GaitSymODE\n");
            fprintf(f,"%i %i\n", width,height);
            fprintf(f,"255\n");
            fclose(f);
            f = fopen( filename, "ab" );  /* reopen in binary append mode */
            for (y=height-1; y>=0; y--) 
            {
                for (x=0; x<width; x++) 
                {
                    i = (y*width + x) * 4;
                    fputc(ptr[i], f);   /* write red */
                    fputc(ptr[i+1], f); /* write green */
                    fputc(ptr[i+2], f); /* write blue */
                }
            }
        }
        else 
        {
            /*ASCII*/
            int counter = 0;
            fprintf(f,"P3\n");
            fprintf(f,"# ascii ppm file created by OSMesaWritePPM in GaitSymODE\n");
            fprintf(f,"%i %i\n", width, height);
            fprintf(f,"255\n");
            for (y=height-1; y>=0; y--) 
            {
                for (x=0; x<width; x++) 
                {
                    i = (y*width + x) * 4;
                    fprintf(f, " %3d %3d %3d", ptr[i], ptr[i+1], ptr[i+2]);
                    counter++;
                    if (counter % 5 == 0)
                        fprintf(f, "\n");
                }
            }
        }
    fclose(f);
    }
}
#endif // USE_OSMESA

#endif // USE_OPENGL

#endif // USE_CARBON
