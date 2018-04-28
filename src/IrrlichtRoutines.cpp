/*
 *  IrrlichtRoutines.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/02/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

#ifdef USE_IRRLICHT

#include <iostream>
#include <map>
#include <string>

#include "IrrlichtRoutines.h"

#include "FacetedObject.h"
#include "Face.h"
#include "Body.h"
#include "Simulation.h"

/*
 In the Irrlicht Engine, everything can be found in the namespace 'irr'. So if
 you want to use a class of the engine, you have to write irr:: before the name
 of the class. For example to use the IrrlichtDevice write: irr::IrrlichtDevice.
 To get rid of the irr:: in front of the name of every class, we tell the
 compiler that we use that namespace from now on, and we will not have to write
 irr:: anymore.
 */
using namespace irr;

// Simulation global
extern Simulation *gSimulation;

// external functions
int ReadModel();

// globals inherited from the GLUI interface
int gAxisFlag = true;
int gDrawMuscleForces = true;
int gDrawContactForces = true;
int gDrawMuscles = true;
int gDrawBonesFlag = true;
int gDrawGeomsFlag = true;
int gWhiteBackground = false;
int gBadMesh = true;
int g_ActivationDisplay = true;

bool gBoundingBox = false;
bool gWireFrame = false;
bool gYUp = false;

static scene::IAnimatedMesh* CreateMesh(FacetedObject* fo, scene::ISceneManager* smgr);

// this routine starts up Irrlicht. It returns when the program terminates
// It returns 0 on a normal exit
int StartIrrlicht(void)
{
	/*
     The most important function of the engine is the createDevice()
     function. The IrrlichtDevice is created by it, which is the root
     object for doing anything with the engine. createDevice() has 7
     parameters:
     
     - deviceType: Type of the device. This can currently be the Null-device,
     one of the two software renderers, D3D8, D3D9, or OpenGL. In this
     example we use EDT_SOFTWARE, but to try out, you might want to
     change it to EDT_BURNINGSVIDEO, EDT_NULL, EDT_DIRECT3D8,
     EDT_DIRECT3D9, or EDT_OPENGL.
     
     - windowSize: Size of the Window or screen in FullScreenMode to be
     created. In this example we use 640x480.
     
     - bits: Amount of color bits per pixel. This should be 16 or 32. The
     parameter is often ignored when running in windowed mode.
     
     - fullscreen: Specifies if we want the device to run in fullscreen mode
     or not.
     
     - stencilbuffer: Specifies if we want to use the stencil buffer (for
     drawing shadows).
     
     - vsync: Specifies if we want to have vsync enabled, this is only useful
     in fullscreen mode.
     
     - eventReceiver: An object to receive events. We do not want to use this
     parameter here, and set it to 0.
     
     Always check the return value to cope with unsupported drivers,
     dimensions, etc.
     */
    
	IrrlichtDevice *device =
    createDevice( video::EDT_OPENGL, core::dimension2d<s32>(1466, 1100), 16, false, false, false, 0);
    
	if (!device)
    {
        std::cerr << "Error initialising IrrlichtDevice\n";
        return __LINE__;
    }
    
	device->setWindowCaption(L"GaitSym: Irrlicht Engine");
    
	/*
     Get a pointer to the video driver and the SceneManager so that
     we do not always have to call irr::IrrlichtDevice::getVideoDriver() and
     irr::IrrlichtDevice::getSceneManager().
     */
	video::IVideoDriver* driver = device->getVideoDriver();
	scene::ISceneManager* smgr = device->getSceneManager();
    
    // for the time being use a fixed camera. 
    scene::ICameraSceneNode* camera = smgr->addCameraSceneNode(0, // parent
                             core::vector3df(-50, 0, 0), // position
                             core::vector3df(0, 0, 0), // lookat
                             -1); // id 
    camera->setFOV(0.1);
    if (gYUp) 
    {
        camera->setUpVector(core::vector3df(0, 1, 0));
        camera->setAspectRatio(4.0/3.0);
    }
    else 
    {
        camera->setUpVector(core::vector3df(0, 0, 1));
        camera->setAspectRatio(-4.0/3.0);
    }
    //camera->setPosition(core::vector3df(-10, 0, 0));
    //camera->setTarget(core::vector3df(0, 0, 0));
        
	/* store pointers to the gui environment. */
    gui::IGUIEnvironment* env = device->getGUIEnvironment();

	/*
     We add some buttons. The third
     parameter is the id of the button, with which we can easily identify
     the button in the event receiver.
     */	
    
    int x = 10;
    int y = 10;
    int h = 20;
    int w = 100;
    int g = 10;
    
    // debug checkboxes
    env->addCheckBox(gBoundingBox, core::rect<s32>(x, y, x + w, y + h), 0, GUI_ID_BBOX_CHECK, L"Bounding Box");
    y += h + g;
    env->addCheckBox(gWireFrame, core::rect<s32>(x, y, x + w, y + h), 0, GUI_ID_WIREFRAME_CHECK, L"Wire Frame");
    y += h + g;    
    env->addCheckBox(gWireFrame, core::rect<s32>(x, y, x + w, y + h), 0, GUI_ID_YUP_CHECK, L"Y Up");
    y += h + g;    
    env->addButton(core::rect<s32>(x, y, x + w, y + h), 0, GUI_ID_QUIT_BUTTON, L"Quit", L"Exits Program");
    y += h + g;    
    
    // Store the appropriate data in a context structure.
	SAppContext context;
	context.device = device;
	context.counter = 0;
	// context.listbox = listbox;
    
	// Then create the event receiver, giving it that context structure.
	MyEventReceiver receiver(context);
    
	// And tell the device to use our custom event receiver.
	device->setEventReceiver(&receiver);
    
    // now create the scene
    scene::IAnimatedMesh* mesh;
    scene::ISceneNode* node = 0;
    core::matrix4 matrix;
    ReadModel();
    std::map<std::string, Body *> *bodyList = gSimulation->GetBodyList();
    std::map<std::string, Body *>::const_iterator iter1;
    for (iter1 = bodyList->begin(); iter1 != bodyList->end(); iter1++) 
    {
        mesh = CreateMesh(iter1->second->GetFacetedObject(), smgr);
        node = smgr->addMeshSceneNode(mesh->getMesh(0));
        node->setName(iter1->second->GetName()->c_str());
        const dReal *R = iter1->second->GetRotation();
        const dReal *pos = iter1->second->GetPosition();
        // OpenGL style 4x4 matrix
        matrix[0]=R[0];   matrix[1]=R[4];  matrix[2]=R[8];    matrix[3]=0;
        matrix[4]=R[1];   matrix[5]=R[5];  matrix[6]=R[9];    matrix[7]=0;
        matrix[8]=R[2];   matrix[9]=R[6];  matrix[10]=R[10];  matrix[11]=0;
        matrix[12]=pos[0];matrix[13]=pos[1];matrix[14]=pos[2];matrix[15]=1;
        node->setRotation(matrix.getRotationDegrees());
        node->setPosition(matrix.getTranslation());
        
        s32 state = 0;
        if (gBoundingBox) state += scene::EDS_BBOX;
        if (gWireFrame) state += scene::EDS_MESH_WIRE_OVERLAY;
        node->setDebugDataVisible(state);
    }            
    
    while(device->run() && driver)
    {
        if (device->isWindowActive())
        {
            for (iter1 = bodyList->begin(); iter1 != bodyList->end(); iter1++) 
            {
                node = smgr->getSceneNodeFromName(iter1->second->GetName()->c_str());
                const dReal *R = iter1->second->GetRotation();
                const dReal *pos = iter1->second->GetPosition();
                // OpenGL style 4x4 matrix
                matrix[0]=R[0];   matrix[1]=R[4];  matrix[2]=R[8];    matrix[3]=0;
                matrix[4]=R[1];   matrix[5]=R[5];  matrix[6]=R[9];    matrix[7]=0;
                matrix[8]=R[2];   matrix[9]=R[6];  matrix[10]=R[10];  matrix[11]=0;
                matrix[12]=pos[0];matrix[13]=pos[1];matrix[14]=pos[2];matrix[15]=1;
                node->setRotation(matrix.getRotationDegrees());
                node->setPosition(matrix.getTranslation());
                
                s32 state = 0;
                if (gBoundingBox) state += scene::EDS_BBOX;
                if (gWireFrame) state += scene::EDS_MESH_WIRE_OVERLAY;
                node->setDebugDataVisible(state);
            }            

            if (gYUp) 
            {
                camera->setUpVector(core::vector3df(0, 1, 0));
                camera->setAspectRatio(4.0/3.0);
            }
            else 
            {
                camera->setUpVector(core::vector3df(0, 0, 1));
                camera->setAspectRatio(-4.0/3.0);
            }
            
            driver->beginScene(true, true, video::SColor(0,200,200,200));
            
            smgr->drawAll();
            
            env->drawAll();
            
            driver->endScene();
        }
    }
    
	device->drop();
    
    return 0;
}


// this is the main event receiver and dispatcher
bool MyEventReceiver::OnEvent(const SEvent& event)
{
    if (event.EventType == EET_GUI_EVENT)
    {
        s32 id = event.GUIEvent.Caller->getID();
        // gui::IGUIEnvironment* env = Context.device->getGUIEnvironment();
        
        switch(event.GUIEvent.EventType)
        {
            case gui::EGET_BUTTON_CLICKED:
                
                switch(id)
                {
                    case GUI_ID_QUIT_BUTTON:
                        Context.device->closeDevice();
                        return true;
                        
                        
                    default:
                        return false;
                }
                break;
                
            case gui::EGET_CHECKBOX_CHANGED:
                
                switch(id)
                {
                    case GUI_ID_BBOX_CHECK:
                        gBoundingBox = !gBoundingBox;
                        return true;
                        
                    case GUI_ID_WIREFRAME_CHECK:
                        gWireFrame = !gWireFrame;
                        return true;
                        
                    case GUI_ID_YUP_CHECK:
                        gYUp = !gYUp;
                        return true;
                        
                    default:
                        return false;
                }
                break;

            default:
                break;
        }
    }
    
    return false;
}
    
// creates an animated mesh from a FacetedObject
// return Pointer to the created mesh. Returns 0 if loading failed.
// If you no longer need the mesh, you should call IAnimatedMesh::drop().
// See IReferenceCounted::drop() for more information.
scene::IAnimatedMesh* CreateMesh(FacetedObject* fo, scene::ISceneManager* smgr)
{
    int i, j;
    Face *face;
    Vertex *vertex;
    video::S3DVertex v;
    scene::SMeshBuffer *meshbuffer = new scene::SMeshBuffer();
    core::array<int> faceCorners;
    faceCorners.reallocate(32); // should be large enough
    
    const Colour *colour = fo->GetColour();
    meshbuffer->Material.DiffuseColor.setRed((s32)(colour->r * 255));
    meshbuffer->Material.DiffuseColor.setGreen((s32)(colour->g * 255));
    meshbuffer->Material.DiffuseColor.setBlue((s32)(colour->b * 255));
    meshbuffer->Material.DiffuseColor.setAlpha((s32)(colour->alpha * 255));
    if (colour->alpha < 1.0f)
        meshbuffer->Material.MaterialType = video::EMT_TRANSPARENT_VERTEX_ALPHA; 
    else
        meshbuffer->Material.MaterialType = video::EMT_SOLID;
    meshbuffer->Material.BackfaceCulling = false;
    meshbuffer->Material.Wireframe = true;
            
    for (i = 0; i < fo->GetNumVertices(); i++)
    {
        vertex = fo->GetVertex(i);
        v.Pos.set(vertex->x, vertex->y, vertex->z);
        meshbuffer->Vertices.push_back(v);
    }

    for (i = 0; i < fo->GetNumFaces(); i++)
    {
        face = fo->GetFace(i);
        for (j = 0; j < face->GetNumVertices(); j++)
        {
            faceCorners.push_back(face->GetVertex(j));
        }
        
        // triangulate the face
        for ( u32 ui = 1; ui < faceCorners.size() - 1; ui++ )
        {
            // Add a triangle
            meshbuffer->Indices.push_back( faceCorners[ui + 1] );
            meshbuffer->Indices.push_back( faceCorners[ui] );
            meshbuffer->Indices.push_back( faceCorners[0] );
        }
        faceCorners.set_used(0); // fast clear
        faceCorners.reallocate(32);
    }
        
    meshbuffer->recalculateBoundingBox();
    smgr->getMeshManipulator()->recalculateNormals(meshbuffer);
    
	scene::SMesh *mesh = new scene::SMesh();
    mesh->addMeshBuffer( meshbuffer );
    mesh->recalculateBoundingBox();
    
	// Create the Animated mesh if there's anything in the mesh
    scene::SAnimatedMesh *animMesh = 0;
	if ( mesh->getMeshBufferCount() != 0)
	{
		animMesh = new scene::SAnimatedMesh();
		animMesh->Type = scene::EAMT_UNKNOWN;
		animMesh->addMesh(mesh);
		animMesh->recalculateBoundingBox();
	}
        
	mesh->drop();
    meshbuffer->drop();
    
	return animMesh;
}

#endif


                         
