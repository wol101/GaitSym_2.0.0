/*
 *  IrrlichtRoutines.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/02/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

#ifndef IrrlichtRoutines_h
#define IrrlichtRoutines_h

#include <irrlicht.h>


// Declare a structure to hold some context for the event receiver so that it
// has it available inside its OnEvent() method.
struct SAppContext
{
    irr::IrrlichtDevice *device;
    irr::s32				counter;
    irr::gui::IGUIListBox*	listbox;
};

// Define some values that we'll use to identify individual GUI controls.
enum
{
	GUI_ID_QUIT_BUTTON = 101,
	GUI_ID_BBOX_CHECK,
	GUI_ID_WIREFRAME_CHECK,
    GUI_ID_YUP_CHECK
};

/*
 The Event Receiver is not only capable of getting keyboard and
 mouse input events, but also events of the graphical user interface
 (gui). There are events for almost everything: Button click,
 Listbox selection change, events that say that a element was hovered
 and so on. To be able to react to some of these events, we create
 an event receiver.
 We only react to gui events, and if it's such an event, we get the
 id of the caller (the gui element which caused the event) and get
 the pointer to the gui environment.
 */
class MyEventReceiver : public irr::IEventReceiver
{
public:
        
    MyEventReceiver(SAppContext & context) : Context(context) { };
        
    virtual bool OnEvent(const irr::SEvent& event);
        
protected:
    
    SAppContext & Context;
};


// this routine starts up Irrlicht. It returns when the program terminates
// It returns 0 on a normal exit
int StartIrrlicht(void);

#endif
