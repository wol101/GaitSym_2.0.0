/*
 *  Simulation.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Simulation.h - this simulation object is used to encapsulate
// a dynamechs simulation

#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include <map>
#include <string>
#include <fstream>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include <ode/ode.h>

#include "Environment.h"
#include "DataFile.h"

class Body;
class Joint;
class Geom;
class Muscle;
class Driver;
class DataTarget;
class Contact;
class Marker;

#ifdef USE_OPENGL
enum DrawingOrderControl
{
    DOCEnvironment, DOCJoint, DOCMuscle, DOCGeom, DOCBody
        };

struct Interface
{
    GLfloat EnvironmentAxisSize[3];
    Colour EnvironmentColour;
    GLfloat BodyAxisSize[3];
    Colour BodyColour;
    GLfloat JointAxisSize[3];
    Colour JointColour;
    Colour GeomColour;
    Colour StrapColour;
    GLfloat StrapRadius;
    Colour StrapForceColour;
    GLfloat StrapForceRadius;
    GLfloat StrapForceScale;
    Colour StrapCylinderColour;
    GLfloat StrapCylinderLength;
    GLfloat ContactAxisSize[3];
    Colour ContactColour;
    Colour ContactForceColour;
    GLfloat ContactForceRadius;
    GLfloat ContactForceScale;
    Colour MarkerColour;
    GLfloat MarkerRadius;

    std::string TrackBodyID;
    std::vector<DrawingOrderControl> DrawingOrder;
};
#endif

enum WorldStepType
{
    WorldStep,
    QuickStep,
    StepFast
};

class Simulation
{
public:

    Simulation(void);
    ~Simulation(void);

    enum FitnessType
    {
        DistanceTravelled = 0,
        KinematicMatch = 1,
        KinematicMatchMiniMax = 2
                          };

    // friend void NearCallback(void *data, dGeomID o1, dGeomID o2);
    static void NearCallback(void *data, dGeomID o1, dGeomID o2);

    int LoadModel(char *buffer);  // load parameters from the XML configuration file
    void UpdateSimulation(void);     // called at each iteration through simulation

    // get hold of various variables

    dReal GetTime(void) { return m_SimulationTime; }
    dReal GetTimeIncrement(void) { return m_StepSize; }
    long long GetStepCount(void) { return m_StepCount; }
    dReal GetMechanicalEnergy(void) { return m_MechanicalEnergy; }
    dReal GetMetabolicEnergy(void) { return m_MetabolicEnergy; }
    dReal GetTimeLimit(void) { return m_TimeLimit; }
    dReal GetMetabolicEnergyLimit(void) { return m_MetabolicEnergyLimit; }
    dReal GetMechanicalEnergyLimit(void) { return m_MechanicalEnergyLimit; }
    Body *GetBody(const char *name);
    Joint *GetJoint(const char *name);
    Marker *GetMarker(const char *name);

    void SetTimeLimit(dReal timeLimit) { m_TimeLimit = timeLimit; }
    void SetMetabolicEnergyLimit(dReal energyLimit) { m_MetabolicEnergyLimit = energyLimit; }
    void SetMechanicalEnergyLimit(dReal energyLimit) { m_MechanicalEnergyLimit = energyLimit; }
    void SetOutputModelStateAtTime(dReal outputModelStateAtTime) { m_OutputModelStateAtTime = outputModelStateAtTime; }
    void SetOutputModelStateAtCycle(dReal outputModelStateAtCycle) { m_OutputModelStateAtCycle = outputModelStateAtCycle; }
    void SetOutputKinematicsFile(const char *filename);
    void SetInputKinematicsFile(const char *filename);
    void SetOutputModelStateFile(const char *filename);
    void SetOutputWarehouseFile(const char *filename);
    void SetMungeModelStateFlag(bool f) { m_MungeModelStateFlag = f; }
    void SetMungeRotationFlag(bool f) { m_MungeRotationFlag = f; }
    void SetModelStateRelative(bool f) { m_ModelStateRelative = f; }

    // get hold of the internal lists (HANDLE WITH CARE)
    std::map<std::string, Body *> *GetBodyList() { return &m_BodyList; }
    std::map<std::string, Joint *> *GetJointList() { return &m_JointList; }
    std::map<std::string, Geom *> *GetGeomList() { return &m_GeomList; }
    std::map<std::string, Muscle *> *GetMuscleList() { return &m_MuscleList; }
    std::map<std::string, Driver *> *GetDriverList() { return &m_DriverList; }
    std::map<std::string, DataTarget *> *GetDataTargetList() { return &m_DataTargetList; }
    std::map<std::string, Marker *> *GetMarkerList() { return &m_MarkerList; }

    // fitness related values

    bool TestForCatastrophy();
    dReal CalculateInstantaneousFitness();
    bool ShouldQuit();
    void SetContactAbort(bool contactAbort) { m_ContactAbort = contactAbort; }
    void SetDataTargetAbort(bool dataTargetAbort) { m_DataTargetAbort = dataTargetAbort; }

    // format control
    void SetOldStyleInputs(bool oldStyleInputs)  { m_OldStyleInputs = oldStyleInputs; }
    void SetOldStyleOutputs(bool oldStyleOutputs)  { m_OldStyleOutputs = oldStyleOutputs; }

    // these should probably only be internal
    void InputKinematics();
    void OutputKinematics();
    void OutputProgramState();
    void OutputWarehouse();

    void Dump();


    // draw the simulation
#ifdef USE_OPENGL
    void Draw();
    Interface *GetInterface() { return &m_Interface; }
    std::vector<Geom *> *GetPickGeomList() { return &m_PickGeomList; }
#endif

protected:

    void ParseGlobal(xmlNodePtr cur);
    void ParseEnvironment(xmlNodePtr cur);
    void ParseBody(xmlNodePtr cur);
    void ParseGeom(xmlNodePtr cur);
    void ParseJoint(xmlNodePtr cur);
    void ParseMuscle(xmlNodePtr cur);
    void ParseDriver(xmlNodePtr cur);
    void ParseDataTarget(xmlNodePtr cur);
    void ParseIOControl(xmlNodePtr cur);
    void ParseMarker(xmlNodePtr cur);
    std::vector<xmlNodePtr> m_TagContentsList;

    char *DoXmlGetProp(xmlNode *cur, const xmlChar *name);
    xmlAttr *DoXmlReplaceProp(xmlNode *cur, const xmlChar *name, const xmlChar *newValue);

    std::map<std::string, Body *>m_BodyList;
    std::map<std::string, Joint *>m_JointList;
    std::map<std::string, Geom *>m_GeomList;
    std::map<std::string, Muscle *>m_MuscleList;
    std::map<std::string, Driver *>m_DriverList;
    std::map<std::string, DataTarget *>m_DataTargetList;
    std::map<std::string, Marker *>m_MarkerList;
    bool m_DataTargetAbort;

    // Simulation variables
    dWorldID m_WorldID;
    dSpaceID m_SpaceID;
    dJointGroupID m_ContactGroup;
    Environment *m_Environment;
    int m_MaxContacts;
    bool m_AllowInternalCollisions;
    WorldStepType m_StepType;

    // keep track of simulation time

    dReal m_SimulationTime; // current time
    dReal m_StepSize; // step size
    long long m_StepCount; // number of steps taken
    dReal m_CycleTime;

    // and calculated energy

    dReal m_MechanicalEnergy;
    dReal m_MetabolicEnergy;
    dReal m_BMR;

    // FitnessType
    FitnessType m_FitnessType;
    dReal m_TimeLimit;
    dReal m_MechanicalEnergyLimit;
    dReal m_MetabolicEnergyLimit;
    dReal m_KinematicMatchMiniMaxFitness;

    // some control values
    bool m_InputKinematicsFlag;
    DataFile m_InputKinematicsFile;
    bool m_OutputKinematicsFlag;
    bool m_OutputWarehouseFlag;
    std::string m_OutputKinematicsFilename;
    std::ofstream m_OutputKinematicsFile;
    std::string m_OutputModelStateFilename;
    std::string m_OutputWarehouseFilename;
    std::ofstream m_OutputWarehouseFile;
    dReal m_OutputModelStateAtTime;
    dReal m_OutputModelStateAtCycle;
    bool m_MungeModelStateFlag;
    bool m_MungeRotationFlag;
    int m_SimulationError;
    bool m_ModelStateRelative;

    // internal memory allocations
    // it will crash if these are exceeded but they should be plenty big enough
    int m_BufferSize;
    char *m_Buffer;
    char *m_Buffer2;
    char **m_BufferPtrs;
    dReal *m_DoubleList;

    // for fitness calculations
    dReal m_KinematicMatchFitness;
    std::string m_DistanceTravelledBodyIDName;
    Body *m_DistanceTravelledBodyID;

    // internal formatting controls
    bool m_OldStyleInputs;
    bool m_OldStyleOutputs;

    // contact joint list
    std::vector<Contact *> m_ContactList;
    bool m_ContactAbort;

    // values for energy partition
    dReal m_PositiveMechanicalWork;
    dReal m_NegativeMechanicalWork;
    dReal m_PositiveContractileWork;
    dReal m_NegativeContractileWork;
    dReal m_PositiveSerialElasticWork;
    dReal m_NegativeSerialElasticWork;
    dReal m_PositiveParallelElasticWork;
    dReal m_NegativeParallelElasticWork;

#ifdef USE_OPENGL
    void ParseInterface(xmlNodePtr cur);
    Interface m_Interface;
    std::vector<Geom *> m_PickGeomList;
#endif
};



#endif //__SIMULATION_H__
