/*
 *  Simulation.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Simulation.cpp - this simulation object is used to encapsulate
// a dynamechs simulation

// this is an encapsulated version of the main routine used in most of Scott McMillan's examples
// with various bits and pieces added and removed and moved.

// modified by wis to include a specific definintion of the model
// to allow me to customise things a bit more easily

#ifdef USE_OPENGL
#include <gl.h>
#include "GLUtils.h"
#include "FacetedObject.h"
#endif

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include <typeinfo>
#include <iostream>
#include <fstream>
#include <string>
#include <ctype.h>

#include <ode/ode.h>

#include "Util.h"
#include "DebugControl.h"
#include "CyclicDriver.h"
#include "StepDriver.h"
#include "DataTarget.h"
#include "DataTargetScalar.h"
#include "DataTargetQuaternion.h"
#include "DataTargetVector.h"
#include "DataFile.h"
#include "PGDMath.h"
#include "Body.h"
#include "HingeJoint.h"
#include "BallJoint.h"
#include "FloatingHingeJoint.h"
#include "CappedCylinderGeom.h"
#include "SphereGeom.h"
#include "Muscle.h"
#include "MAMuscle.h"
#include "MAMuscleExtended.h"
#include "MAMuscleExtendedDamped.h"
#include "MAMuscleComplete.h"
#include "UGMMuscle.h"
#include "DampedSpringMuscle.h"
#include "TwoPointStrap.h"
#include "ThreePointStrap.h"
#include "CylinderWrapStrap.h"
#include "Environment.h"
#include "PlaneGeom.h"
#include "Contact.h"
#include "ErrorHandler.h"
#include "NPointStrap.h"
#include "FixedJoint.h"
#include "TrimeshGeom.h"
#include "Marker.h"

#include "Simulation.h"

extern char *gGraphicsRoot;
extern int gDisplaySkip;
extern int gBadMesh;

#define _I(i,j) I[(i)*4+(j)]

Simulation::Simulation()
{
    // allocate some general purpose memory
    // this is assumed to be big enough!
    m_BufferSize = 10000;
    m_Buffer = new char[m_BufferSize];
    m_Buffer2 = new char[m_BufferSize];
    m_BufferPtrs = new char *[m_BufferSize];
    m_DoubleList = new dReal [m_BufferSize];

    // initialise the ODE world
    dInitODE();
    m_WorldID = dWorldCreate();
    m_SpaceID = dSimpleSpaceCreate(0);
    m_ContactGroup = dJointGroupCreate(0);

    m_Environment = new Environment();
    m_MaxContacts = 16;

    // set some variables
    m_SimulationTime = 0;
    m_StepCount = 0;
    m_StepSize = 0;
    m_CycleTime = -1;
    m_MechanicalEnergy = 0;
    m_MetabolicEnergy = 0;
    m_FitnessType = DistanceTravelled;
    m_DistanceTravelledBodyID = 0;
    m_BMR = 0;
    m_OutputModelStateAtTime = -1;
    m_OutputModelStateAtCycle = -1;
    m_TimeLimit = 0;
    m_MechanicalEnergyLimit = 0;
    m_MetabolicEnergyLimit = 0;
    m_InputKinematicsFlag = false;
    m_OutputKinematicsFlag = false;
    m_OutputWarehouseFlag = false;
    m_OutputModelStateFilename = "ModelState.xml";
    m_OutputKinematicsFilename = "Kinematics.txt";
    m_OutputWarehouseFilename = "Warehouse.txt";
    m_KinematicMatchFitness = 0;
    m_MungeModelStateFlag = false;
    m_MungeRotationFlag = false;
    m_ModelStateRelative = true;
    m_AllowInternalCollisions = true;
    m_StepType = WorldStep;
    m_ContactAbort = false;
    m_SimulationError = 0;
    m_DataTargetAbort = false;
    m_KinematicMatchMiniMaxFitness = 0;

    // values for energy partition
    m_PositiveMechanicalWork = 0;
    m_NegativeMechanicalWork = 0;
    m_PositiveContractileWork = 0;
    m_NegativeContractileWork = 0;
    m_PositiveSerialElasticWork = 0;
    m_NegativeSerialElasticWork = 0;
    m_PositiveParallelElasticWork = 0;
    m_NegativeParallelElasticWork = 0;

    // format controls
    m_OldStyleInputs = true;
    m_OldStyleOutputs = true;


    dSetMessageHandler(ODEMessageTrap);

#ifdef USE_OPENGL
    m_Interface.EnvironmentAxisSize[0] = m_Interface.EnvironmentAxisSize[1] = m_Interface.EnvironmentAxisSize[2] = 1.0;
    m_Interface.EnvironmentColour.SetColour(1, 0, 1, 1);
    m_Interface.BodyAxisSize[0] = m_Interface.BodyAxisSize[1] = m_Interface.BodyAxisSize[2] = 0.05;
    m_Interface.BodyColour.SetColour(0.275, 0.725, 0.451, 1.0);
    m_Interface.JointAxisSize[0] = m_Interface.JointAxisSize[1] = m_Interface.JointAxisSize[2] = 0.05;
    m_Interface.JointColour.SetColour(0, 1, 0, 1);
    m_Interface.GeomColour.SetColour(0, 0, 1, 0.5);
    m_Interface.StrapColour.SetColour(1, 0, 0, 1);
    m_Interface.StrapRadius = 0.005;
    m_Interface.StrapForceColour.SetColour(1, 0, 0, 0.5);
    m_Interface.StrapForceRadius = 0.02;
    m_Interface.StrapForceScale = 0.0001;
    m_Interface.StrapCylinderColour.SetColour(0, 1, 1, 0.5);
    m_Interface.StrapCylinderLength = 0.1;
    m_Interface.ContactColour.SetColour(0, 0, 1, 0.5);
    m_Interface.ContactAxisSize[0] = m_Interface.ContactAxisSize[1] = m_Interface.ContactAxisSize[2] = 0.05;
    m_Interface.ContactForceColour.SetColour(0, 0, 1, 0.5);
    m_Interface.ContactForceRadius = 0.02;
    m_Interface.ContactForceScale = 0.0001;
    m_Interface.DrawingOrder.push_back(DOCEnvironment);
    m_Interface.DrawingOrder.push_back(DOCJoint);
    m_Interface.DrawingOrder.push_back(DOCMuscle);
    m_Interface.DrawingOrder.push_back(DOCGeom);
    m_Interface.DrawingOrder.push_back(DOCBody);
    m_Interface.TrackBodyID = "Torso";
#endif

}

//----------------------------------------------------------------------------
Simulation::~Simulation()
{
    dSetMessageHandler(0);

    if (gDebug == EnergyPartitionDebug)
    {
        *gDebugStream << "m_PositiveMechanicalWork " << m_PositiveMechanicalWork <<
                " m_NegativeMechanicalWork " << m_NegativeMechanicalWork <<
                " m_PositiveContractileWork " << m_PositiveContractileWork <<
                " m_NegativeContractileWork " << m_NegativeContractileWork <<
                " m_PositiveSerialElasticWork " << m_PositiveSerialElasticWork <<
                " m_NegativeSerialElasticWork " << m_NegativeSerialElasticWork <<
                " m_PositiveParallelElasticWork " << m_PositiveParallelElasticWork <<
                " m_NegativeParallelElasticWork " << m_NegativeParallelElasticWork <<
                "\n";
    }

    // get rid of all those memory alloactions

    std::map<std::string, Body *>::const_iterator iter1;
    for (iter1=m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
        delete iter1->second;

    std::map<std::string, Joint *>::const_iterator iter2;
    for (iter2=m_JointList.begin(); iter2 != m_JointList.end(); iter2++)
        delete iter2->second;

    std::map<std::string, Muscle *>::const_iterator iter3;
    for (iter3=m_MuscleList.begin(); iter3 != m_MuscleList.end(); iter3++)
        delete iter3->second;

    std::map<std::string, Driver *>::const_iterator iter4;
    for (iter4=m_DriverList.begin(); iter4 != m_DriverList.end(); iter4++)
        delete iter4->second;

    std::map<std::string, DataTarget *>::const_iterator iter5;
    for (iter5=m_DataTargetList.begin(); iter5 != m_DataTargetList.end(); iter5++)
        delete iter5->second;

    std::map<std::string, Geom *>::const_iterator iter6;
    for (iter6 = m_GeomList.begin(); iter6 != m_GeomList.end(); iter6++)
        delete iter6->second;

    std::map<std::string, Marker *>::const_iterator iter7;
    for (iter7 = m_MarkerList.begin(); iter7 != m_MarkerList.end(); iter7++)
        delete iter7->second;

    delete m_Environment;

    // destroy the ODE world
#ifdef OPENGL
    for (unsigned int i = 0; i < m_PickGeomList.size(); i++) delete m_PickGeomList[i];
#endif
    dJointGroupDestroy(m_ContactGroup);
    dSpaceDestroy(m_SpaceID);
    dWorldDestroy(m_WorldID);
    dCloseODE();

    // clear the stored xml data
    std::vector<xmlNodePtr>::const_iterator iter0;
    for (iter0 = m_TagContentsList.begin(); iter0 != m_TagContentsList.end(); iter0++)
        xmlFreeNode(*iter0);

    // delete the rest of the allocated memory
    for (unsigned int c = 0; c < m_ContactList.size(); c++) delete m_ContactList[c];


    delete [] m_Buffer;
    delete [] m_Buffer2;
    delete [] m_BufferPtrs;
    delete [] m_DoubleList;

}

//----------------------------------------------------------------------------
int Simulation::LoadModel(char *xmlDataBuffer)
{
    xmlDocPtr doc;
    xmlNodePtr cur, nodeCopy;
    int size = strlen(xmlDataBuffer);

    if (gDebug == SimulationDebug)
    {
        *gDebugStream << "Simulation::LoadModel " << size << "\n" <<
                xmlDataBuffer << "\n";
    }

    // do the basic XML parsing

    doc = xmlReadMemory(xmlDataBuffer, size, 0, 0, 0);

    if (doc == NULL )
    {
        fprintf(stderr,"Document not parsed successfully. \n");
        return 1;
    }

    cur = xmlDocGetRootElement(doc);

    if (cur == NULL)
    {
        fprintf(stderr,"Empty document\n");
        xmlFreeDoc(doc);
        return 1;
    }

    if (xmlStrcmp(cur->name, (const xmlChar *) "GAITSYMODE"))
    {
        fprintf(stderr,"Document of the wrong type, root node != GAITSYMODE");
        xmlFreeDoc(doc);
        return 1;
    }

    // now parse the elements in the file

    try
    {
        cur = cur->xmlChildrenNode;

        while (cur != NULL)
        {
            nodeCopy = xmlCopyNode(cur, 1);
            m_TagContentsList.push_back(nodeCopy);

            if (gDebug == XMLDebug)
            {
                *gDebugStream << "cur->name " << cur->name << "\n";
            }

            if ((!xmlStrcmp(cur->name, (const xmlChar *)"GLOBAL"))) ParseGlobal(cur);
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"ENVIRONMENT"))) ParseEnvironment(cur);
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"BODY"))) ParseBody(cur);
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"JOINT"))) ParseJoint(cur);
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"GEOM"))) ParseGeom(cur);
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"MUSCLE"))) ParseMuscle(cur);
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"DRIVER"))) ParseDriver(cur);
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"DATATARGET"))) ParseDataTarget(cur);
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"IOCONTROL"))) ParseIOControl(cur);
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"MARKER"))) ParseMarker(cur);
#ifdef USE_OPENGL
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"INTERFACE"))) ParseInterface(cur);
#endif

            cur = cur->next;
        }


        // and do the late initialisation
        std::map<std::string, Muscle *>::const_iterator iter2;
        for (iter2 = m_MuscleList.begin(); iter2 != m_MuscleList.end(); iter2++)
        {
            if (gDebug == XMLDebug)
            {
                *gDebugStream << iter2->first << " late initialisation\n";
            }
            iter2->second->CalculateStrap(0);
            iter2->second->SetActivation(0, 0);
        }

        THROWIFZERO(m_DistanceTravelledBodyID = m_BodyList[m_DistanceTravelledBodyIDName]);
    }

    catch(int e)
    {
        std::cerr << __FILE__ << " " << e << " Error parsing XML file: " << cur->name << "\n";
        xmlFreeDoc(doc);
        return 1;
    }

    if (m_OutputModelStateAtTime == 0.0 || m_OutputModelStateAtCycle == 0)
    {
        OutputProgramState();
        m_OutputModelStateAtTime = -1.0;
        m_OutputModelStateAtCycle = -1;
    }

    xmlFreeDoc(doc);
    return 0;
}


//----------------------------------------------------------------------------
void Simulation::UpdateSimulation()
{
    if (m_InputKinematicsFlag)
    {
        InputKinematics();
        std::map<std::string, Muscle *>::const_iterator iter1;
        for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
            iter1->second->CalculateStrap(m_StepSize);
        m_StepCount++;
        return;
    }

    // check collisions first
    dJointGroupEmpty(m_ContactGroup);
    for (unsigned int c = 0; c < m_ContactList.size(); c++) delete m_ContactList[c];
    m_ContactList.clear();
    std::map<std::string, Geom *>::const_iterator GeomIter;
    for (GeomIter = m_GeomList.begin(); GeomIter != m_GeomList.end(); GeomIter++) GeomIter->second->ClearContacts();
    dSpaceCollide(m_SpaceID, this, &NearCallback);

    // update the activation
    dReal activation;
    std::map<std::string, Driver *>::const_iterator iter2;
    for (iter2 = m_DriverList.begin(); iter2 != m_DriverList.end(); iter2++)
    {
        activation = iter2->second->GetValue(m_SimulationTime);
        iter2->second->GetTarget()->SetActivation(activation, m_StepSize);
    }

    // update the muscles
    dReal tension;
    std::vector<PointForce *> *pointForceList;
    std::map<std::string, Muscle *>::const_iterator iter1;
    PointForce *pointForce;
    for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
    {
        iter1->second->CalculateStrap(m_StepSize);

        pointForceList = iter1->second->GetPointForceList();
        tension = iter1->second->GetTension();
#ifdef DEBUG_CHECK_FORCES
        pgd::Vector force(0, 0, 0);
#endif
        for (unsigned int i = 0; i < pointForceList->size(); i++)
        {
            pointForce = (*pointForceList)[i];
            dBodyAddForceAtPos(pointForce->body->GetBodyID(),
                               pointForce->vector[0] * tension, pointForce->vector[1] * tension, pointForce->vector[2] * tension,
                               pointForce->point[0], pointForce->point[1], pointForce->point[2]);
#ifdef DEBUG_CHECK_FORCES
            force += pgd::Vector(pointForce->vector[0] * tension, pointForce->vector[1] * tension, pointForce->vector[2] * tension);
#endif
        }
#ifdef DEBUG_CHECK_FORCES
        std::cerr.setf(std::ios::floatfield, std::ios::fixed);
        std::cerr << iter1->first << " " << force.x << " " << force.y << " " << force.z << "\n";
        std::cerr.unsetf(std::ios::floatfield);
#endif
    }

    // run the simulation
    switch (m_StepType)
    {
    case WorldStep:
        dWorldStep(m_WorldID, m_StepSize);
        break;

    case QuickStep:
        dWorldQuickStep(m_WorldID, m_StepSize);
        break;

    case StepFast:
        dWorldStepFast1(m_WorldID, m_StepSize, 20);
        break;
    }

    // update the time counter
    m_SimulationTime += m_StepSize;

    // update the step counter
    m_StepCount++;

    for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
    {
        m_MechanicalEnergy += iter1->second->GetPower() * m_StepSize;
        m_MetabolicEnergy += iter1->second->GetMetabolicPower() * m_StepSize;
    }
    m_MetabolicEnergy += m_BMR * m_StepSize;

    // all reporting is done after a simulation step

    Dump();

    if (gDebug == MuscleDebug)
    {
        for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
        {
            *gDebugStream << *iter1->second->GetName() << " " << m_SimulationTime
                    << " length " << iter1->second->GetLength()
                    << " velocity " << iter1->second->GetVelocity()
                    << " tension " << iter1->second->GetTension()
                    << " power " << iter1->second->GetPower()
                    << " activation " << iter1->second->GetActivation()
                    << " metabolic " << iter1->second->GetMetabolicPower()
                    << "\n";
        }
    }

    dReal totalESE = 0;
    dReal totalEPE = 0;
    dReal totalElasticEnergy = 0;
    if (gDebug == EnergyPartitionDebug)
    {
        for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
        {
            UGMMuscle *ugm = dynamic_cast<UGMMuscle *>(iter1->second);
            if (ugm)
            {
                if (ugm->GetPower() > 0)
                    m_PositiveMechanicalWork += ugm->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += ugm->GetPower() * m_StepSize;

                if (ugm->GetVCE() < 0)
                    m_PositiveContractileWork += -1 * ugm->GetVCE() *
                                                 ugm->GetFCE() * m_StepSize;
                else
                    m_NegativeContractileWork += -1 * ugm->GetVCE() *
                                                 ugm->GetFCE() * m_StepSize;

                if (ugm->GetVSE() < 0)
                    m_PositiveSerialElasticWork += -1 * ugm->GetVSE() *
                                                   ugm->GetFSE() * m_StepSize;
                else
                    m_NegativeSerialElasticWork += -1 * ugm->GetVSE() *
                                                   ugm->GetFSE() * m_StepSize;

                if (ugm->GetVPE() < 0)
                    m_PositiveParallelElasticWork += -1 * ugm->GetVPE() *
                                                     ugm->GetFPE() * m_StepSize;
                else
                    m_NegativeParallelElasticWork += -1 * ugm->GetVPE() *
                                                     ugm->GetFPE() * m_StepSize;

                *gDebugStream << *ugm->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << ugm->GetPower() << " ContractilePower "
                        << ugm->GetFCE() << " SerialElasticPower "
                        << ugm->GetFSE() << " ParallelElasticPower "
                        << ugm->GetFPE() << " SerialElasticEnergy "
                        << ugm->GetESE() << " ParallelElasticEnergy "
                        << ugm->GetEPE() << "\n";

                totalESE += ugm->GetESE();
                totalEPE += ugm->GetEPE();
                totalElasticEnergy += ugm->GetESE();
                totalElasticEnergy += ugm->GetEPE();
            }

            MAMuscle *mam = dynamic_cast<MAMuscle *>(iter1->second);
            if (mam)
            {
                if (mam->GetPower() > 0)
                    m_PositiveMechanicalWork += mam->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += mam->GetPower() * m_StepSize;

                *gDebugStream << *mam->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << mam->GetPower() << "\n";
            }

            MAMuscleExtended *mamext = dynamic_cast<MAMuscleExtended *>(iter1->second);
            if (mamext)
            {
                if (mamext->GetPower() > 0)
                    m_PositiveMechanicalWork += mamext->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += mamext->GetPower() * m_StepSize;

                if (mamext->GetVCE() < 0)
                    m_PositiveContractileWork += -1 * mamext->GetVCE() *
                                                 mamext->GetFCE() * m_StepSize;
                else
                    m_NegativeContractileWork += -1 * mamext->GetVCE() *
                                                 mamext->GetFCE() * m_StepSize;

                if (mamext->GetVSE() < 0)
                    m_PositiveSerialElasticWork += -1 * mamext->GetVSE() *
                                                   mamext->GetFSE() * m_StepSize;
                else
                    m_NegativeSerialElasticWork += -1 * mamext->GetVSE() *
                                                   mamext->GetFSE() * m_StepSize;

                if (mamext->GetVPE() < 0)
                    m_PositiveParallelElasticWork += -1 * mamext->GetVPE() *
                                                     mamext->GetFPE() * m_StepSize;
                else
                    m_NegativeParallelElasticWork += -1 * mamext->GetVPE() *
                                                     mamext->GetFPE() * m_StepSize;

                *gDebugStream << *mamext->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << mamext->GetPower() << " ContractilePower "
                        << mamext->GetPCE() << " SerialElasticPower "
                        << mamext->GetPSE() << " ParallelElasticPower "
                        << mamext->GetPPE() << " SerialElasticEnergy "
                        << mamext->GetESE() << " ParallelElasticEnergy "
                        << mamext->GetEPE() << "\n";

                totalESE += mamext->GetESE();
                totalEPE += mamext->GetEPE();
                totalElasticEnergy += mamext->GetESE();
                totalElasticEnergy += mamext->GetEPE();
            }

            MAMuscleExtendedDamped *mamextdamp = dynamic_cast<MAMuscleExtendedDamped *>(iter1->second);
            if (mamextdamp)
            {
                if (mamextdamp->GetPower() > 0)
                    m_PositiveMechanicalWork += mamextdamp->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += mamextdamp->GetPower() * m_StepSize;

                if (mamextdamp->GetVCE() < 0)
                    m_PositiveContractileWork += -1 * mamextdamp->GetVCE() *
                                                 mamextdamp->GetFCE() * m_StepSize;
                else
                    m_NegativeContractileWork += -1 * mamextdamp->GetVCE() *
                                                 mamextdamp->GetFCE() * m_StepSize;

                if (mamextdamp->GetVSE() < 0)
                    m_PositiveSerialElasticWork += -1 * mamextdamp->GetVSE() *
                                                   mamextdamp->GetFSE() * m_StepSize;
                else
                    m_NegativeSerialElasticWork += -1 * mamextdamp->GetVSE() *
                                                   mamextdamp->GetFSE() * m_StepSize;

                if (mamextdamp->GetVPE() < 0)
                    m_PositiveParallelElasticWork += -1 * mamextdamp->GetVPE() *
                                                     mamextdamp->GetFPE() * m_StepSize;
                else
                    m_NegativeParallelElasticWork += -1 * mamextdamp->GetVPE() *
                                                     mamextdamp->GetFPE() * m_StepSize;

                *gDebugStream << *mamextdamp->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << mamextdamp->GetPower() << " ContractilePower "
                        << mamextdamp->GetPCE() << " SerialElasticPower "
                        << mamextdamp->GetPSE() << " ParallelElasticPower "
                        << mamextdamp->GetPPE() << " SerialElasticEnergy "
                        << mamextdamp->GetESE() << " ParallelElasticEnergy "
                        << mamextdamp->GetEPE() << "\n";

                totalESE += mamextdamp->GetESE();
                totalEPE += mamextdamp->GetEPE();
                totalElasticEnergy += mamextdamp->GetESE();
                totalElasticEnergy += mamextdamp->GetEPE();
            }

            DampedSpringMuscle *dsm = dynamic_cast<DampedSpringMuscle *>(iter1->second);
            if (dsm)
            {
                if (dsm->GetPower() > 0)
                    m_PositiveMechanicalWork += dsm->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += dsm->GetPower() * m_StepSize;

                *gDebugStream << *dsm->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << dsm->GetPower() << " ElasticEnergy "
                        << dsm->GetElasticEnergy() << "\n";

                totalElasticEnergy += dsm->GetElasticEnergy();
            }
        }

        dReal potentialEnergy, rotationalKineticEnergy;
        dVector3 linearKineticEnergy;
        dReal totalPotentialEnergy = 0;
        dVector3 totalLinearKineticEnergy;
        dReal totalRotationalKineticEnergy = 0;

        totalLinearKineticEnergy[0] = totalLinearKineticEnergy[1] = totalLinearKineticEnergy[2] = 0;

        std::map<std::string, Body *>::const_iterator iter5;
        for (iter5 = m_BodyList.begin(); iter5 != m_BodyList.end(); iter5++)
        {
            potentialEnergy = iter5->second->GetGravitationalPotentialEnergy();
            iter5->second->GetLinearKineticEnergy(linearKineticEnergy);
            rotationalKineticEnergy = iter5->second->GetRotationalKineticEnergy();
            *gDebugStream << *iter5->second->GetName() << " "
                    << m_SimulationTime << " "
                    << potentialEnergy << " "
                    << linearKineticEnergy[0] << " " << linearKineticEnergy[1] << " " << linearKineticEnergy[2] << " "
                    << rotationalKineticEnergy << "\n";
            totalPotentialEnergy += potentialEnergy;
            totalLinearKineticEnergy[0] += linearKineticEnergy[0];
            totalLinearKineticEnergy[1] += linearKineticEnergy[1];
            totalLinearKineticEnergy[2] += linearKineticEnergy[2];
            totalRotationalKineticEnergy += rotationalKineticEnergy;
        }
        *gDebugStream << "total_pe_lke3_rke_ese_epe" << " "
                << m_SimulationTime << " "
                << totalPotentialEnergy << " "
                << totalLinearKineticEnergy[0] << " " << totalLinearKineticEnergy[1] << " " << totalLinearKineticEnergy[2] << " "
                << totalRotationalKineticEnergy << " "
                << totalESE << " "
                << totalEPE << "\n";
    }

    if (gDebug == CentreOfMassDebug)
    {
        dVector3 cm = {0, 0, 0, 0};
        dVector3 cmv = {0, 0, 0, 0};
        const dReal *p;
        dReal mass;
        dReal totalMass = 0;
        std::map<std::string, Body *>::const_iterator iter4;
        for (iter4 = m_BodyList.begin(); iter4 != m_BodyList.end(); iter4++)
        {
            p = iter4->second->GetPosition();
            mass = iter4->second->GetMass();
            cm[0] += mass * p[0];
            cm[1] += mass * p[1];
            cm[2] += mass * p[2];
            p = iter4->second->GetLinearVelocity();
            cmv[0] += mass * p[0];
            cmv[1] += mass * p[1];
            cmv[2] += mass * p[2];
            totalMass += mass;
        }
        cm[0] /= totalMass; cm[1] /= totalMass; cm[2] /= totalMass;
        cmv[0] /= totalMass; cmv[1] /= totalMass; cmv[2] /= totalMass;
        *gDebugStream << "Time " << m_SimulationTime
                << " Mass " << totalMass
                << " CM " << cm[0] << " " << cm[1] << " " << cm[2]
                << " " << cmv[0] << " " << cmv[1] << " " << cmv[2] << "\n";
    }

    if (gDebug == JointDebug)
    {
        dJointFeedback *jointFeedback;
        std::map<std::string, Joint *>::const_iterator iter3;
        for (iter3 = m_JointList.begin(); iter3 != m_JointList.end(); iter3++)
        {
            jointFeedback = iter3->second->GetFeedback();
            *gDebugStream << "Joint " << *iter3->second->GetName() <<
                    " f1 " << jointFeedback->f1[0] << " " << jointFeedback->f1[1] << " " << jointFeedback->f1[2] << " " <<
                    " t1 " << jointFeedback->t1[0] << " " << jointFeedback->t1[1] << " " << jointFeedback->t1[2] << " " <<
                    " f2 " << jointFeedback->f2[0] << " " << jointFeedback->f2[1] << " " << jointFeedback->f2[2] << " " <<
                    " t2 " << jointFeedback->t2[0] << " " << jointFeedback->t2[1] << " " << jointFeedback->t2[2] << "\n";
            HingeJoint *hingeJoint = dynamic_cast<HingeJoint *>(iter3->second);
            if (hingeJoint)
            {
                dVector3 anchor, anchor2, axis;
                hingeJoint->GetHingeAnchor(anchor);
                hingeJoint->GetHingeAnchor2(anchor2);
                hingeJoint->GetHingeAxis(axis);
                *gDebugStream << "Joint " << *hingeJoint->GetName() <<
                        " Angle " << hingeJoint->GetHingeAngle() <<
                        " AngleRate " << hingeJoint->GetHingeAngleRate() <<
                        " Anchor " << anchor[0] << " "  << anchor[1] << " "  << anchor[2] <<
                        " Anchor2 " << anchor2[0] << " "  << anchor2[1] << " "  << anchor2[2] <<
                        " Axis " << axis[0] << " "  << axis[1] << " "  << axis[2] <<
                        "\n";
            }
            BallJoint *ballJoint = dynamic_cast<BallJoint *>(iter3->second);
            if (ballJoint)
            {
                dVector3 anchor, anchor2;
                ballJoint->GetBallAnchor(anchor);
                ballJoint->GetBallAnchor2(anchor2);
                *gDebugStream << "Joint " << *ballJoint->GetName() <<
                        " Anchor " << anchor[0] << " "  << anchor[1] << " "  << anchor[2] <<
                        " Anchor2 " << anchor2[0] << " "  << anchor2[1] << " "  << anchor2[2] <<
                        "\n";
            }        }
    }

    if (gDebug == ContactDebug)
    {
        dJointFeedback *jointFeedback;
        dBodyID bodyID;
        if (m_ContactList.size())
        {
            for (unsigned int i = 0; i < m_ContactList.size(); i++)
            {
                *gDebugStream << "Time " << m_SimulationTime << " ";
                bodyID = dJointGetBody(m_ContactList[i]->GetJointID(), 0);
                if (bodyID == 0) *gDebugStream << "Static_Environment ";
                else *gDebugStream << *((Body *)(dBodyGetData(bodyID)))->GetName() << " ";
                bodyID = dJointGetBody(m_ContactList[i]->GetJointID(), 1);
                if (bodyID == 0) *gDebugStream << "Static_Environment";
                else *gDebugStream << *((Body *)(dBodyGetData(bodyID)))->GetName();

                *gDebugStream << " x " << (*m_ContactList[i]->GetContactPosition())[0] <<
                        " y " << (*m_ContactList[i]->GetContactPosition())[1] <<
                        " z " << (*m_ContactList[i]->GetContactPosition())[2];

                jointFeedback = m_ContactList[i]->GetJointFeedback();
                *gDebugStream <<
                        " f1 " << jointFeedback->f1[0] << " " << jointFeedback->f1[1] << " " << jointFeedback->f1[2] << " " <<
                        " t1 " << jointFeedback->t1[0] << " " << jointFeedback->t1[1] << " " << jointFeedback->t1[2] << " " <<
                        " f2 " << jointFeedback->f2[0] << " " << jointFeedback->f2[1] << " " << jointFeedback->f2[2] << " " <<
                        " t2 " << jointFeedback->t2[0] << " " << jointFeedback->t2[1] << " " << jointFeedback->t2[2] << "\n";
            }
        }
        else
        {
            *gDebugStream << "Time " << m_SimulationTime << " ";
            *gDebugStream << "nil nil";
            *gDebugStream << " x " << 0 << " y " << 0 << " z " << 0;
            *gDebugStream <<
                    " f1 " << 0 << " " << 0 << " " << 0 << " " <<
                    " t1 " << 0 << " " << 0 << " " << 0 << " " <<
                    " f2 " << 0 << " " << 0 << " " << 0 << " " <<
                    " t2 " << 0 << " " << 0 << " " << 0 << "\n";
        }
    }

    if (m_FitnessType != DistanceTravelled)
    {
        dReal minScore = DBL_MAX;
        dReal matchScore;
        std::map<std::string, DataTarget *>::const_iterator iter3;
        for (iter3=m_DataTargetList.begin(); iter3 != m_DataTargetList.end(); iter3++)
        {
            if (iter3->second->TargetMatch(m_SimulationTime))
            {
                matchScore = iter3->second->GetMatchValue();
                m_KinematicMatchFitness += matchScore;
                if (matchScore < minScore)
                    minScore = matchScore;
                if (gDebug == FitnessDebug) *gDebugStream <<
                        "Simulation::UpdateSimulation m_SimulationTime " << m_SimulationTime <<
                        " DataTarget->name " << *iter3->second->GetName() <<
                        " matchScore " << matchScore <<
                        " minScore " << minScore <<
                        " m_KinematicMatchFitness " << m_KinematicMatchFitness << "\n";
            }
        }
        if (minScore < DBL_MAX)
            m_KinematicMatchMiniMaxFitness += minScore;
    }

    if (gDebug == ActivationSegmentStateDebug)
    {
        *gDebugStream << m_DriverList.size();
        std::map<std::string, Driver *>::const_iterator iter6;
        for (iter6 = m_DriverList.begin(); iter6 != m_DriverList.end(); iter6++)
        {
            *gDebugStream << "\t" << iter6->second->GetValue(m_SimulationTime);
        }

        *gDebugStream << "\t" << m_BodyList.size();
        std::map<std::string, Body *>::const_iterator iter5;
        for (iter5 = m_BodyList.begin(); iter5 != m_BodyList.end(); iter5++)
        {
            const dReal *p = iter5->second->GetPosition();
            const dReal *r = iter5->second->GetRotation();
            const dReal *v = iter5->second->GetLinearVelocity();
            const dReal *rv = iter5->second->GetAngularVelocity();
            *gDebugStream << "\t" << p[0] << "\t" << p[1] << "\t" << p[2];
            *gDebugStream << "\t" << r[0] << "\t" << r[1] << "\t" << r[2];
            *gDebugStream << "\t" << r[4] << "\t" << r[5] << "\t" << r[6];
            *gDebugStream << "\t" << r[8] << "\t" << r[9] << "\t" << r[10];
            *gDebugStream << "\t" << v[0] << "\t" << v[1] << "\t" << v[2];
            *gDebugStream << "\t" << rv[0] << "\t" << rv[1] << "\t" << rv[2];
        }
        *gDebugStream << "\n";
    }

    if (m_OutputKinematicsFlag && m_StepCount % gDisplaySkip == 0) OutputKinematics();
    if (m_OutputWarehouseFlag) OutputWarehouse();
    if (m_OutputModelStateAtTime > 0.0)
    {
        if (m_SimulationTime >= m_OutputModelStateAtTime)
        {
            OutputProgramState();
            m_OutputModelStateAtTime = 0.0;
        }
    }
    else if (m_OutputModelStateAtCycle >= 0 && m_CycleTime >= 0)
    {
        if (m_SimulationTime >= m_CycleTime * m_OutputModelStateAtCycle)
        {
            OutputProgramState();
            m_OutputModelStateAtCycle = -1;
        }
    }

}

//----------------------------------------------------------------------------
bool Simulation::TestForCatastrophy()
{
    // first of all check to see that ODE is happy
    if (IsMessage())
    {
        int num;
        const char *messageText = GetLastMessage(&num);
        std::cerr << "Failed due to ODE warning " << num << " " << messageText << "\n";
        return true;
    }

    // check for simulation error
    if (m_SimulationError)
    {
        std::cerr << "Failed due to simulation error " << m_SimulationError << "\n";
        return true;
    }

    // check for contact abort
    if (m_ContactAbort)
    {
        std::cerr << "Failed due to contact abort\n";
        return true;
    }

    // check for data target abort
    if (m_DataTargetAbort)
    {
        std::cerr << "Failed due to DataTarget abort\n";
        return true;
    }

    // check that all bodies meet velocity and stop conditions

    std::map<std::string, Body *>::const_iterator iter1;
    LimitTestResult p;
    for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
    {
        p = iter1->second->TestLimits();
        switch (p)
        {
        case WithinLimits:
            break;

        case XPosError:
        case YPosError:
        case ZPosError:
            std::cerr << "Failed due to position error " << p << " in: "
                    << *iter1->second->GetName() << "\n";
            return true;

        case XVelError:
        case YVelError:
        case ZVelError:
            std::cerr << "Failed due to velocity error " << p << " in: "
                    << *iter1->second->GetName() << "\n";
            return true;
        }
    }

    std::map<std::string, Joint *>::const_iterator iter3;
    HingeJoint *j;
    int t;
    for (iter3 = m_JointList.begin(); iter3 != m_JointList.end(); iter3++)
    {
        j = dynamic_cast<HingeJoint *>(iter3->second);
        if (j)
        {
            t = j->TestLimits();
            if (t < 0)
            {
                std::cerr << "Failed due to LoStopTorqueLimit error " << p << " in: "
                        << *iter3->second->GetName() << "\n";
                return true;
            }
            else if (t > 0)
            {
                std::cerr << "Failed due to HiStopTorqueLimit error " << p << " in: "
                        << *iter3->second->GetName() << "\n";
                return true;
            }
        }
    }

    return false;
}


//----------------------------------------------------------------------------
dReal Simulation::CalculateInstantaneousFitness()
{
    switch (m_FitnessType)
    {
    case DistanceTravelled:
        {
            const dReal *p = m_DistanceTravelledBodyID->GetPosition();
            if (isinf(p[0]) || isnan(p[0]))
            {
                m_SimulationError = 1;
                return 0;
            }
            else
            {
                return p[0];
            }
        }
    case KinematicMatch:
        {
            return m_KinematicMatchFitness;
        }
    case KinematicMatchMiniMax:
        {
            return m_KinematicMatchMiniMaxFitness;
        }
    }
    return 0;
}

void Simulation::ParseGlobal(xmlNodePtr cur)
{
    char *buf;

    dVector3 gravity;
    dReal ERP;
    dReal CFM;
    dReal contactMaxCorrectingVel;
    dReal contactSurfaceLayer;

    // gravity
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"GravityVector"));
    Util::Double(buf, 3, m_DoubleList);
    gravity[0] = m_DoubleList[0];
    gravity[1] = m_DoubleList[1];
    gravity[2] = m_DoubleList[2];

    // set the simulation integration step size
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"IntegrationStepSize"));
    m_StepSize = Util::Double(buf);

    // can specify ERP & CFM; SpringConstant & DampingConstant; SpringConstant & ERP
    // but not CFM & DampingConstant - can't think why you would want to
    buf = DoXmlGetProp(cur, (const xmlChar *)"SpringConstant");
    if (buf)
    {
        dReal ks = Util::Double(buf);
        buf = DoXmlGetProp(cur, (const xmlChar *)"DampingConstant");
        if (buf)
        {
            dReal kd = Util::Double(buf);
            ERP = m_StepSize * ks/(m_StepSize * ks + kd);
            CFM = 1.0/(m_StepSize * ks + kd);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ERP"));
            ERP = Util::Double(buf);
            CFM = ERP / (m_StepSize * ks);
        }
    }
    else
    {
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ERP"));
        ERP = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CFM"));
        CFM = Util::Double(buf);
    }

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ContactMaxCorrectingVel"));
    contactMaxCorrectingVel = Util::Double(buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ContactSurfaceLayer"));
    contactSurfaceLayer = Util::Double(buf);

    // set the global simulation parameters
    dWorldSetGravity(m_WorldID, gravity[0], gravity[1], gravity[2]);
    dWorldSetERP(m_WorldID, ERP);
    dWorldSetCFM(m_WorldID, CFM);
    dWorldSetContactMaxCorrectingVel(m_WorldID, contactMaxCorrectingVel);
    dWorldSetContactSurfaceLayer(m_WorldID, contactSurfaceLayer);

    // get the stepper required
    // WorldStep, accurate but slow
    // QuickStep, faster but less accurate
    // StepFast, not recommended - superceded by QuickStep
    buf = DoXmlGetProp(cur, (const xmlChar *)"StepType");
    if (buf)
    {
        if (strcmp((char *)buf, "WorldStep") == 0) m_StepType = WorldStep;
        else if (strcmp((char *)buf, "QuickStep") == 0) m_StepType = QuickStep;
        else if (strcmp((char *)buf, "StepFast") == 0) m_StepType = StepFast;
        else
        {
            throw __LINE__;
        }
    }

    // allow internal collisions
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"AllowInternalCollisions"));
    m_AllowInternalCollisions = Util::Bool(buf);

    // now some run parameters

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BMR"));
    m_BMR = Util::Double(buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TimeLimit"));
    m_TimeLimit = Util::Double(buf);
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MechanicalEnergyLimit"));
    m_MechanicalEnergyLimit = Util::Double(buf);
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MetabolicEnergyLimit"));
    m_MetabolicEnergyLimit = Util::Double(buf);
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"DistanceTravelledBodyID")); // DistanceTravelledBodyID is used for Munge so it is necessary anyway
    m_DistanceTravelledBodyIDName = (char *)buf;
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FitnessType"));
    if (strcmp((char *)buf, "DistanceTravelled") == 0) m_FitnessType = DistanceTravelled;
    else if (strcmp((char *)buf, "KinematicMatch") == 0) m_FitnessType = KinematicMatch;
    else if (strcmp((char *)buf, "KinematicMatchMiniMax") == 0) m_FitnessType = KinematicMatchMiniMax;
    else throw __LINE__;

    buf = DoXmlGetProp(cur, (const xmlChar *)"OutputModelStateFilename");
    if (buf)
    {
        SetOutputModelStateFile((char *)buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"OutputModelStateAtTime");
    if (buf)
    {
        SetOutputModelStateAtTime(Util::Double(buf));
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"OutputModelStateAtCycle");
    if (buf)
    {
        SetOutputModelStateAtCycle(Util::Int(buf));
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"MungeModelState");
    if (buf)
    {
        SetMungeModelStateFlag(Util::Bool(buf));
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"OutputKinematicsFile");
    if (buf)
    {
        SetOutputKinematicsFile((char *)buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"InputKinematicsFile");
    if (buf)
    {
        SetInputKinematicsFile((char *)buf);
    }

}

void Simulation::ParseEnvironment(xmlNodePtr cur)
{
    char *buf;

    // planes
    buf = DoXmlGetProp(cur, (const xmlChar *)"Plane");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        PlaneGeom *plane = new PlaneGeom(m_SpaceID, m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        plane->SetGeomLocation(Geom::environment);
        m_Environment->AddGeom(plane);
    }

#ifdef USE_OPENGL
    m_Environment->SetAxisSize(m_Interface.EnvironmentAxisSize);
    m_Environment->SetColour(m_Interface.EnvironmentColour);
#endif
}

void Simulation::ParseBody(xmlNodePtr cur)
{
    char *buf;
    dMass mass;
    dReal theMass;
    dReal I11, I22, I33, I12, I13, I23;

    // create the new body
    Body *theBody = new Body(m_WorldID);
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
    theBody->SetName((const char *)buf);

    // set the start parameters
    // note quaternion is (qs,qx,qy,qz)
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Quaternion"));
    theBody->SetQuaternion((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Position"));
    theBody->SetPosition((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"LinearVelocity"));
    Util::Double(buf, 3, m_DoubleList);
    theBody->SetLinearVelocity(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"AngularVelocity"));
    Util::Double(buf, 3, m_DoubleList);
    theBody->SetAngularVelocity(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);

    // and now the mass properties
    // (remember the origin is always at the centre of mass)

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Mass"));
    theMass = Util::Double(buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MOI"));
    Util::Double(buf, 6, m_DoubleList);

    // note: inertial matrix is as follows
    // [ I11 I12 I13 ]
    // [ I12 I22 I23 ]
    // [ I13 I23 I33 ]
    I11 = m_DoubleList[0];
    I22 = m_DoubleList[1];
    I33 = m_DoubleList[2];
    I12 = m_DoubleList[3];
    I13 = m_DoubleList[4];
    I23 = m_DoubleList[5];
    dMassSetParameters(&mass, theMass, 0, 0, 0, I11, I22, I33, I12, I13, I23);
    theBody->SetMass(&mass);

    // get limits if available
    buf = DoXmlGetProp(cur, (const xmlChar *)"PositionLowBound");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        theBody->SetPositionLowBound(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"PositionHighBound");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        theBody->SetPositionHighBound(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"LinearVelocityLowBound");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        theBody->SetLinearVelocityLowBound(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"LinearVelocityHighBound");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        theBody->SetLinearVelocityHighBound(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
    }

#ifdef USE_OPENGL
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"GraphicFile"));
    FacetedObject *facetedObject = new FacetedObject();
    std::string filename;
    if (gGraphicsRoot) filename = std::string(gGraphicsRoot) + std::string("/");
    filename += std::string((const char *)buf);
    facetedObject->ParseOBJFile(filename.c_str());

    // parameters for altering mesh

    buf = DoXmlGetProp(cur, (const xmlChar *)"Scale");
    if (buf)
    {
        GLfloat scale = Util::Double(buf);
        facetedObject->Scale(scale, scale, scale);
    }
    dReal density = -1;
    buf = DoXmlGetProp(cur, (const xmlChar *)"Density");
    if (buf)
    {
        density = Util::Double(buf);
    }
    if (density <= 0)
    {
        buf = DoXmlGetProp(cur, (const xmlChar *)"Offset");
        if (buf)
        {
            Util::Double(buf, 3, m_DoubleList);
            theBody->SetOffset(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
            facetedObject->Move(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
        }
    }

    bool clockwise = false;
    buf = DoXmlGetProp(cur, (const xmlChar *)"Clockwise");
    if (buf)
    {
        clockwise = Util::Bool(buf);
    }
    // but we always want anticlockwise objects
    if (clockwise) facetedObject->ReverseWinding();
    facetedObject->SetDrawClockwise(false);

    theBody->SetFacetedObject(facetedObject);
    theBody->SetColour(m_Interface.BodyColour);
    theBody->SetAxisSize(m_Interface.BodyAxisSize);

    if (density > 0)
    {
        // override the position values
        theBody->SetPosition(0, 0, 0);
        theBody->SetQuaternion(1, 0, 0, 0);

        facetedObject->CalculateMassProperties(&mass, density, false); // generally we assume anticlockwise winding
        std::cerr << *theBody->GetName() << " mass " << mass.mass
                << " CM " << mass.c[0] << " " << mass.c[1] << " " << mass.c[2] << " "
                << " I11_I22_I33 " << mass._I(0,0) << " " << mass._I(1,1) << " " << mass._I(2,2) << " "
                << " I12_I13_I23 " << mass._I(0,1) << " " << mass._I(0,2) << " " << mass._I(1,2) << "\n";
        const dReal *p = theBody->GetPosition();
        dVector3 newP;
        newP[0] = mass.c[0] + p[0]; newP[1] = mass.c[1] + p[1]; newP[2] = mass.c[2] + p[2];
        theBody->SetOffset(-mass.c[0], -mass.c[1], -mass.c[2]);
        facetedObject->Move(-mass.c[0], -mass.c[1], -mass.c[2]);
        facetedObject->CalculateMassProperties(&mass, density, false); // generally we assume anticlockwise winding
        std::cerr << *theBody->GetName() << " mass " << mass.mass
                << " CM " << mass.c[0] << " " << mass.c[1] << " " << mass.c[2] << " "
                << " I11_I22_I33 " << mass._I(0,0) << " " << mass._I(1,1) << " " << mass._I(2,2) << " "
                << " I12_I13_I23 " << mass._I(0,1) << " " << mass._I(0,2) << " " << mass._I(1,2) << "\n";
        mass.c[0] = mass.c[1] = mass.c[2]  = 0;
        theBody->SetMass(&mass);
        theBody->SetPosition(newP[0], newP[1], newP[2]);
    }

    // now create a dummy geom for picking
    TrimeshGeom *trimeshGeom = new TrimeshGeom(0, facetedObject);
    trimeshGeom->SetBody(theBody->GetBodyID());
    trimeshGeom->SetName(filename);
    m_PickGeomList.push_back(trimeshGeom);

#endif

    m_BodyList[*theBody->GetName()] = theBody;
}


void Simulation::ParseJoint(xmlNodePtr cur)
{
    char *buf;
    Joint *joint = 0;

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));

    if (strcmp((const char *)buf, "Hinge") == 0)
    {

        HingeJoint *hingeJoint = new HingeJoint(m_WorldID);
        joint = hingeJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        hingeJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        dBodyID body1ID = 0;
        if (strcmp((const char *)buf, "World")) THROWIFZERO(body1ID = m_BodyList[(const char *)buf]->GetBodyID());
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        dBodyID body2ID = 0;
        if (strcmp((const char *)buf, "World")) THROWIFZERO(body2ID = m_BodyList[(const char *)buf]->GetBodyID());
        hingeJoint->Attach(body1ID, body2ID);

        if (m_OldStyleInputs)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"HingeAnchor"));
            Util::Double(buf, 3, m_DoubleList);
            dVector3 result;
            dBodyGetRelPointPos(body1ID, m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], result);
            hingeJoint->SetHingeAnchor(result[0], result[1], result[2]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"HingeAxis"));
            Util::Double(buf, 3, m_DoubleList);
            dBodyVectorToWorld(body1ID, m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], result);
            hingeJoint->SetHingeAxis(result[0], result[1], result[2]);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"HingeAnchor"));
            hingeJoint->SetHingeAnchor((const char *)buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"HingeAxis"));
            hingeJoint->SetHingeAxis((const char *)buf);
        }

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"StartAngleReference"));
        hingeJoint->SetStartAngleReference(Util::GetAngle(buf));
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamLoStop"));
        dReal loStop = Util::GetAngle(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamHiStop"));
        dReal hiStop = Util::GetAngle(buf);
        hingeJoint->SetJointStops(loStop, hiStop);

        buf = DoXmlGetProp(cur, (const xmlChar *)"HiStopTorqueLimit");
        if (buf)
        {
            dReal hiStopTorqueLimit = Util::Double(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"LoStopTorqueLimit"));
            dReal loStopTorqueLimit = Util::Double(buf);
            hingeJoint->SetTorqueLimits(loStopTorqueLimit, hiStopTorqueLimit);
        }

        // can specify StopERP & StopCFM; StopSpringConstant & StopDampingConstant; StopSpringConstant & StopERP
        // but not StopCFM & StopDampingConstant - can't think why you would want to
        buf = DoXmlGetProp(cur, (const xmlChar *)"StopCFM");
        if (buf)
        {
            hingeJoint->SetStopCFM(Util::Double(buf));
        }
        buf = DoXmlGetProp(cur, (const xmlChar *)"StopERP");
        if (buf)
        {
            hingeJoint->SetStopERP(Util::Double(buf));
        }
        buf = DoXmlGetProp(cur, (const xmlChar *)"StopSpringConstant");
        if (buf)
        {
            dReal ks = Util::Double(buf);
            buf = DoXmlGetProp(cur, (const xmlChar *)"StopDampingConstant");
            if (buf)
            {
                dReal kd = Util::Double(buf);
                hingeJoint->SetStopSpringDamp(ks, kd, m_StepSize);
            }
            else
            {
                THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"StopERP"));
                dReal erp = Util::Double(buf);
                hingeJoint->SetStopSpringERP(ks, erp, m_StepSize);
            }
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StopBounce");
        if (buf)
        {
            hingeJoint->SetStopBounce(Util::Double(buf));
        }

    }

    else if (strcmp((const char *)buf, "FloatingHinge") == 0)
    {

        FloatingHingeJoint *floatingHingeJoint = new FloatingHingeJoint(m_WorldID);
        joint = floatingHingeJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        floatingHingeJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        dBodyID body1ID = 0;
        if (strcmp((const char *)buf, "World")) body1ID = m_BodyList[(const char *)buf]->GetBodyID();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        dBodyID body2ID = 0;
        if (strcmp((const char *)buf, "World")) body2ID = m_BodyList[(const char *)buf]->GetBodyID();
        floatingHingeJoint->Attach(body1ID, body2ID);

        if (m_OldStyleInputs)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FloatingHingeAxis"));
            Util::Double(buf, 3, m_DoubleList);
            dVector3 result;
            dBodyVectorToWorld(body1ID, m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], result);
            floatingHingeJoint->SetFloatingHingeAxis(result[0], result[1], result[2]);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FloatingHingeAxis"));
            floatingHingeJoint->SetFloatingHingeAxis((const char *)buf);
        }

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"StartAngleReference"));
        floatingHingeJoint->SetStartAngleReference(Util::GetAngle(buf));
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamLoStop"));
        dReal loStop = Util::GetAngle(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamHiStop"));
        dReal hiStop = Util::GetAngle(buf);
        floatingHingeJoint->SetJointStops(loStop, hiStop);
    }

    else if (strcmp((const char *)buf, "Ball") == 0)
    {

        BallJoint *ballJoint = new BallJoint(m_WorldID);
        joint = ballJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        ballJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        dBodyID body1ID = 0;
        if (strcmp((const char *)buf, "World")) THROWIFZERO(body1ID = m_BodyList[(const char *)buf]->GetBodyID());
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        dBodyID body2ID = 0;
        if (strcmp((const char *)buf, "World")) THROWIFZERO(body2ID = m_BodyList[(const char *)buf]->GetBodyID());
        ballJoint->Attach(body1ID, body2ID);

        if (m_OldStyleInputs)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BallAnchor"));
            Util::Double(buf, 3, m_DoubleList);
            dVector3 result;
            dBodyGetRelPointPos(body1ID, m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], result);
            ballJoint->SetBallAnchor(result[0], result[1], result[2]);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BallAnchor"));
            ballJoint->SetBallAnchor((const char *)buf);
        }


    }

    else if (strcmp((const char *)buf, "Fixed") == 0)
    {

        FixedJoint *fixedJoint = new FixedJoint(m_WorldID);
        joint = fixedJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        fixedJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        dBodyID body1ID = 0;
        if (strcmp((const char *)buf, "World")) body1ID = m_BodyList[(const char *)buf]->GetBodyID();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        dBodyID body2ID = 0;
        if (strcmp((const char *)buf, "World")) body2ID = m_BodyList[(const char *)buf]->GetBodyID();
        fixedJoint->Attach(body1ID, body2ID);
        fixedJoint->SetFixed();
    }

    else
    {
        throw __LINE__;
    }

#ifdef USE_OPENGL
    joint->SetAxisSize(m_Interface.JointAxisSize);
    joint->SetColour(m_Interface.JointColour);
#endif

    m_JointList[*joint->GetName()] = joint;
}

void Simulation::ParseGeom(xmlNodePtr cur)
{
    char *buf;
    Geom *geom = 0;

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));

    if (strcmp((const char *)buf, "CappedCylinder") == 0)
    {

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Radius"));
        dReal radius = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Length"));
        dReal length = Util::Double(buf);

        CappedCylinderGeom *cappedCylinderGeom = new CappedCylinderGeom(m_SpaceID, radius, length);
        geom = cappedCylinderGeom;
    }

    else if (strcmp((const char *)buf, "Sphere") == 0)
    {

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Radius"));
        dReal radius = Util::Double(buf);

        SphereGeom *cappedCylinderGeom = new SphereGeom(m_SpaceID, radius);
        geom = cappedCylinderGeom;
    }

    else
    {
        throw __LINE__;
    }

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
    geom->SetName((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BodyID"));
    dBodyID bodyID = m_BodyList[(const char *)buf]->GetBodyID();
    geom->SetBody(bodyID);

    if (m_OldStyleInputs)
    {
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Position"));
        Util::Double(buf, 3, m_DoubleList);
        geom->SetPosition(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Quaternion"));
        Util::Double(buf, 4, m_DoubleList);
        geom->SetQuaternion(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }
    else
    {
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Position"));
        geom->SetPosition((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Quaternion"));
        geom->SetQuaternion((const char *)buf);
    }

    // can specify ContactSoftERP & ContactSoftCFM; SpringConstant & DampingConstant; SpringConstant & ContactSoftERP
    // but not ContactSoftCFM & DampingConstant - can't think why you would want to
    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactSoftCFM");
    if (buf)
    {
        geom->SetContactSoftCFM(Util::Double(buf));
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactSoftERP");
    if (buf)
    {
        geom->SetContactSoftERP(Util::Double(buf));
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"SpringConstant");
    if (buf)
    {
        dReal ks = Util::Double(buf);
        buf = DoXmlGetProp(cur, (const xmlChar *)"DampingConstant");
        if (buf)
        {
            dReal kd = Util::Double(buf);
            geom->SetSpringDamp(ks, kd, m_StepSize);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ContactSoftERP"));
            dReal erp = Util::Double(buf);
            geom->SetSpringERP(ks, erp, m_StepSize);
        }
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"Bounce");
    if (buf)
    {
        geom->SetContactBounce(Util::Double(buf));
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"Mu");
    if (buf)
    {
        if (strcasecmp((const char *)buf, "infinity") == 0) geom->SetContactMu(dInfinity);
        else geom->SetContactMu(Util::Double(buf));
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"Abort");
    if (buf)
    {
        geom->SetAbort(Util::Bool(buf));
    }

#ifdef USE_OPENGL
    geom->SetColour(m_Interface.GeomColour);
#endif

    geom->SetGeomLocation(Geom::body);
    m_GeomList[*geom->GetName()] = geom;
}

void Simulation::ParseMuscle(xmlNodePtr cur)
{
    char *buf;
    Muscle *muscle;
    Strap *strap;
    std::string muscleID;
    std::string strapID;
    Body *theBody;
    dVector3 p;

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
    muscleID = (const char *)buf;

    strapID = muscleID + "Strap";

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Strap"));
    if (strcmp((char *)buf, "TwoPoint") ==  0)
    {
        // 2 attachment point muscle

        TwoPointStrap *twoPointStrap = new TwoPointStrap();
        strap = twoPointStrap;
        twoPointStrap->SetName(strapID);

        if (m_OldStyleInputs)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
            Util::Double(buf, 3, m_DoubleList);
            p[0] = m_DoubleList[0];
            p[1] = m_DoubleList[1];
            p[2] = m_DoubleList[2];
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            twoPointStrap->SetOrigin(theBody, p);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
            Util::Double(buf, 3, m_DoubleList);
            p[0] = m_DoubleList[0];
            p[1] = m_DoubleList[1];
            p[2] = m_DoubleList[2];
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            twoPointStrap->SetInsertion(theBody, p);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
            twoPointStrap->SetOrigin(theBody, (const char *)buf);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
            twoPointStrap->SetInsertion(theBody, (const char *)buf);
        }
    }
    else if (strcmp((char *)buf, "ThreePoint") ==  0)
    {
        // 3 attachment point muscle

        ThreePointStrap *threePointStrap = new ThreePointStrap();
        strap = threePointStrap;
        threePointStrap->SetName(strapID);

        if (m_OldStyleInputs)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
            Util::Double(buf, 3, m_DoubleList);
            p[0] = m_DoubleList[0];
            p[1] = m_DoubleList[1];
            p[2] = m_DoubleList[2];
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            threePointStrap->SetOrigin(theBody, p);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Midpoint"));
            Util::Double(buf, 3, m_DoubleList);
            p[0] = m_DoubleList[0];
            p[1] = m_DoubleList[1];
            p[2] = m_DoubleList[2];
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MidpointBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            threePointStrap->SetMidpoint(theBody, p);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
            Util::Double(buf, 3, m_DoubleList);
            p[0] = m_DoubleList[0];
            p[1] = m_DoubleList[1];
            p[2] = m_DoubleList[2];
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            threePointStrap->SetInsertion(theBody, p);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
            threePointStrap->SetOrigin(theBody, (const char *)buf);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MidpointBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Midpoint"));
            threePointStrap->SetMidpoint(theBody, (const char *)buf);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
            threePointStrap->SetInsertion(theBody, (const char *)buf);
        }
    }
    else if (strcmp((char *)buf, "NPoint") ==  0)
    {
        // 3 attachment point muscle

        NPointStrap *nPointStrap = new NPointStrap();
        strap = nPointStrap;
        nPointStrap->SetName(strapID);

        if (m_OldStyleInputs)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
            Util::Double(buf, 3, m_DoubleList);
            p[0] = m_DoubleList[0];
            p[1] = m_DoubleList[1];
            p[2] = m_DoubleList[2];
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            nPointStrap->SetOrigin(theBody, p);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
            Util::Double(buf, 3, m_DoubleList);
            p[0] = m_DoubleList[0];
            p[1] = m_DoubleList[1];
            p[2] = m_DoubleList[2];
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            nPointStrap->SetInsertion(theBody, p);

            int viaCount = 0;
            std::vector<Body *> bodyList;
            std::vector<dReal *> pointList;
            while (1)
            {
                sprintf(m_Buffer, "ViaPoint%d", viaCount);
                buf = DoXmlGetProp(cur, (const xmlChar *)m_Buffer);
                if (buf == 0) break;
                Util::Double(buf, 3, m_DoubleList);
                dReal *tempP = new dReal[sizeof(dVector3)];
                memcpy(tempP, m_DoubleList, sizeof(dVector3));
                pointList.push_back(tempP);
                sprintf(m_Buffer, "ViaPointBody%d", viaCount);
                THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)m_Buffer));
                THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
                bodyList.push_back(theBody);
                viaCount++;
            }
            THROWIFZERO(pointList.size());
            nPointStrap->SetViaPoints(&bodyList, &pointList);
            for (unsigned int i = 0; i < pointList.size(); i++) delete [] pointList[i];
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
            nPointStrap->SetOrigin(theBody, (const char *)buf);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
            nPointStrap->SetInsertion(theBody, (const char *)buf);

            int viaCount = 0;
            std::vector<Body *> bodyList;
            std::vector<std::string *> pointList;
            while (1)
            {
                sprintf(m_Buffer, "ViaPoint%d", viaCount);
                buf = DoXmlGetProp(cur, (const xmlChar *)m_Buffer);
                if (buf == 0) break;
                std::string *tempP = new std::string((const char *)buf);
                pointList.push_back(tempP);
                sprintf(m_Buffer, "ViaPointBody%d", viaCount);
                THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)m_Buffer));
                THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
                bodyList.push_back(theBody);
                viaCount++;
            }
            THROWIFZERO(pointList.size());
            nPointStrap->SetViaPoints(&bodyList, &pointList);
            for (unsigned int i = 0; i < pointList.size(); i++) delete pointList[i];
        }
    }
    else if (strcmp((char *)buf, "CylinderWrap") ==  0)
    {
        // cylinder wrapping muscle

        CylinderWrapStrap *cylinderWrapStrap = new CylinderWrapStrap();
        strap = cylinderWrapStrap;
        cylinderWrapStrap->SetName(strapID);

        if (m_OldStyleInputs)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
            Util::Double(buf, 3, m_DoubleList);
            p[0] = m_DoubleList[0];
            p[1] = m_DoubleList[1];
            p[2] = m_DoubleList[2];
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            cylinderWrapStrap->SetOrigin(theBody, p);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
            Util::Double(buf, 3, m_DoubleList);
            p[0] = m_DoubleList[0];
            p[1] = m_DoubleList[1];
            p[2] = m_DoubleList[2];
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            cylinderWrapStrap->SetInsertion(theBody, p);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            cylinderWrapStrap->SetCylinderBody(theBody);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderPosition"));
            Util::Double(buf, 3, m_DoubleList);
            cylinderWrapStrap->SetCylinderPosition(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderRadius"));
            cylinderWrapStrap->SetCylinderRadius(Util::Double(buf));
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderQuaternion"));
            Util::Double(buf, 4, m_DoubleList);
            cylinderWrapStrap->SetCylinderQuaternion(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
            cylinderWrapStrap->SetOrigin(theBody, (const char *)buf);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
            cylinderWrapStrap->SetInsertion(theBody, (const char *)buf);

            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderBodyID"));
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            cylinderWrapStrap->SetCylinderBody(theBody);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderPosition"));
            cylinderWrapStrap->SetCylinderPosition((const char *)buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderRadius"));
            cylinderWrapStrap->SetCylinderRadius(Util::Double(buf));
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderQuaternion"));
            cylinderWrapStrap->SetCylinderQuaternion((const char *)buf);
        }

#ifdef USE_OPENGL
        cylinderWrapStrap->SetCylinderColour(m_Interface.StrapCylinderColour);
        cylinderWrapStrap->SetCylinderLength(m_Interface.StrapCylinderLength);
#endif
    }
    else
    {
        std::cerr << "Unrecognised Strap Type:" << buf << "\n";
        return;
    }

#ifdef USE_OPENGL
    strap->SetColour(m_Interface.StrapColour);
    strap->SetRadius(m_Interface.StrapRadius);
    strap->SetForceColour(m_Interface.StrapForceColour);
    strap->SetForceRadius(m_Interface.StrapForceRadius);
    strap->SetForceScale(m_Interface.StrapForceScale);
#endif

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));
    if (strcmp((const char *)buf, "MinettiAlexander") == 0)
    {
        muscle = new MAMuscle(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ForcePerUnitArea"));
        dReal forcePerUnitArea = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VMaxFactor"));
        dReal vMaxFactor = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PCA"));
        dReal pca = Util::Double(buf);
        ((MAMuscle *)muscle)->SetF0(pca * forcePerUnitArea);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FibreLength"));
        dReal fibreLength = Util::Double(buf);
        ((MAMuscle *)muscle)->SetVMax(fibreLength * vMaxFactor);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationK"));
        ((MAMuscle *)muscle)->SetK(Util::Double(buf));
    }
    else if (strcmp((const char *)buf, "MinettiAlexanderExtended") == 0)
    {
        muscle = new MAMuscleExtended(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ForcePerUnitArea"));
        dReal forcePerUnitArea = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VMaxFactor"));
        dReal vMaxFactor = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PCA"));
        dReal pca = Util::Double(buf);
        dReal f0 = pca * forcePerUnitArea;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FibreLength"));
        dReal fibreLength = Util::Double(buf);
        dReal vMax = fibreLength * vMaxFactor;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationK"));
        dReal activationK = Util::Double(buf);
        ((MAMuscleExtended *)muscle)->SetMuscleProperties(vMax, f0, activationK);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TendonLength"));
        dReal tendonLength = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainAtFmax"));
        dReal serialStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainAtFmax"));
        dReal parallelStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationKinetics"));
        bool activationKinetics = Util::Bool(buf);
        dReal serialElasticConstant = f0 / (serialStrainAtFmax * tendonLength);
        dReal parallelElasticConstant = f0 / (parallelStrainAtFmax * fibreLength);
        ((MAMuscleExtended *)muscle)->SetParallelElasticProperties(parallelElasticConstant, fibreLength);
        ((MAMuscleExtended *)muscle)->SetSerialElasticProperties(serialElasticConstant, tendonLength);
        ((MAMuscleExtended *)muscle)->SetActivationKinetics(activationKinetics);
    }
    else if (strcmp((const char *)buf, "MinettiAlexanderExtendedDamped") == 0)
    {
        muscle = new MAMuscleExtendedDamped(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ForcePerUnitArea"));
        dReal forcePerUnitArea = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VMaxFactor"));
        dReal vMaxFactor = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PCA"));
        dReal pca = Util::Double(buf);
        dReal f0 = pca * forcePerUnitArea;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FibreLength"));
        dReal fibreLength = Util::Double(buf);
        dReal vMax = fibreLength * vMaxFactor;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationK"));
        dReal activationK = Util::Double(buf);
        ((MAMuscleExtendedDamped *)muscle)->SetMuscleProperties(vMax, f0, activationK);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TendonLength"));
        dReal tendonLength = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainAtFmax"));
        dReal serialStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainAtFmax"));
        dReal parallelStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationKinetics"));
        bool activationKinetics = Util::Bool(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialDampingOverFmax"));
        dReal serialDampingOverFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelDampingOverFmax"));
        dReal parallelDampingOverFmax = Util::Double(buf);
        dReal serialElasticConstant = f0 / (serialStrainAtFmax * tendonLength);
        dReal parallelElasticConstant = f0 / (parallelStrainAtFmax * fibreLength);
        dReal serialDampingConstant = f0 * serialDampingOverFmax / tendonLength;
        dReal parallelDampingConstant = f0 * parallelDampingOverFmax / fibreLength;
        ((MAMuscleExtendedDamped *)muscle)->SetParallelElasticProperties(parallelElasticConstant, parallelDampingConstant, fibreLength);
        ((MAMuscleExtendedDamped *)muscle)->SetSerialElasticProperties(serialElasticConstant, serialDampingConstant, tendonLength);
        ((MAMuscleExtendedDamped *)muscle)->SetActivationKinetics(activationKinetics);
    }
    else if (strcmp((const char *)buf, "MinettiAlexanderComplete") == 0)
    {
#ifdef USE_GSL
        muscle = new MAMuscleComplete(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ForcePerUnitArea"));
        dReal forcePerUnitArea = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VMaxFactor"));
        dReal vMaxFactor = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PCA"));
        dReal pca = Util::Double(buf);
        dReal f0 = pca * forcePerUnitArea;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FibreLength"));
        dReal fibreLength = Util::Double(buf);
        dReal vMax = fibreLength * vMaxFactor;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationK"));
        dReal activationK = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Width"));
        dReal width = Util::Double(buf);
        ((MAMuscleComplete *)muscle)->SetMuscleProperties(vMax, f0, activationK, width);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TendonLength"));
        dReal tendonLength = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainAtFmax"));
        dReal serialStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainAtFmax"));
        dReal parallelStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationKinetics"));
        bool activationKinetics = Util::Bool(buf);
        dReal parallelElasticConstant = f0/(SQUARE(parallelStrainAtFmax*fibreLength));
        dReal serialElasticConstant = f0/(SQUARE(serialStrainAtFmax*tendonLength));
        ((MAMuscleComplete *)muscle)->SetSerialElasticProperties(serialElasticConstant, tendonLength);
        ((MAMuscleComplete *)muscle)->SetParallelElasticProperties(parallelElasticConstant, fibreLength);
        ((MAMuscleComplete *)muscle)->SetActivationKinetics(activationKinetics);
#else
        std::cerr << "Compiled without GSL so MinettiAlexanderComplete not available\n";
#endif // USE_GSL
    }
    else if (strcmp((const char *)buf, "DampedSpring") == 0)
    {
        muscle = new DampedSpringMuscle(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"UnloadedLength"));
        ((DampedSpringMuscle *)muscle)->SetUnloadedLength(Util::Double(buf));
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SpringConstant"));
        ((DampedSpringMuscle *)muscle)->SetSpringConstant(Util::Double(buf));
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Area"));
        ((DampedSpringMuscle *)muscle)->SetArea(Util::Double(buf));
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Damping"));
        ((DampedSpringMuscle *)muscle)->SetDamping(Util::Double(buf));

    }
    else if (strcmp((const char *)buf, "UmbergerGerritsenMartin") == 0)
    {
        muscle = new UGMMuscle(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PCA"));
        dReal PCSA = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FibreLength"));
        dReal optimumLength = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TendonLength"));
        dReal tendonLength = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainModel"));
        UGMMuscle::StrainModel serialStrainModel;
        if (strcmp((const char *)buf, "Linear") == 0)
            serialStrainModel = UGMMuscle::linear;
        else if (strcmp((const char *)buf, "Square") == 0)
            serialStrainModel = UGMMuscle::square;
        else throw __LINE__;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainAtFmax"));
        dReal serialStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainModel"));
        UGMMuscle::StrainModel parallelStrainModel;
        if (strcmp((const char *)buf, "Linear") == 0)
            parallelStrainModel = UGMMuscle::linear;
        else if (strcmp((const char *)buf, "Square") == 0)
            parallelStrainModel = UGMMuscle::square;
        else throw __LINE__;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainAtFmax"));
        dReal parallelStrainAtFmax = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ForcePerUnitArea"));
        dReal forcePerUnitArea = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VMaxFactor"));
        dReal vMaxFactor = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MuscleDensity"));
        dReal muscleDensity = Util::Double(buf);
        ((UGMMuscle *)muscle)->SetModellingConstants(forcePerUnitArea, vMaxFactor, muscleDensity);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FastTwitchProportion"));
        ((UGMMuscle *)muscle)->SetFibreComposition(Util::Double(buf));

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Width"));
        dReal muscleWidth = Util::Double(buf);
        ((UGMMuscle *)muscle)->SetMuscleGeometry(PCSA, optimumLength, muscleWidth, tendonLength,
                                                 serialStrainModel, serialStrainAtFmax,
                                                 parallelStrainModel, parallelStrainAtFmax);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Aerobic"));
        ((UGMMuscle *)muscle)->SetAerobic(Util::Bool(buf));

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"AllowReverseWork"));
        ((UGMMuscle *)muscle)->AllowReverseWork(Util::Bool(buf));

    }
    else
    {
        std::cerr << "Unrecognised Muscle Type:" << buf << "\n";
        return;
    }

    m_MuscleList[*muscle->GetName()] = muscle;

}

void Simulation::ParseDriver(xmlNodePtr cur)
{
    char *buf;
    int count;

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));

    if (strcmp((const char *)buf, "Cyclic") == 0)
    {
        CyclicDriver *cyclicDriver = new CyclicDriver();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        m_DriverList[(const char *)buf] = cyclicDriver;
        cyclicDriver->SetName((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"DurationValuePairs"));
        count = DataFile::CountTokens((char *)buf);
        Util::Double(buf, count, m_DoubleList);
        cyclicDriver->SetValueDurationPairs(count, m_DoubleList);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Target"));
        cyclicDriver->SetTarget(m_MuscleList[(const char *)buf]);

        // check presence of PhaseDelay
        buf = DoXmlGetProp(cur, (const xmlChar *)"PhaseDelay");
        if (buf)
        {
            cyclicDriver->SetPhaseDelay(Util::Double(buf));
        }

        // assumes all cycles are the same duration
        m_CycleTime = cyclicDriver->GetCycleTime();
    }
    else if (strcmp((const char *)buf, "Step") == 0)
    {
        StepDriver *stepDriver = new StepDriver();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        m_DriverList[(const char *)buf] = stepDriver;
        stepDriver->SetName((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"DurationValuePairs"));
        count = DataFile::CountTokens((char *)buf);
        Util::Double(buf, count, m_DoubleList);
        stepDriver->SetValueDurationPairs(count, m_DoubleList);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Target"));
        stepDriver->SetTarget(m_MuscleList[(const char *)buf]);
    }
    else
    {
        std::cerr << "Unrecognised Driver Type:" << buf << "\n";
        return;
    }
}


void Simulation::ParseDataTarget(xmlNodePtr cur)
{
    char *buf;
    int count, i;
    DataTarget *dataTarget;
    buf = DoXmlGetProp(cur, (const xmlChar *)"Type");
    if (buf == 0) // default to "Scalar"
    {
        strcpy(m_Buffer2, "Scalar");
        buf = m_Buffer2;
    }

    if (strcmp((const char *)buf, "Scalar") == 0)
    {
        DataTargetScalar *dataTargetScalar = new DataTargetScalar();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        dataTargetScalar->SetName((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"DataType"));
        if (strcmp((const char *)buf, "XP") == 0) dataTargetScalar->SetDataType(DataTargetScalar::XP);
        else if (strcmp((const char *)buf, "YP") == 0) dataTargetScalar->SetDataType(DataTargetScalar::YP);
        else if (strcmp((const char *)buf, "ZP") == 0) dataTargetScalar->SetDataType(DataTargetScalar::ZP);
        else if (strcmp((const char *)buf, "Q0") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Q0);
        else if (strcmp((const char *)buf, "Q1") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Q1);
        else if (strcmp((const char *)buf, "Q2") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Q2);
        else if (strcmp((const char *)buf, "Q3") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Q3);
        else if (strcmp((const char *)buf, "XV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::XV);
        else if (strcmp((const char *)buf, "YV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::YV);
        else if (strcmp((const char *)buf, "ZV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::ZV);
        else if (strcmp((const char *)buf, "XRV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::XRV);
        else if (strcmp((const char *)buf, "YRV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::YRV);
        else if (strcmp((const char *)buf, "ZRV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::ZRV);
        else if (strcmp((const char *)buf, "Angle") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Angle);
        else if (strcmp((const char *)buf, "MetabolicEnergy") == 0) dataTargetScalar->SetDataType(DataTargetScalar::MetabolicEnergy);
        else if (strcmp((const char *)buf, "MechanicalEnergy") == 0) dataTargetScalar->SetDataType(DataTargetScalar::MechanicalEnergy);
        else throw(__LINE__);

        if (dataTargetScalar->GetDataType() != DataTargetScalar::MetabolicEnergy && dataTargetScalar->GetDataType() != DataTargetScalar::MechanicalEnergy)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Target"));
            std::map<std::string, Body *>::const_iterator iterBody = m_BodyList.find(buf);
            if (iterBody != m_BodyList.end()) dataTargetScalar->SetTarget(iterBody->second);
            std::map<std::string, Joint *>::const_iterator iterJoint = m_JointList.find(buf);
            if (iterJoint != m_JointList.end()) dataTargetScalar->SetTarget(iterJoint->second);
            std::map<std::string, Geom *>::const_iterator iterGeom = m_GeomList.find(buf);
            if (iterGeom != m_GeomList.end()) dataTargetScalar->SetTarget(iterGeom->second);
            THROWIFZERO(dataTargetScalar->GetTarget());
        }

        // check presence of Weight
        buf = DoXmlGetProp(cur, (const xmlChar *)"Weight");
        if (buf)
        {
            dataTargetScalar->SetWeight(Util::Double(buf));
        }
        // check presence of Slope
        buf = DoXmlGetProp(cur, (const xmlChar *)"Slope");
        if (buf)
        {
            dataTargetScalar->SetSlope(Util::Double(buf));
        }
        // check presence of MatchType
        buf = DoXmlGetProp(cur, (const xmlChar *)"MatchType");
        if (buf)
        {
            if (strcmp((const char *)buf, "Linear") == 0) dataTargetScalar->SetMatchType(DataTarget::linear);
            else if (strcmp((const char *)buf, "Square") == 0) dataTargetScalar->SetMatchType(DataTarget::square);
            else throw(__LINE__);
        }
        // check presence of AbortThreshold
        buf = DoXmlGetProp(cur, (const xmlChar *)"AbortThreshold");
        if (buf)
        {
            dataTargetScalar->SetAbortThreshold(Util::Double(buf));
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"DurationValuePairs");
        if (buf)
        {
            count = DataFile::ReturnTokens(buf, m_BufferPtrs, m_BufferSize);

            Util::Double(buf, count, m_DoubleList);
            count = count / 2;
            dReal *times = new dReal[count];
            dReal *values = new dReal[count];
            for (i = 0; i < count; i++)
            {
                times[i] = Util::Double(m_BufferPtrs[i * 2]);
                if (dataTargetScalar->GetDataType() == DataTargetScalar::Angle)
                    values[i] = Util::GetAngle(m_BufferPtrs[i * 2 + 1]);
                else
                    values[i] = Util::Double(m_BufferPtrs[i * 2 + 1]);
            }
            dataTargetScalar->SetTargetTimes(count, times);
            dataTargetScalar->SetTargetValues(count, values);
            delete [] times;
            delete [] values;
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetTimes"));
            count = DataFile::CountTokens((char *)buf);
            Util::Double(buf, count, m_DoubleList);
            dataTargetScalar->SetTargetTimes(count, m_DoubleList);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetValues"));
            count = DataFile::CountTokens((char *)buf);
            Util::Double(buf, count, m_DoubleList);
            dataTargetScalar->SetTargetValues(count, m_DoubleList);
        }
        dataTarget = dataTargetScalar;
    }
    else if (strcmp((const char *)buf, "Quaternion") == 0)
    {
        DataTargetQuaternion *dataTargetQuaternion = new DataTargetQuaternion();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        dataTargetQuaternion->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Target"));
        std::map<std::string, Body *>::const_iterator iterBody = m_BodyList.find(buf);
        if (iterBody != m_BodyList.end()) dataTargetQuaternion->SetTarget(iterBody->second);
        std::map<std::string, Geom *>::const_iterator iterGeom = m_GeomList.find(buf);
        if (iterGeom != m_GeomList.end()) dataTargetQuaternion->SetTarget(iterGeom->second);
        THROWIFZERO(dataTargetQuaternion->GetTarget());

        // check presence of Weight
        buf = DoXmlGetProp(cur, (const xmlChar *)"Weight");
        if (buf)
        {
            dataTargetQuaternion->SetWeight(Util::Double(buf));
        }
        // check presence of Slope
        buf = DoXmlGetProp(cur, (const xmlChar *)"Slope");
        if (buf)
        {
            dataTargetQuaternion->SetSlope(Util::Double(buf));
        }
        // check presence of MatchType
        buf = DoXmlGetProp(cur, (const xmlChar *)"MatchType");
        if (buf)
        {
            if (strcmp((const char *)buf, "Linear") == 0) dataTargetQuaternion->SetMatchType(DataTarget::linear);
            else if (strcmp((const char *)buf, "Square") == 0) dataTargetQuaternion->SetMatchType(DataTarget::square);
            else throw(__LINE__);
        }
        // check presence of AbortThreshold
        buf = DoXmlGetProp(cur, (const xmlChar *)"AbortThreshold");
        if (buf)
        {
            dataTargetQuaternion->SetAbortThreshold(Util::Double(buf));
        }

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetTimes"));
        int timeCount = DataFile::CountTokens((char *)buf);
        Util::Double(buf, timeCount, m_DoubleList);
        dataTargetQuaternion->SetTargetTimes(timeCount, m_DoubleList);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetValues"));
        dataTargetQuaternion->SetTargetValues((char *)buf);

        dataTarget = dataTargetQuaternion;
    }
    else if (strcmp((const char *)buf, "Vector") == 0)
    {
        DataTargetVector *dataTargetVector = new DataTargetVector();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        dataTargetVector->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Target"));
        std::map<std::string, Body *>::const_iterator iterBody = m_BodyList.find(buf);
        if (iterBody != m_BodyList.end()) dataTargetVector->SetTarget(iterBody->second);
        std::map<std::string, Joint *>::const_iterator iterJoint = m_JointList.find(buf);
        if (iterJoint != m_JointList.end()) dataTargetVector->SetTarget(iterJoint->second);
        std::map<std::string, Geom *>::const_iterator iterGeom = m_GeomList.find(buf);
        if (iterGeom != m_GeomList.end()) dataTargetVector->SetTarget(iterGeom->second);
        THROWIFZERO(dataTargetVector->GetTarget());

        // check presence of Weight
        buf = DoXmlGetProp(cur, (const xmlChar *)"Weight");
        if (buf)
        {
            dataTargetVector->SetWeight(Util::Double(buf));
        }
        // check presence of Slope
        buf = DoXmlGetProp(cur, (const xmlChar *)"Slope");
        if (buf)
        {
            dataTargetVector->SetSlope(Util::Double(buf));
        }
        // check presence of MatchType
        buf = DoXmlGetProp(cur, (const xmlChar *)"MatchType");
        if (buf)
        {
            if (strcmp((const char *)buf, "Linear") == 0) dataTargetVector->SetMatchType(DataTarget::linear);
            else if (strcmp((const char *)buf, "Square") == 0) dataTargetVector->SetMatchType(DataTarget::square);
            else throw(__LINE__);
        }
        // check presence of AbortThreshold
        buf = DoXmlGetProp(cur, (const xmlChar *)"AbortThreshold");
        if (buf)
        {
            dataTargetVector->SetAbortThreshold(Util::Double(buf));
        }

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetTimes"));
        int timeCount = DataFile::CountTokens((char *)buf);
        Util::Double(buf, timeCount, m_DoubleList);
        dataTargetVector->SetTargetTimes(timeCount, m_DoubleList);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetValues"));
        dataTargetVector->SetTargetValues((char *)buf);

        dataTarget = dataTargetVector;
    }

    m_DataTargetList[*dataTarget->GetName()] = dataTarget;

}

void Simulation::ParseIOControl(xmlNodePtr cur)
{
    char *buf;
    buf = DoXmlGetProp(cur, (const xmlChar *)"OldStyleInputs");
    if (buf)
    {
        SetOldStyleInputs(Util::Bool(buf));
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"OldStyleOutputs");
    if (buf)
    {
        SetOldStyleOutputs(Util::Bool(buf));
    }
}

void Simulation::ParseMarker(xmlNodePtr cur)
{
    char *buf;
    Marker *marker = new Marker();

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
    marker->SetName((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BodyID"));
    if (strcmp(buf, "World"))
    {
        dBodyID bodyID = m_BodyList[(const char *)buf]->GetBodyID();
        marker->SetBody(bodyID);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"Position");
    if (buf)
        marker->SetPosition((const char *)buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"Quaternion");
    if (buf)
        marker->SetQuaternion((const char *)buf);

#ifdef USE_OPENGL
    marker->SetColour(m_Interface.MarkerColour);
    marker->SetRadius(m_Interface.MarkerRadius);
#endif

    m_MarkerList[*marker->GetName()] = marker;
}

#ifdef USE_OPENGL
void Simulation::ParseInterface(xmlNodePtr cur)
{
    char  *buf;

    buf = DoXmlGetProp(cur, (const xmlChar *)"TrackBodyID");
    if (buf)
    {
        m_Interface.TrackBodyID = (char *)buf;
    }


    buf = DoXmlGetProp(cur, (const xmlChar *)"EnvironmentAxisSize");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        m_Interface.EnvironmentAxisSize[0] = m_DoubleList[0];
        m_Interface.EnvironmentAxisSize[1] = m_DoubleList[1];
        m_Interface.EnvironmentAxisSize[2] = m_DoubleList[2];
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"EnvironmentColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.EnvironmentColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"BodyAxisSize");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        m_Interface.BodyAxisSize[0] = m_DoubleList[0];
        m_Interface.BodyAxisSize[1] = m_DoubleList[1];
        m_Interface.BodyAxisSize[2] = m_DoubleList[2];
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"BodyColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.BodyColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"JointAxisSize");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        m_Interface.JointAxisSize[0] = m_DoubleList[0];
        m_Interface.JointAxisSize[1] = m_DoubleList[1];
        m_Interface.JointAxisSize[2] = m_DoubleList[2];
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"JointColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.JointColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"GeomColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.GeomColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"StrapColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.StrapColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"StrapRadius");
    if (buf)
    {
        m_Interface.StrapRadius = Util::Double(buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"StrapForceColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.StrapForceColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"StrapForceRadius");
    if (buf)
    {
        m_Interface.StrapForceRadius = Util::Double(buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"StrapForceScale");
    if (buf)
    {
        m_Interface.StrapForceScale = Util::Double(buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"StrapCylinderColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.StrapCylinderColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"StrapCylinderLength");
    if (buf)
    {
        m_Interface.StrapCylinderLength = Util::Double(buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactAxisSize");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        m_Interface.ContactAxisSize[0] = m_DoubleList[0];
        m_Interface.ContactAxisSize[1] = m_DoubleList[1];
        m_Interface.ContactAxisSize[2] = m_DoubleList[2];
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.ContactColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactForceColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.ContactForceColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactForceRadius");
    if (buf)
    {
        m_Interface.ContactForceRadius = Util::Double(buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactForceScale");
    if (buf)
    {
        m_Interface.ContactForceScale = Util::Double(buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"MarkerColour");
    if (buf)
    {
        Util::Double(buf, 4, m_DoubleList);
        m_Interface.MarkerColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"MarkerRadius");
    if (buf)
    {
        m_Interface.MarkerRadius = Util::Double(buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"DrawingOrder");
    if (buf)
    {
        strcpy(m_Buffer, (char *)buf);
        int count = DataFile::ReturnTokens(m_Buffer, m_BufferPtrs, m_BufferSize);

        m_Interface.DrawingOrder.clear();
        for (int i = 0; i < count; i++)
        {
            if (strcmp(m_BufferPtrs[i], "Environment") == 0) m_Interface.DrawingOrder.push_back(DOCEnvironment);
            else if (strcmp(m_BufferPtrs[i], "Joint") == 0) m_Interface.DrawingOrder.push_back(DOCJoint);
            else if (strcmp(m_BufferPtrs[i], "Muscle") == 0) m_Interface.DrawingOrder.push_back(DOCMuscle);
            else if (strcmp(m_BufferPtrs[i], "Geom") == 0) m_Interface.DrawingOrder.push_back(DOCGeom);
            else if (strcmp(m_BufferPtrs[i], "Body") == 0) m_Interface.DrawingOrder.push_back(DOCBody);
            else throw(__LINE__);
        }
    }
    else
    {
        m_Interface.DrawingOrder.push_back(DOCEnvironment);
        m_Interface.DrawingOrder.push_back(DOCBody);
        m_Interface.DrawingOrder.push_back(DOCJoint);
        m_Interface.DrawingOrder.push_back(DOCMuscle);
        m_Interface.DrawingOrder.push_back(DOCGeom);
    }


}
#endif

// function to produce a file of link kinematics in tab delimited format

void
        Simulation::OutputKinematics()
{
    static bool firstTimeFlag = true;
    const dReal *p;
    std::map<std::string, Body *>::const_iterator iter1;

    // first time through output the column headings
    if (firstTimeFlag)
    {
        m_OutputKinematicsFile.open(m_OutputKinematicsFilename.c_str());

        m_OutputKinematicsFile << "time\t";
        for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
        {
            m_OutputKinematicsFile << *iter1->second->GetName() << "_X\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Y\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Z\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Q0\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Q1\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Q2\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Q3\t";
        }
        firstTimeFlag = false;
        m_OutputKinematicsFile << "\n";
    }

    // start by outputting the simulation time
    m_OutputKinematicsFile << m_SimulationTime << "\t";

    for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
    {
        p = iter1->second->GetPosition();
        m_OutputKinematicsFile << p[0] << "\t";
        m_OutputKinematicsFile << p[1] << "\t";
        m_OutputKinematicsFile << p[2] << "\t";

        p = iter1->second->GetQuaternion();
        m_OutputKinematicsFile << p[0] << "\t";
        m_OutputKinematicsFile << p[1] << "\t";
        m_OutputKinematicsFile << p[2] << "\t";
        m_OutputKinematicsFile << p[3] << "\t";
    }
    m_OutputKinematicsFile << "\n";
}

// function to input a file of link kinematics in tab delimited format
// requires the data in the order produced by OutputKinematics

void
        Simulation::InputKinematics()
{
    std::map<std::string, Body *>::const_iterator iter1;
    double v;
    dQuaternion q;
    dReal x, y, z;

    m_InputKinematicsFile.SetExitOnError(true);
    if (m_InputKinematicsFile.ReadNext(&v))
    {
        std::cerr << "End of kinematics file\n";
        return;
    }
    m_InputKinematicsFile.SetExitOnError(true);
    m_SimulationTime = v;
    m_OutputKinematicsFile << m_SimulationTime << "\t";

    for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
    {
        m_InputKinematicsFile.ReadNext(&v); x = v;
        m_InputKinematicsFile.ReadNext(&v); y = v;
        m_InputKinematicsFile.ReadNext(&v); z = v;
        iter1->second->SetPosition(x, y, z);

        m_InputKinematicsFile.ReadNext(&v); q[0] = v;
        m_InputKinematicsFile.ReadNext(&v); q[1] = v;
        m_InputKinematicsFile.ReadNext(&v); q[2] = v;
        m_InputKinematicsFile.ReadNext(&v); q[3] = v;
        iter1->second->SetQuaternion(q[0], q[1], q[2], q[3]);
    }
}

// function to produce a file of link kinematics in tab delimited format
// plus additional muscle activation information for use in gait warehousing

void
        Simulation::OutputWarehouse()
{
    static dReal lastTime = -9999;
    const dReal minTimeInc = 0.01;
    const dReal fpError = 1e-10;

    // first time through output the column headings
    if (lastTime < 0)
    {
        m_OutputWarehouseFile.open(m_OutputWarehouseFilename.c_str());
    }

    if (m_SimulationTime - lastTime >= minTimeInc - fpError)
    {
        lastTime = m_SimulationTime;

        // start by outputting the simulation time
        m_OutputWarehouseFile << "Time\t" << m_SimulationTime << "\n";

        m_OutputWarehouseFile << "NumDrivers\t" << m_DriverList.size() << "\n";
        std::map<std::string, Driver *>::const_iterator iter6;
        for (iter6 = m_DriverList.begin(); iter6 != m_DriverList.end(); iter6++)
        {
            m_OutputWarehouseFile << *iter6->second->GetName() << "\t" << iter6->second->GetValue(m_SimulationTime) << "\n";
        }

        m_OutputWarehouseFile << "NumBodies\t" << m_BodyList.size() << "\n";
        std::map<std::string, Body *>::const_iterator iter5;
        for (iter5 = m_BodyList.begin(); iter5 != m_BodyList.end(); iter5++)
        {
            m_OutputWarehouseFile << *iter5->second->GetName() << "\n";
            const dReal *p = iter5->second->GetPosition();
            const dReal *r = iter5->second->GetRotation();
            const dReal *v = iter5->second->GetLinearVelocity();
            const dReal *rv = iter5->second->GetAngularVelocity();
            m_OutputWarehouseFile << p[0] << "\t" << p[1] << "\t" << p[2] << "\n";
            m_OutputWarehouseFile << r[0] << "\t" << r[1] << "\t" << r[2] << "\n";
            m_OutputWarehouseFile << r[4] << "\t" << r[5] << "\t" << r[6] << "\n";
            m_OutputWarehouseFile << r[8] << "\t" << r[9] << "\t" << r[10] << "\n";
            m_OutputWarehouseFile << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
            m_OutputWarehouseFile << rv[0] << "\t" << rv[1] << "\t" << rv[2] << "\n";
        }
        m_OutputWarehouseFile << "\n";
    }
}


// output the simulation state in an XML format that can be re-read

void Simulation::OutputProgramState()
{
    xmlDocPtr doc;
    xmlNodePtr rootNode;
    xmlNodePtr newNode;
    xmlAttrPtr newAttr;
    dVector3 v;
    dVector3 result;

    doc = xmlNewDoc((xmlChar *)"1.0");
    if (doc == 0) return;

    rootNode = xmlNewDocNode(doc, 0, (xmlChar *)"GAITSYMODE", 0);
    xmlDocSetRootElement(doc, rootNode);

    sprintf(m_Buffer, "%.17g", m_SimulationTime);
    newNode = xmlNewTextChild(rootNode, 0, (xmlChar *)"STATE", 0);
    newAttr = xmlNewProp(newNode, (xmlChar *)"SimulationTime", (xmlChar *)m_Buffer);
    if (m_OutputModelStateAtCycle >= 0)
    {
        sprintf(m_Buffer, "%.17g", m_OutputModelStateAtCycle);
        newAttr = xmlNewProp(newNode, (xmlChar *)"CycleFraction", (xmlChar *)m_Buffer);
    }

    const dReal *munge = m_DistanceTravelledBodyID->GetPosition();
    const dReal *mungeR = m_DistanceTravelledBodyID->GetRotation(); // calculate Z rotation to straighten up X
    pgd::Quaternion mungeRotation(1, 0, 0, 0); // identity quaternion
    if (m_MungeRotationFlag)
    {
        dReal thetaX, thetaY, thetaZ;
        Util::EulerDecompositionZYX(mungeR, thetaX, thetaY, thetaZ);
        mungeRotation = pgd::MakeQFromAxis(0, 0, 1, -thetaZ);
    }

    if (m_OldStyleOutputs)
    {
        newNode = xmlNewTextChild(rootNode, 0, (xmlChar *)"IOCONTROL", 0);
        newAttr = xmlNewProp(newNode, (xmlChar *)"OldStyleInputs", (xmlChar *)"true");
        newAttr = xmlNewProp(newNode, (xmlChar *)"OldStyleOutputs", (xmlChar *)"true");

        std::map<std::string, Body *>::const_iterator iter1;
        for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
        {
            newNode = xmlNewTextChild(rootNode, 0, (xmlChar *)"BODY", 0);
            newAttr = xmlNewProp(newNode, (xmlChar *)"ID", (xmlChar *)iter1->second->GetName()->c_str());
            const dReal *p = iter1->second->GetPosition();
            if (m_MungeModelStateFlag)
            {
                pgd::Vector v1(p[0] - munge[0], p[1] - munge[1], p[2]);
                pgd::Vector v2 = pgd::QVRotate(mungeRotation, v1);
                sprintf(m_Buffer, "%.17g %.17g %.17g", v2.x, v2.y, v2.z);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                p = iter1->second->GetQuaternion();
                pgd::Quaternion qBody1(p[0], p[1], p[2], p[3]);
                pgd::Quaternion qBody2 = mungeRotation * qBody1;
                sprintf(m_Buffer, "%.17g %.17g %.17g %.17g", qBody2.n, qBody2.v.x, qBody2.v.y, qBody2.v.z);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                p = iter1->second->GetLinearVelocity();
                v1.x = p[0]; v1.y = p[1]; v1.z = p[2];
                v2 = pgd::QVRotate(mungeRotation, v1);
                sprintf(m_Buffer, "%.17g %.17g %.17g", v2.x, v2.y, v2.z);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"LinearVelocity", (xmlChar *)m_Buffer);
                p = iter1->second->GetAngularVelocity();
                v1.x = p[0]; v1.y = p[1]; v1.z = p[2];
                v2 = pgd::QVRotate(mungeRotation, v1);
                sprintf(m_Buffer, "%.17g %.17g %.17g", v2.x, v2.y, v2.z);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"AngularVelocity", (xmlChar *)m_Buffer);
            }
            else
            {
                sprintf(m_Buffer, "%.17g %.17g %.17g", p[0], p[1], p[2]);
                newAttr = xmlNewProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                p = iter1->second->GetQuaternion();
                sprintf(m_Buffer, "%.17g %.17g %.17g %.17g", p[0], p[1], p[2], p[3]);
                newAttr = xmlNewProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                p = iter1->second->GetLinearVelocity();
                sprintf(m_Buffer, "%.17g %.17g %.17g", p[0], p[1], p[2]);
                newAttr = xmlNewProp(newNode, (xmlChar *)"LinearVelocity", (xmlChar *)m_Buffer);
                p = iter1->second->GetAngularVelocity();
                sprintf(m_Buffer, "%.17g %.17g %.17g", p[0], p[1], p[2]);
                newAttr = xmlNewProp(newNode, (xmlChar *)"AngularVelocity", (xmlChar *)m_Buffer);
            }
        }

        std::map<std::string, Joint *>::const_iterator iter2;
        for (iter2 = m_JointList.begin(); iter2 != m_JointList.end(); iter2++)
        {
            HingeJoint *jp = dynamic_cast<HingeJoint *>(iter2->second);
            if (jp)
            {
                newNode = xmlNewTextChild(rootNode, 0, (xmlChar *)"JOINT", 0);
                newAttr = xmlNewProp(newNode, (xmlChar *)"ID", (xmlChar *)jp->GetName()->c_str());
                jp->GetHingeAnchor(v);
                dBodyGetPosRelPoint(jp->GetBody(0), v[0], v[1], v[2], result);
                sprintf(m_Buffer, "%.17g %.17g %.17g", result[0], result[1], result[2]);
                newAttr = xmlNewProp(newNode, (xmlChar *)"HingeAnchor", (xmlChar *)m_Buffer);
                jp->GetHingeAxis(v);
                dBodyVectorFromWorld(jp->GetBody(0), v[0], v[1], v[2], result);
                sprintf(m_Buffer, "%.17g %.17g %.17g", result[0], result[1], result[2]);
                newAttr = xmlNewProp(newNode, (xmlChar *)"HingeAxis", (xmlChar *)m_Buffer);
                sprintf(m_Buffer, "%.17g", jp->GetHingeAngle());
                newAttr = xmlNewProp(newNode, (xmlChar *)"StartAngleReference", (xmlChar *)m_Buffer);
            }
        }
    }
    else
    {
        char *buf;
        xmlNodePtr cur;

        newNode = xmlNewTextChild(rootNode, 0, (xmlChar *)"IOCONTROL", 0);
        newAttr = xmlNewProp(newNode, (xmlChar *)"OldStyleInputs", (xmlChar *)"false");
        newAttr = xmlNewProp(newNode, (xmlChar *)"OldStyleOutputs", (xmlChar *)"false");

        std::vector<xmlNodePtr>::const_iterator iter0;
        for (iter0 = m_TagContentsList.begin(); iter0 != m_TagContentsList.end(); iter0++)
        {
            cur = *iter0;
            if (xmlStrcmp(cur->name, (const xmlChar *)"GLOBAL") == 0 ||
                xmlStrcmp(cur->name, (const xmlChar *)"ENVIRONMENT") == 0 ||
                xmlStrcmp(cur->name, (const xmlChar *)"INTERFACE") == 0 ||
                xmlStrcmp(cur->name, (const xmlChar *)"DRIVER") == 0 ||
                xmlStrcmp(cur->name, (const xmlChar *)"DATATARGET") == 0)
            {
                newNode = xmlCopyNode(cur, 1);
                xmlAddChild(rootNode, newNode);
            }

            if ((!xmlStrcmp(cur->name, (const xmlChar *)"BODY")))
            {
                buf = DoXmlGetProp(cur, (const xmlChar *)"ID");
                Body *body = m_BodyList[(const char *)buf];
                newNode = xmlCopyNode(cur, 1);
                xmlAddChild(rootNode, newNode);

                const dReal *p;

                p = body->GetOffset();
                sprintf(m_Buffer, "%.17g %.17g %.17g", p[0], p[1], p[2]);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Offset", (xmlChar *)m_Buffer);

                dMass mass;
                dBodyGetMass(body->GetBodyID(), &mass);
                sprintf(m_Buffer, "%.17g", mass.mass);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Mass", (xmlChar *)m_Buffer);
                sprintf(m_Buffer, "%.17g %.17g %.17g %.17g %.17g %.17g", mass._I(0,0), mass._I(1,1), mass._I(2,2),
                        mass._I(0,1), mass._I(0,2), mass._I(1,2));
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"MOI", (xmlChar *)m_Buffer);

                sprintf(m_Buffer, "-1");
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Density", (xmlChar *)m_Buffer);

                if (m_MungeModelStateFlag)
                {
                    p = body->GetPosition();
                    pgd::Vector v1(p[0] - munge[0], p[1] - munge[1], p[2]);
                    pgd::Vector v2 = pgd::QVRotate(mungeRotation, v1);
                    sprintf(m_Buffer, "World %.17g %.17g %.17g", v2.x, v2.y, v2.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                    p = body->GetQuaternion();
                    pgd::Quaternion qBody1(p[0], p[1], p[2], p[3]);
                    pgd::Quaternion qBody2 = mungeRotation * qBody1;
                    sprintf(m_Buffer, "World %.17g %.17g %.17g %.17g", qBody2.n, qBody2.v.x, qBody2.v.y, qBody2.v.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                    p = body->GetLinearVelocity();
                    v1.x = p[0]; v1.y = p[1]; v1.z = p[2];
                    v2 = pgd::QVRotate(mungeRotation, v1);
                    sprintf(m_Buffer, "%.17g %.17g %.17g", v2.x, v2.y, v2.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"LinearVelocity", (xmlChar *)m_Buffer);
                    p = body->GetAngularVelocity();
                    v1.x = p[0]; v1.y = p[1]; v1.z = p[2];
                    v2 = pgd::QVRotate(mungeRotation, v1);
                    sprintf(m_Buffer, "%.17g %.17g %.17g", v2.x, v2.y, v2.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"AngularVelocity", (xmlChar *)m_Buffer);
                }
                else
                {
                    p = body->GetPosition();
                    sprintf(m_Buffer, "World %.17g %.17g %.17g", p[0], p[1], p[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                    p = body->GetQuaternion();
                    sprintf(m_Buffer, "World %.17g %.17g %.17g %.17g", p[0], p[1], p[2], p[3]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                    p = body->GetLinearVelocity();
                    sprintf(m_Buffer, "%.17g %.17g %.17g", p[0], p[1], p[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"LinearVelocity", (xmlChar *)m_Buffer);
                    p = body->GetAngularVelocity();
                    sprintf(m_Buffer, "%.17g %.17g %.17g", p[0], p[1], p[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"AngularVelocity", (xmlChar *)m_Buffer);
                }

            }

            if ((!xmlStrcmp(cur->name, (const xmlChar *)"JOINT")))
            {
                buf = DoXmlGetProp(cur, (const xmlChar *)"ID");
                Joint *joint = m_JointList[(const char *)buf];
                newNode = xmlCopyNode(cur, 1);
                xmlAddChild(rootNode, newNode);

                HingeJoint *jp = dynamic_cast<HingeJoint *>(joint);
                if (jp)
                {
                    Body *body = (Body *)(dBodyGetData(jp->GetBody(0)));
                    jp->GetHingeAnchor(v);
                    if (m_ModelStateRelative)
                    {
                        dBodyGetPosRelPoint(jp->GetBody(0), v[0], v[1], v[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(), result[0], result[1], result[2]);
                    }
                    else
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g","World" , v[0], v[1], v[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"HingeAnchor", (xmlChar *)m_Buffer);
                    jp->GetHingeAxis(v);
                    if (m_ModelStateRelative)
                    {
                        dBodyVectorFromWorld(jp->GetBody(0), v[0], v[1], v[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(), result[0], result[1], result[2]);
                    }
                    else
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World" , v[0], v[1], v[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"HingeAxis", (xmlChar *)m_Buffer);

                    // always output the extra joint data - it's very useful
                    jp->GetHingeAnchor(v);
                    dBodyGetPosRelPoint(jp->GetBody(1), v[0], v[1], v[2], result);
                    body = (Body *)(dBodyGetData(jp->GetBody(1)));
                    sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(), result[0], result[1], result[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Body2HingeAnchor", (xmlChar *)m_Buffer);
                    jp->GetHingeAxis(v);
                    dBodyVectorFromWorld(jp->GetBody(1), v[0], v[1], v[2], result);
                    sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(), result[0], result[1], result[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Body2HingeAxis", (xmlChar *)m_Buffer);

                    sprintf(m_Buffer, "%.17g", jp->GetHingeAngle());
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"StartAngleReference", (xmlChar *)m_Buffer);
                }

                BallJoint *jpb = dynamic_cast<BallJoint *>(joint);
                if (jpb)
                {
                    Body *body = (Body *)(dBodyGetData(jpb->GetBody(0)));
                    jpb->GetBallAnchor(v);
                    if (m_ModelStateRelative)
                    {
                        dBodyGetPosRelPoint(jpb->GetBody(0), v[0], v[1], v[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(), result[0], result[1], result[2]);
                    }
                    else
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World" , v[0], v[1], v[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"BallAnchor", (xmlChar *)m_Buffer);

                    // always output the extra joint data - it's very useful
                    jpb->GetBallAnchor(v);
                    dBodyGetPosRelPoint(jpb->GetBody(1), v[0], v[1], v[2], result);
                    body = (Body *)(dBodyGetData(jpb->GetBody(1)));
                    sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(), result[0], result[1], result[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Body2BallAnchor", (xmlChar *)m_Buffer);
                }

            }

            if ((!xmlStrcmp(cur->name, (const xmlChar *)"GEOM")))
            {
                buf = DoXmlGetProp(cur, (const xmlChar *)"ID");
                Geom *geom = m_GeomList[(const char *)buf];
                newNode = xmlCopyNode(cur, 1);
                xmlAddChild(rootNode, newNode);

                if (geom->GetGeomLocation() == Geom::body)
                {
                    Body *body = (Body *)(dBodyGetData(geom->GetBody()));
                    const dReal *p = geom->GetPosition();
                    if (m_ModelStateRelative)
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(),  p[0], p[1], p[2]);
                    }
                    else
                    {
                        dBodyGetRelPointPos (geom->GetBody(), p[0], p[1], p[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World", result[0], result[1], result[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                    dQuaternion q;
                    geom->GetQuaternion(q);
                    sprintf(m_Buffer, "%s %.17g %.17g %.17g %.17g", body->GetName()->c_str(), q[0], q[1], q[2], q[3]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                }
            }

            if ((!xmlStrcmp(cur->name, (const xmlChar *)"MUSCLE")))
            {
                buf = DoXmlGetProp(cur, (const xmlChar *)"ID");
                Muscle *muscle = m_MuscleList[(const char *)buf];
                newNode = xmlCopyNode(cur, 1);
                xmlAddChild(rootNode, newNode);

                MAMuscleExtended *mam = dynamic_cast<MAMuscleExtended *>(muscle);
                if (mam)
                {
                    sprintf(m_Buffer, "%.17g", mam->GetSSE());
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"TendonLength", (xmlChar *)m_Buffer);
                }

                TwoPointStrap *twoPointStrap = dynamic_cast<TwoPointStrap *>(muscle->GetStrap());
                if (twoPointStrap)
                {
                    Body *body;
                    dReal *p;
                    twoPointStrap->GetOrigin(&body, &p);
                    if (m_ModelStateRelative)
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(),  p[0], p[1], p[2]);
                    }
                    else
                    {
                        dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World", result[0], result[1], result[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Origin", (xmlChar *)m_Buffer);
                    twoPointStrap->GetInsertion(&body,&p);
                    if (m_ModelStateRelative)
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(),  p[0], p[1], p[2]);
                    }
                    else
                    {
                        dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World", result[0], result[1], result[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Insertion", (xmlChar *)m_Buffer);
                }
                ThreePointStrap *threePointStrap = dynamic_cast<ThreePointStrap *>(muscle->GetStrap());
                if (threePointStrap)
                {
                    Body *body;
                    dReal *p;
                    threePointStrap->GetMidpoint(&body, &p);
                    if (m_ModelStateRelative)
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(),  p[0], p[1], p[2]);
                    }
                    else
                    {
                        dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World", result[0], result[1], result[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Midpoint", (xmlChar *)m_Buffer);
                }
                CylinderWrapStrap *cylinderWrapStrap = dynamic_cast<CylinderWrapStrap *>(muscle->GetStrap());
                if (cylinderWrapStrap)
                {
                    Body *body;
                    dVector3 p;
                    dQuaternion q;
                    dReal radius;
                    cylinderWrapStrap->GetOrigin(&body, p);
                    if (m_ModelStateRelative)
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(),  p[0], p[1], p[2]);
                    }
                    else
                    {
                        dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World", result[0], result[1], result[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Origin", (xmlChar *)m_Buffer);
                    cylinderWrapStrap->GetInsertion(&body,p);
                    if (m_ModelStateRelative)
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(),  p[0], p[1], p[2]);
                    }
                    else
                    {
                        dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World", result[0], result[1], result[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Insertion", (xmlChar *)m_Buffer);
                    cylinderWrapStrap->GetCylinder(&body, p, &radius, q);
                    if (m_ModelStateRelative)
                    {
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", body->GetName()->c_str(),  p[0], p[1], p[2]);
                    }
                    else
                    {
                        dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                        sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World", result[0], result[1], result[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"CylinderPosition", (xmlChar *)m_Buffer);
                    sprintf(m_Buffer, "%.17g", radius);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"CylinderRadius", (xmlChar *)m_Buffer);
                    sprintf(m_Buffer, "%s %.17g %.17g %.17g %.17g", body->GetName()->c_str(),  q[0], q[1], q[2], q[3]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"CylinderQuaternion", (xmlChar *)m_Buffer);
                }
                NPointStrap *nPointStrap = dynamic_cast<NPointStrap *>(muscle->GetStrap());
                if (nPointStrap)
                {
                    std::vector<dReal *> *viaPoints = nPointStrap->GetViaPoints();
                    std::vector<Body *> *viaPointBodies = nPointStrap->GetViaPointBodies();
                    char viaPointName[256];
                    dReal *p;
                    for (unsigned int i = 0; i < viaPoints->size(); i++)
                    {
                        p = (*viaPoints)[i];
                        sprintf(viaPointName, "ViaPoint%d", i);
                        if (m_ModelStateRelative)
                        {
                            sprintf(m_Buffer, "%s %.17g %.17g %.17g", (*viaPointBodies)[i]->GetName()->c_str(),  p[0], p[1], p[2]);
                        }
                        else
                        {
                            dBodyGetRelPointPos ((*viaPointBodies)[i]->GetBodyID(), p[0], p[1], p[2], result);
                            sprintf(m_Buffer, "%s %.17g %.17g %.17g", "World", result[0], result[1], result[2]);
                        }
                        newAttr = DoXmlReplaceProp(newNode, (xmlChar *)viaPointName, (xmlChar *)m_Buffer);
                    }
                }
            }
        }
    }

    xmlThrDefIndentTreeOutput(1);
    xmlSaveFormatFile(m_OutputModelStateFilename.c_str(), doc, 1);

    xmlFreeDoc(doc);
}

void Simulation::SetOutputKinematicsFile(const char *filename)
{
    if (filename)
    {
        m_OutputKinematicsFilename = filename;
        m_OutputKinematicsFlag = true;
    }
    else
    {
        m_OutputKinematicsFlag = false;
    }
}

void Simulation::SetInputKinematicsFile(const char *filename)
{
    if (filename)
    {
        m_InputKinematicsFile.SetExitOnError(true);
        m_InputKinematicsFile.ReadFile(filename);
        // and skip first line
        m_InputKinematicsFile.ReadNextLine(m_Buffer, m_BufferSize, false);
        m_InputKinematicsFlag = true;
    }
    else
    {
        m_InputKinematicsFlag = false;
    }
}

void Simulation::SetOutputModelStateFile(const char *filename)
{
    if (filename)
    {
        m_OutputModelStateFilename = filename;
    }
}

void Simulation::SetOutputWarehouseFile(const char *filename)
{
    if (filename)
    {
        m_OutputWarehouseFilename = filename;
        m_OutputWarehouseFlag = true;
    }
}

bool Simulation::ShouldQuit()
{
    if (m_TimeLimit > 0)
        if (m_SimulationTime > m_TimeLimit) return true;
    if (m_MechanicalEnergyLimit > 0)
        if (m_MechanicalEnergy > m_MechanicalEnergyLimit) return true;
    if (m_MetabolicEnergyLimit > 0)
        if (m_MetabolicEnergy > m_MetabolicEnergyLimit) return true;
    return false;
}

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

void Simulation::NearCallback(void *data, dGeomID o1, dGeomID o2)
{
    int i;
    int numc;
    Simulation *s = (Simulation *)data;

    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    Contact *myContact;
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

    if (s->m_AllowInternalCollisions == false)
    {
        if (((Geom *)dGeomGetData(o1))->GetGeomLocation() == ((Geom *)dGeomGetData(o2))->GetGeomLocation()) return;
    }

    dContact *contact = new dContact[s->m_MaxContacts];   // up to m_MaxContacts contacts per box-box
    dReal cfm = MAX(((Geom *)dGeomGetData(o1))->GetContactSoftCFM(),
                    ((Geom *)dGeomGetData(o2))->GetContactSoftCFM());
    dReal erp = MIN(((Geom *)dGeomGetData(o1))->GetContactSoftERP(),
                    ((Geom *)dGeomGetData(o2))->GetContactSoftERP());
    dReal mu = MIN(((Geom *)dGeomGetData(o1))->GetContactMu(),
                   ((Geom *)dGeomGetData(o2))->GetContactMu());
    dReal bounce = MAX(((Geom *)dGeomGetData(o1))->GetContactBounce(),
                       ((Geom *)dGeomGetData(o2))->GetContactBounce());
    for (i = 0; i < s->m_MaxContacts; i++)
    {
        contact[i].surface.mode = dContactApprox1;
        contact[i].surface.mu = mu;
        if (bounce >= 0)
        {
            contact[i].surface.bounce = bounce;
            contact[i].surface.mode += dContactBounce;
        }
        if (cfm >= 0)
        {
            contact[i].surface.soft_cfm = cfm;
            contact[i].surface.mode += dContactSoftCFM;
        }
        if (erp <= 1)
        {
            contact[i].surface.soft_erp = erp;
            contact[i].surface.mode += dContactSoftERP;
        }
    }
    numc = dCollide(o1, o2, s->m_MaxContacts, &contact[0].geom, sizeof(dContact));
    if (numc)
    {
        for (i = 0; i < numc; i++)
        {
            dJointID c = dJointCreateContact(s->m_WorldID, s->m_ContactGroup, contact + i);
            dJointAttach(c, b1, b2);

            if (((Geom *)dGeomGetData(o1))->GetAbort()) s->SetContactAbort(true);
            if (((Geom *)dGeomGetData(o2))->GetAbort()) s->SetContactAbort(true);

#if !defined(USE_OPENGL) && !defined(USE_QT)
            if (gDebug == ContactDebug)
#endif
            {
                myContact = new Contact();
                dJointSetFeedback(c, myContact->GetJointFeedback());
                myContact->SetJointID(c);
                memcpy(myContact->GetContactPosition(), contact[i].geom.pos, sizeof(dVector3));
                s->m_ContactList.push_back(myContact);
                // only add the contact information once
                // and add it to the non-environment geom
                if (((Geom *)dGeomGetData(o1))->GetGeomLocation() == Geom::environment)
                    ((Geom *)dGeomGetData(o2))->AddContact(myContact);
                else
                    ((Geom *)dGeomGetData(o1))->AddContact(myContact);
#ifdef USE_OPENGL
                myContact->SetColour(s->m_Interface.ContactColour);
                myContact->SetAxisSize(s->m_Interface.ContactAxisSize);
                myContact->SetForceColour(s->m_Interface.ContactForceColour);
                myContact->SetForceRadius(s->m_Interface.ContactForceRadius);
                myContact->SetForceScale(s->m_Interface.ContactForceScale);
#endif
            }
        }
    }
    delete [] contact;
}

Body *Simulation::GetBody(const char *name)
{
    // use find to allow null return if name not found
    std::map<std::string, Body *>::const_iterator iter = m_BodyList.find(name);
    if (iter != m_BodyList.end()) return iter->second;
    return 0;
}

Joint *Simulation::GetJoint(const char *name)
{
    // use find to allow null return if name not found
    std::map<std::string, Joint *>::const_iterator iter = m_JointList.find(name);
    if (iter != m_JointList.end()) return iter->second;
    return 0;
}

Marker *Simulation::GetMarker(const char *name)
{
    // use find to allow null return if name not found
    std::map<std::string, Marker *>::const_iterator iter = m_MarkerList.find(name);
    if (iter != m_MarkerList.end()) return iter->second;
    return 0;
}

char *Simulation::DoXmlGetProp(xmlNode *cur, const xmlChar *name)
{
    static std::string buffer;
    xmlChar *buf = xmlGetProp(cur, name);
    if (gDebug == XMLDebug)
    {
        if (buf)
            *gDebugStream << name << "=\"" << buf << "\"\n";
        else
            *gDebugStream << name << " UNDEFINED\n";
    }
    if (buf)
    {
        strcpy(m_Buffer2, (char *)buf);
        xmlFree(buf);
        return m_Buffer2;
    }
    else return 0;
}

xmlAttr *Simulation::DoXmlReplaceProp(xmlNode *cur, const xmlChar *name, const xmlChar *newValue)
{
    xmlAttr *ptr = xmlHasProp(cur, name);
    if (ptr) xmlRemoveProp(ptr);

    ptr = xmlNewProp(cur, name, newValue);
    return ptr;
}

// this version of the dump routine simply calls the dump functions of the embedded objects
void Simulation::Dump()
{
    /*
    std::map<std::string, Body *>m_BodyList;
    std::map<std::string, Joint *>m_JointList;
    std::map<std::string, Geom *>m_GeomList;
    std::map<std::string, Muscle *>m_MuscleList;
    std::map<std::string, Driver *>m_DriverList;
    std::map<std::string, DataTarget *>m_DataTargetList; */

    std::map<std::string, Body *>::const_iterator BodyIter;
    for (BodyIter = m_BodyList.begin(); BodyIter != m_BodyList.end(); BodyIter++) BodyIter->second->Dump();

    std::map<std::string, Joint *>::const_iterator JointIter;
    for (JointIter = m_JointList.begin(); JointIter != m_JointList.end(); JointIter++) JointIter->second->Dump();

    std::map<std::string, Geom *>::const_iterator GeomIter;
    for (GeomIter = m_GeomList.begin(); GeomIter != m_GeomList.end(); GeomIter++) GeomIter->second->Dump();

    std::map<std::string, Muscle *>::const_iterator MuscleIter;
    for (MuscleIter = m_MuscleList.begin(); MuscleIter != m_MuscleList.end(); MuscleIter++) MuscleIter->second->Dump();

    std::map<std::string, Driver *>::const_iterator DriverIter;
    for (DriverIter = m_DriverList.begin(); DriverIter != m_DriverList.end(); DriverIter++) DriverIter->second->Dump();

    std::map<std::string, DataTarget *>::const_iterator DataTargetIter;
    for (DataTargetIter = m_DataTargetList.begin(); DataTargetIter != m_DataTargetList.end(); DataTargetIter++) DataTargetIter->second->Dump();

}

//----------------------------------------------------------------------------
#ifdef USE_OPENGL
void
        Simulation::Draw()
{
    for (unsigned int i = 0; i < m_Interface.DrawingOrder.size(); i++)
    {
        switch (m_Interface.DrawingOrder[i])
        {
        case DOCBody:
            {
                std::map<std::string, Body *>::const_iterator iter1;
                for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
                {
                    if (gBadMesh) iter1->second->GetFacetedObject()->SetBadMesh(true);
                    else iter1->second->GetFacetedObject()->SetBadMesh(false);
                    iter1->second->Draw();
                }
                // probably a good place to draw contact forces
                for (unsigned int c = 0; c < m_ContactList.size(); c++) m_ContactList[c]->Draw();

                break;
            }

        case DOCJoint:
            {
                std::map<std::string, Joint *>::const_iterator iter2;
                for (iter2 = m_JointList.begin(); iter2 != m_JointList.end(); iter2++) iter2->second->Draw();
                break;
            }

        case DOCGeom:
            {
                std::map<std::string, Geom *>::const_iterator iter3;
                for (iter3 = m_GeomList.begin(); iter3 != m_GeomList.end(); iter3++) iter3->second->Draw();
                break;
            }

        case DOCMuscle:
            {
                std::map<std::string, Muscle *>::const_iterator iter4;
                for (iter4 = m_MuscleList.begin(); iter4 != m_MuscleList.end(); iter4++) iter4->second->Draw();
                break;
            }

        case DOCEnvironment:
            {
                m_Environment->Draw();
                break;
            }
        }
    }
}
#endif


