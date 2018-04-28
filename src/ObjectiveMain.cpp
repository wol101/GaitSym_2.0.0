/*
 *  ObjectiveMain.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#if defined(USE_MPI)
#include <mpi.h>
#endif

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <float.h>
#include <time.h>
#include <vector>

#include <ode/ode.h>

#ifdef USE_OPENGL
#ifdef USE_IRRLICHT
#include <irrlicht.h>
#include "IrrlichtRoutines.h"
#else
#if !defined(USE_CARBON) && !defined(USE_QT)
#include <glut.h>
#include "GLUIRoutines.h"
#endif
#endif
#endif


#ifdef USE_SOCKETS
#include <pinet.h>
#include "SocketMessages.h"
#endif

#include "Simulation.h"
#include "DataFile.h"
#include "Util.h"
#include "PGDMath.h"
#include "ObjectiveMain.h"

#ifdef USE_UDP
#include "UDP.h"
#include <netdb.h>
#endif

#ifdef USE_TCP
#include "TCP.h"
#include <netdb.h>
#endif

#ifdef USE_CARBON
int carbonMain(int argc, char* argv[]);
#endif

#define DEBUG_MAIN
#include "DebugControl.h"

// Simulation global
Simulation *gSimulation = 0;

// window size
int gWindowWidth = 850;
int gWindowHeight = 850;

int gFrameGrabWindowWidth = 4096;
int gFrameGrabWindowHeight = 4096;

// control window separate?
bool gUseSeparateWindow = true;

// display frames every...
// also used for kinematics output
int gDisplaySkip = 1;

// default field of view (degrees)
float gFOV = 5;

// default distance
float gCameraDistance = -40.0;

// root directory for bones graphics
char *gGraphicsRoot = 0;

// run control
bool gFinishedFlag = true;

// computer runtime limit
long gRunTimeLimit = 0;

// usleep between read failure
long gSleepTime = 10000;

// default host settings
int gPort = 8086;
char gHost[256] = "localhost";

#if defined(USE_SOCKETS)
// the server
static pt::ipstream *gHost;
#endif

// command line arguments
static char *gHostlistFilenamePtr = "hosts.txt";
char *gConfigFilenamePtr = "config.xml";
static char *gScoreFilenamePtr = 0;
static char *gOutputKinematicsFilenamePtr = 0;
static char *gOutputModelStateFilenamePtr = 0;
static char *gOutputWarehouseFilenamePtr = 0;
static dReal gOutputModelStateAtTime = -1.0;
static char *gInputKinematicsFilenamePtr = 0;
static dReal gOutputModelStateAtCycle = -1;
static bool gMungeModelStateFlag = false;
static bool gMungeRotationFlag = false;
static bool gNewStylePositionInputs = false;
static bool gNewStylePositionOutputs = false;
static int gRedundancyPercent = 0;
static bool gModelStateRelative = true;

// hostlist globals
struct Hosts
{
    char host[256];
    int port;
};
static std::vector<Hosts>gHosts;
int gUseHost = 0;

#if defined(USE_SOCKETS) || defined(USE_UDP) || defined(USE_TCP)
static void ParseHostlistFile(void);
#endif

#if defined(USE_UDP)
UDP gUDP;
#endif

#ifdef USE_TCP
TCP gTCP;
#endif

#if defined(USE_MPI)
#include "MPIStuff.h"
MPI_Status g_Status;
extern int gMPI_Comm_size;
extern int gMPI_Comm_rank;
#endif

int gAxisFlag = true;
int gDrawMuscleForces = false;
int gDrawContactForces = false;
int gDrawMuscles = true;
int gDrawBonesFlag = true;
int gDrawGeomsFlag = true;
int gWhiteBackground = false;
int gBadMesh = false;
int g_ActivationDisplay = true;

#ifndef USE_QT

#ifndef USE_MPI
int main(int argc, char *argv[])
#else
int GaitSymMain(int argc, char *argv[])
#endif
{
#if defined(USE_OPENGL) && !defined(USE_IRRLICHT) && !defined(USE_CARBON)
    // read any arguments relevent to OpenGL and remove them from the list
    // also intialises the windowing system
    glutInit(&argc, argv);
#endif
    
    // start by parsing the command line arguments
    ParseArguments(argc - 1, &(argv[1]));
    
#if defined(USE_SOCKETS) || defined(USE_UDP) || defined(USE_TCP)
    ParseHostlistFile();
#endif
    
#if defined(USE_UDP)
    if ((gUDP.StartListener(0)) == -1) 
    {
        std::cerr << "Error setting up listener\n";
        return 1;
    }
    if ((gUDP.StartTalker()) == -1)
    {
        std::cerr << "Error setting up talker\n";
        return 1;
    }
#endif

    
#if defined(USE_OPENGL)
#if defined(USE_CARBON)
    return carbonMain(argc, argv);
#else
#if defined(USE_IRRLICHT)
    // Jump to the Irrlicht code (returns on exit)
    return StartIrrlicht();
#else
    // now initialise the OpenGL bits
    StartGlut();
    
    // and enter the never to be returned loop
    glutMainLoop();
#endif
#endif
#else
    // another never returned loop (exits are in WriteModel when required)
    long runTime = 0;
    long startTime = time(0);
    while(gRunTimeLimit == 0 || runTime <= gRunTimeLimit)
    {
        runTime = time(0) - startTime;
        
        if (gFinishedFlag)
        {
            if (ReadModel() == 0)
            {
                gFinishedFlag = false;
            }
            else
            {
                usleep(gSleepTime); // slight pause on read failure
            }
        }
        else
        {
            while (gSimulation->ShouldQuit() == false)
            {
                gSimulation->UpdateSimulation();
                
                if (gSimulation->TestForCatastrophy()) break;
            }
            
            gFinishedFlag = true;
            WriteModel();
        }
    }
#endif
    return 0;
}

#endif

void ParseArguments(int argc, char ** argv)
{
    int i;
    
    if (gDebug == MainDebug)
    {
        for (i = 0; i < argc; i++)
        {
            *gDebugStream <<  "ParseArguments " << i << argv[i] << "\n";
        }
    }
    
    // do some simple stuff with command line arguments
    
    for (i = 0; i < argc; i++)
    {
        if (strcmp(argv[i], "--score") == 0 ||
            strcmp(argv[i], "-s") == 0)
        {
            i++;
            if (i >= argc)
            {
                std::cerr << "Error parsing score filename\n";
                exit(1);
            }
            gScoreFilenamePtr = argv[i];
        }
        else
            if (strcmp(argv[i], "--config") == 0 ||
                strcmp(argv[i], "-c") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing config filename\n";
                    exit(1);
                }
                gConfigFilenamePtr = argv[i];
            }
        else
            if (strcmp(argv[i], "--graphicsRoot") == 0 ||
                strcmp(argv[i], "-g") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing graphics root directory\n";
                    exit(1);
                }
                gGraphicsRoot = argv[i];
            }
        else
            if (strcmp(argv[i], "--debug") == 0 ||
                strcmp(argv[i], "-d") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing debug level\n";
                    exit(1);
                }
                gDebug = (DebugControl)strtol(argv[i], 0, 10);
            }
        else
            if (strcmp(argv[i], "--debugFunctionFilter") == 0 ||
                strcmp(argv[i], "-df") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing debugFunctionFilter\n";
                    exit(1);
                }
                gDebugFunctionFilter = argv[i];
            }
        else
            if (strcmp(argv[i], "--debugNameFilter") == 0 ||
                strcmp(argv[i], "-dn") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing debugNameFilter\n";
                    exit(1);
                }
                gDebugNameFilter = argv[i];
            }
        else
            if (strcmp(argv[i], "--windowSize") == 0 ||
                strcmp(argv[i], "-w") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing window size x\n";
                    exit(1);
                }
                gWindowWidth = strtol(argv[i], 0, 10);
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing window size y\n";
                    exit(1);
                }
                gWindowHeight = strtol(argv[i], 0, 10);
            }
        else
            if (strcmp(argv[i], "--framGrabWindowSize") == 0 ||
                strcmp(argv[i], "-W") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing window size x\n";
                    exit(1);
                }
                gFrameGrabWindowWidth = strtol(argv[i], 0, 10);
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing window size y\n";
                    exit(1);
                }
                gFrameGrabWindowHeight = strtol(argv[i], 0, 10);
            }
        else
            if (strcmp(argv[i], "--displaySkip") == 0 ||
                strcmp(argv[i], "-S") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing --displaySkip\n";
                    exit(1);
                }
                gDisplaySkip = strtol(argv[i], 0, 10);
            }
        else 
            if (strcmp(argv[i], "--fieldOfView") == 0 ||
                strcmp(argv[i], "-f") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing --fieldOfView\n";
                    exit(1);
                }
                gFOV = strtod(argv[i], 0);
            }
        else 
            if (strcmp(argv[i], "--cameraDistance") == 0 ||
                strcmp(argv[i], "-D") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing --cameraDistance\n";
                    exit(1);
                }
                gCameraDistance = strtod(argv[i], 0);
            }
        else
            if (strcmp(argv[i], "--hostList") == 0 ||
                strcmp(argv[i], "-L") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing --hostList\n";
                    exit(1);
                }
                gHostlistFilenamePtr = argv[i];
            }
        else
            if (strcmp(argv[i], "--runTimeLimit") == 0 ||
                strcmp(argv[i], "-r") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing --runTimeLimit\n";
                    exit(1);
                }
                gRunTimeLimit = strtol(argv[i], 0, 10);
            }
        else
            if (strcmp(argv[i], "--outputKinematics") == 0 ||
                strcmp(argv[i], "-K") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing output kinematics filename\n";
                    exit(1);
                }
                gOutputKinematicsFilenamePtr = argv[i];
            }
        else
            if (strcmp(argv[i], "--inputKinematics") == 0 ||
                strcmp(argv[i], "-J") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing input kinematics filename\n";
                    exit(1);
                }
                gInputKinematicsFilenamePtr = argv[i];
            }
        else
            if (strcmp(argv[i], "--outputWarehouse") == 0 ||
                strcmp(argv[i], "-H") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing input kinematics filename\n";
                    exit(1);
                }
                gOutputWarehouseFilenamePtr = argv[i];
            }
        else
            if (strcmp(argv[i], "--outputModelStateFile") == 0 ||
                strcmp(argv[i], "-M") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing model state filename\n";
                    exit(1);
                }
                gOutputModelStateFilenamePtr = argv[i];
            }
        else
            if (strcmp(argv[i], "--outputModelStateAtTime") == 0 ||
                strcmp(argv[i], "-t") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing --outputModelStateAtTime\n";
                    exit(1);
                }
                gOutputModelStateAtTime = strtod(argv[i], 0);
            }
        else
            if (strcmp(argv[i], "--outputModelStateAtCycle") == 0 ||
                strcmp(argv[i], "-T") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing --outputModelStateAtCycle\n";
                    exit(1);
                }
                gOutputModelStateAtCycle = strtod(argv[i], 0);
            }
        else
            if (strcmp(argv[i], "--redundancyPercent") == 0 ||
                strcmp(argv[i], "-R") == 0)
            {
                i++;
                if (i >= argc)
                {
                    std::cerr << "Error parsing --redundancyPercent\n";
                    exit(1);
                }
                gRedundancyPercent = strtol(argv[i], 0, 10);
            }
        else
            if (strcmp(argv[i], "--MungeModelState") == 0 ||
                strcmp(argv[i], "-U") == 0)
            {
                gMungeModelStateFlag = true;
            }
        else
            if (strcmp(argv[i], "--MungeRotation") == 0 ||
                strcmp(argv[i], "-u") == 0)
            {
                gMungeRotationFlag = true;
            }
        else
            if (strcmp(argv[i], "--ModelStateWorld") == 0 ||
                strcmp(argv[i], "-e") == 0)
            {
                gModelStateRelative = false;
            }
        else
            if (strcmp(argv[i], "--NewStylePositionInputs") == 0 ||
                strcmp(argv[i], "-N") == 0)
            {
                gNewStylePositionInputs = true;
            }
        else
            if (strcmp(argv[i], "--NewStylePositionOutputs") == 0 ||
                strcmp(argv[i], "-O") == 0)
            {
                gNewStylePositionOutputs = true;
            }
        else
            if (strcmp(argv[i], "--quiet") == 0 ||
                strcmp(argv[i], "-q") == 0)
            {
                freopen ("/dev/null", "w", stdout);
                freopen ("/dev/null", "w", stderr);
            }
        else
            if (strcmp(argv[i], "--help") == 0 ||
                strcmp(argv[i], "-h") == 0 ||
                strcmp(argv[i], "-?") == 0)
            {
                std::cerr << "\nObjective build " << __DATE__ << " " << __TIME__ << "\n\n";
                std::cerr << "-c filename, --config filename\n";
                std::cerr << "Reads filename rather than the default config.xml as the config data\n\n";
                std::cerr << "-s filename, --score filename\n";
                std::cerr << "Writes filename rather than the default score.tmp as the fitness data\n\n";
                std::cerr << "-g path, --graphicsRoot path\n";
                std::cerr << "Prepends path to the graphics filenames\n\n";
                std::cerr << "-w x y, --windowSize x y\n";
                std::cerr << "Sets the initial display window size - requires separate x and y values.\n\n";
                std::cerr << "-W x y, --frameGrabWindowSize x y\n";
                std::cerr << "Sets the frame grab window size - requires separate x and y values.\n\n";
                std::cerr << "-S n, --displaySkip n\n";
                std::cerr << "Displays a new frame every n calculations\n\n";
                std::cerr << "-f x, --fieldOfView x\n";
                std::cerr << "Set the field of view angle (degrees)\n\n";
                std::cerr << "-D x, --cameraDistance x\n";
                std::cerr << "Set the camera distance (metres)\n\n";
                std::cerr << "-L filename, --hostList filename\n";
                std::cerr << "Set the filename for the list of servers for the socket version\n\n";
                std::cerr << "-r n, --runTimeLimit n\n";
                std::cerr << "Quits the program (approximately) after it has run n seconds\n\n";
                std::cerr << "-J filename, --inputKinematics filename\n";
                std::cerr << "Reads tab-delimited kinematic data from filename\n\n";
                std::cerr << "-K filename, --outputKinematics filename\n";
                std::cerr << "Writes tab-delimited kinematic data to filename\n\n";
                std::cerr << "-H filename, --outputWarehouse filename\n";
                std::cerr << "Writes tab-delimited gait warehouse data to filename\n\n";
                std::cerr << "-M filename, --outputModelStateFile filename\n";
                std::cerr << "Sets the model state filename\n\n";
                std::cerr << "-t x, --outputModelStateAtTime x\n";
                std::cerr << "Writes the model state to model state file at time x\n\n";
                std::cerr << "-T n, --outputModelStateAtCycle x\n";
                std::cerr << "Writes the model state to model state file at cycle x\n\n";
                std::cerr << "-R n, --redundancyPercent n\n";
                std::cerr << "% redundancy for forward error correction with UDP (set over 0 for effect)\n\n";
                std::cerr << "-U, --MungeModelState\n";
                std::cerr << "Munges the linear data in the model state file\n\n";
                std::cerr << "-u, --MungeRotation\n";
                std::cerr << "Munges the rotation data in the model state file\n\n";
                std::cerr << "-e, --ModelStateWorld\n";
                std::cerr << "Uses World data in the model state file\n\n";
                std::cerr << "-N, --NewStylePositionInputs\n";
                std::cerr << "Uses new standardised position and quaternion inputs\n\n";
                std::cerr << "-O, --NewStylePositionOutputs\n";
                std::cerr << "Uses new standardised position and quaternion outputs\n\n";
                std::cerr << "-q, --quiet\n";
                std::cerr << "Suppresses stdout and stderr messages by redirecting to /dev/null\n\n";
                std::cerr << "-h, -?, --help\n";
                std::cerr << "Prints this message!\n\n";
                
                int nDebugLabels = sizeof(gDebugLabels) / sizeof(gDebugLabels[0]);
                std::cerr << "-d n, --debug n\n";
                std::cerr << "Prints out a lot of extra debugging information on stderr if n is higher than 0.\n";
                std::cerr << "-df name, --debugFunctionFilter name\n";
                std::cerr << "Filters debugging information by function.\n";
                std::cerr << "-dn name, --debugNameFilter name\n";
                std::cerr << "Filters debugging information by name.\n";
                std::cerr << "Debug numbers:\n";
                for (i = 0; i < nDebugLabels; i++)
                    fprintf(stderr, "%3d %s\n", i, gDebugLabels[i]);
                std::cerr << "\n";
                exit(1);
            }
        else
        {
            std::cerr << "Unrecognised option. Try 'objective --help' for more info\n";
            exit(1);
        }
    }
}

// this routine attemps to read the model specification and initialise the simulation
// it returns zero on success
int ReadModel()
{
#if defined(USE_SOCKETS)
    char *buf = 0;
    int l;
    char buffer[kSocketMaxMessageLength + 1];
    buffer[kSocketMaxMessageLength] = 0;
#endif
    
    DataFile myFile;
#if !defined(USE_SOCKETS) && !defined(USE_UDP) && !defined(USE_TCP) && !defined(USE_MPI)
    myFile.SetExitOnError(true);
#endif
    
    // load the config file
#if defined(USE_SOCKETS)
    gHost = 0;
    
    if (gDebug == SocketDebug) *gDebugStream <<  "ReadModel gUseHost " << gUseHost 
    	<< " host " << gHosts[gUseHost].host 
    	<< " port " << gHosts[gUseHost].port 
        << "\n";
    
    // get model config file from server
    try
    {
        gHost = new pt::ipstream(gHosts[gUseHost].host, gHosts[gUseHost].port);
        gHost->open();
        strcpy(buffer, kSocketRequestTaskMessage);
        gHost->write(buffer, kSocketMaxMessageLength);
        gHost->flush();
        if (gDebug == SocketDebug) *gDebugStream <<  "ReadModel Start String\n" << buffer << "\n";
        
        // wait up to 1000ms for data
        if (gHost->waitfor(1000) == false) 
        {
            if (gDebug == SocketDebug) *gDebugStream << "Error: waited too long for server response\n";
            throw (__LINE__);
        }
        
        gHost->read(buffer, kSocketMaxMessageLength);
        if (gDebug == SocketDebug) *gDebugStream <<  "ReadModel Reply String\n" << buffer << "\n";
        if (strcmp(buffer, kSocketSendingTask) != 0) 
        {
            if (gDebug == SocketDebug) *gDebugStream <<  "Error: incorrect reply string\n";
            throw (__LINE__);
        }
        gHost->read(buffer, 16);
        l = (int)strtol(buffer, 0, 10);
        buf = new char[l + 1];
        buf[l] = 0;
        gHost->read(buf, l);
        
        if (gDebug == SocketDebug) *gDebugStream << "ReadModel Config File\n" <<  buf << "\n";
        
        myFile.SetRawData(buf);
        delete [] buf;
        buf = 0;
    }
    catch (int e)
    {
        if (buf) delete [] buf;
        delete gSimulation;
        if (gHost) delete gHost;
        gUseHost++;
        if (gUseHost >= gHosts.size()) gUseHost = 0;
        return 1;
    }
    catch (pt::estream* e)
    {
        delete e;
        if (buf) delete [] buf;
        delete gSimulation;
        delete gHost;
        gUseHost++;
        if (gUseHost >= gHosts.size()) gUseHost = 0;
        return 1;
    }
#elif defined(USE_UDP)
    
    if (gDebug == UDPDebug) *gDebugStream <<  "ReadModel gUseHost " << gUseHost 
    	<< " host " << gHosts[gUseHost].host 
    	<< " port " << gHosts[gUseHost].port 
        << "\n";
    
    // get model config file from server

    try
    {
        struct hostent *he;
        struct sockaddr_in their_addr;
        if ((he = gethostbyname(gHosts[gUseHost].host)) == NULL) throw __LINE__;
        their_addr.sin_family = AF_INET; // host byte order 
        their_addr.sin_port = htons(gHosts[gUseHost].port); // short, network byte order 
        their_addr.sin_addr = *((struct in_addr *)he->h_addr); 
        memset(&(their_addr.sin_zero), 0, 8); // zero the rest of the struct 
        
        gUDP.BumpUDPPacketID();
        ((RequestSendGenomeUDPPacket *)gUDP.GetUDPPacket())->type = request_send_genome;
        ((RequestSendGenomeUDPPacket *)gUDP.GetUDPPacket())->port = gUDP.GetMyAddress()->sin_port;
        ((RequestSendGenomeUDPPacket *)gUDP.GetUDPPacket())->packetID = gUDP.GetUDPPacketID();
        int numBytes;
        if ((numBytes = gUDP.SendUDPPacket(&their_addr, sizeof(RequestSendGenomeUDPPacket))) == -1) throw __LINE__;
        
        if (gUDP.CheckReceiver(100000) != 1) throw __LINE__;
        
        char *buf;    
        if (gRedundancyPercent <= 0)
        {
            if ((numBytes = gUDP.ReceiveText(&buf, gUDP.GetUDPPacketID())) == -1)  throw __LINE__;
        }
        else
        {
            if ((numBytes = gUDP.ReceiveFEC(&buf, gUDP.GetUDPPacketID(), gRedundancyPercent + 100)) == -1)  throw __LINE__;
        }
        
        ((GenomeReceivedUDPPacket *)gUDP.GetUDPPacket())->type = genome_received;
        ((GenomeReceivedUDPPacket *)gUDP.GetUDPPacket())->port = gUDP.GetMyAddress()->sin_port;
        ((GenomeReceivedUDPPacket *)gUDP.GetUDPPacket())->packetID = gUDP.GetUDPPacketID();
        if ((numBytes = gUDP.SendUDPPacket(&their_addr, sizeof(GenomeReceivedUDPPacket))) == -1) throw __LINE__;
        
        myFile.SetRawData(buf);
        delete [] buf;
        
    }
    
    catch (int e)
    {
        if (gDebug == UDPDebug) *gDebugStream <<  "ReadModel error on line " << e << "\n"; 
        gUseHost++;
        if (gUseHost >= gHosts.size()) gUseHost = 0;
        return 1;
    }
    
#elif defined(USE_TCP)
    
    if (gDebug == TCPDebug) *gDebugStream <<  "ReadModel gUseHost " << gUseHost 
    	<< " host " << gHosts[gUseHost].host 
    	<< " port " << gHosts[gUseHost].port 
        << "\n";
    
    // get model config file from server
    
    int status;
    int numBytes, len;
    char buffer[16];
    try
    {
        status = gTCP.StartClient(gHosts[gUseHost].port, gHosts[gUseHost].host);
        if (status != 0) throw -1 * __LINE__;
        
        strcpy(buffer, "req_send_length");
        numBytes = gTCP.SendData(buffer, 16);
        if (numBytes != 16) throw __LINE__;
        
        numBytes = gTCP.ReceiveData(buffer, 16, 1, 0);
        if (numBytes != 16) throw __LINE__;
        len = *(int *)buffer;
        char *buf = new char[len];
        
        strcpy(buffer, "req_send_data");
        numBytes = gTCP.SendData(buffer, 16);
        if (numBytes != 16) throw __LINE__;

        numBytes = gTCP.ReceiveData(buf, len, 1, 0);
        if (numBytes < len) throw __LINE__;
        myFile.SetRawData(buf);
        delete [] buf;
        
    }
    
    catch (int e)
    {
        if (e > 0) gTCP.StopClient();
        if (gDebug == TCPDebug) *gDebugStream <<  "ReadModel error on line " << e << "\n"; 
        gUseHost++;
        if (gUseHost >= gHosts.size()) gUseHost = 0;
        return 1;
    }
    
#elif defined(USE_MPI)
    
    MPI_Probe(MPI_ANY_SOURCE,    /* source */
              MPI_ANY_TAG,       /* filter by tag */
              MPI_COMM_WORLD,    /* default communicator */
              &g_Status);        /* info about the received message */
    
    if (gDebug == MPIDebug) std::cerr << "ReadModel MPI_Probe g_Status.MPI_TAG " << g_Status.MPI_TAG << "\n";

#if defined(USE_MPI_QUIT)    
    if (g_Status.MPI_TAG == MPI_TAG_QUIT) // this is the quit tag (needs to be defined at compile time)
    {
       std::cerr << "MPI_Finalize MPI_TAG = " << g_Status.MPI_TAG << "\n";
       MPI_Finalize();
       exit(0);
    }
#endif
 
    int count;
    MPI_Get_count(&g_Status, MPI_BYTE, &count);
    if (gDebug == MPIDebug) std::cerr << "ReadModel MPI_Get_count count " << count << "\n";
    char *data = new char[count];
    MPI_Recv(data,               /* message buffer */ 
             count,              /* max number of data items */ 
             MPI_BYTE,           /* of type BYTE */ 
             0,                  /* receive from server */ 
             MPI_ANY_TAG,        /* any type of message */ 
             MPI_COMM_WORLD,     /* default communicator */ 
             &g_Status);         /* info about the received message */
    
    myFile.SetRawData(data);
    delete [] data;
    
#else
    myFile.ReadFile(gConfigFilenamePtr);
#endif
    
    // create the simulation object 
    gSimulation = new Simulation();
    if (gOutputKinematicsFilenamePtr) gSimulation->SetOutputKinematicsFile(gOutputKinematicsFilenamePtr);
    if (gInputKinematicsFilenamePtr) gSimulation->SetInputKinematicsFile(gInputKinematicsFilenamePtr);
    if (gOutputWarehouseFilenamePtr) gSimulation->SetOutputWarehouseFile(gOutputWarehouseFilenamePtr);
    if (gOutputModelStateFilenamePtr) gSimulation->SetOutputModelStateFile(gOutputModelStateFilenamePtr);
    if (gOutputModelStateAtTime >= 0) gSimulation->SetOutputModelStateAtTime(gOutputModelStateAtTime);
    if (gOutputModelStateAtCycle >= 0) gSimulation->SetOutputModelStateAtCycle(gOutputModelStateAtCycle);
    if (gMungeModelStateFlag) gSimulation->SetMungeModelStateFlag(true);
    if (gMungeRotationFlag) gSimulation->SetMungeRotationFlag(true);
    if (gNewStylePositionInputs) gSimulation->SetOldStyleInputs(false);
    if (gNewStylePositionOutputs) gSimulation->SetOldStyleOutputs(false);
    gSimulation->SetModelStateRelative(gModelStateRelative);
    
    if (gSimulation->LoadModel(myFile.GetRawData())) return 1;
    
    return 0;
}

void WriteModel()
{
    dReal score;
#if defined(USE_SOCKETS)
    char buffer[kSocketMaxMessageLength];
#endif
    
    score = gSimulation->CalculateInstantaneousFitness();
    // if (gSimulation->TestForCatastrophy())
    //  score -= 100000;
    
    std::cerr << "Simulation Time: " << gSimulation->GetTime() <<
        " Steps: " << gSimulation->GetStepCount() <<
        " Score: " << score <<
        " Mechanical Energy: " << gSimulation->GetMechanicalEnergy() <<
        " Metabolic Energy: " << gSimulation->GetMetabolicEnergy() << "\n";
    
#if defined(USE_SOCKETS)
    try
    {
        strcpy(buffer, kSocketSendingScore);
        gHost->write(buffer, kSocketMaxMessageLength);
        sprintf(buffer, "%.17f", score);
        gHost->write(buffer, 64);
        gHost->flush();
        if (gDebug == SocketDebug) *gDebugStream << "ObjectiveMain Score\n" <<  buffer << "\n";
    }
    catch (pt::estream* e)
    {
        delete e;
    }
    delete gHost;
#elif defined(USE_UDP)
    try
    {
        struct hostent *he;
        struct sockaddr_in their_addr;
        if ((he = gethostbyname(gHosts[gUseHost].host)) == NULL) throw __LINE__;
        their_addr.sin_family = AF_INET; // host byte order 
        their_addr.sin_port = htons(gHosts[gUseHost].port); // short, network byte order 
        their_addr.sin_addr = *((struct in_addr *)he->h_addr); 
        memset(&(their_addr.sin_zero), 0, 8); // zero the rest of the struct 
        
        ((SendResultUDPPacket *)gUDP.GetUDPPacket())->type = send_result;
        ((SendResultUDPPacket *)gUDP.GetUDPPacket())->result = score;
        ((SendResultUDPPacket *)gUDP.GetUDPPacket())->port = gUDP.GetMyAddress()->sin_port;
        ((SendResultUDPPacket *)gUDP.GetUDPPacket())->packetID = gUDP.GetUDPPacketID();
        int numBytes;
        if ((numBytes = gUDP.SendUDPPacket(&their_addr, sizeof(SendResultUDPPacket))) == -1) throw __LINE__;
    }
    catch (int e)
    {
        if (gDebug == TCPDebug) *gDebugStream <<  "WriteModel error on line " << e << "\n";
    }
#elif defined(USE_TCP)
    try
    {
        char buffer[16];
        double doubleScore = score;
        int numBytes;
        // send the data
        memcpy(buffer, &doubleScore, sizeof(doubleScore));
        numBytes = gTCP.SendData(buffer, 16);
        if (numBytes != 16) throw __LINE__;
   }
    catch (int e)
    {
        if (gDebug == TCPDebug) *gDebugStream <<  "WriteModel error on line " << e << "\n"; 
    }
    gTCP.StopClient();
#elif defined(USE_MPI)
    double results[2];
    results[0] = (double)g_Status.MPI_TAG;
    results[1] = score;
    if (gDebug == MPIDebug) std::cerr << "WriteModelModel MPI_Send results[0] " << results[0] << "\n";
    MPI_Send((char *)results,    /* message buffer */ 
             sizeof(double) * 2, /* 'len' data item */ 
             MPI_BYTE,           /* data items are bytes */ 
             0,                  /* destination */ 
             send_result,        /* user chosen message tag */ 
             MPI_COMM_WORLD);    /* server */ 
#else
    if (gScoreFilenamePtr)
    {
        FILE *out;
        out = fopen(gScoreFilenamePtr, "wb");
        fwrite(&score, sizeof(dReal), 1, out);
        fclose(out);
    }
#endif
    
    if (gDebug == MemoryDebug)
        *gDebugStream << "main About to delete gSimulation\n";
    delete gSimulation;
    
#if ! defined(USE_SOCKETS) && ! defined (USE_UDP) && ! defined (USE_TCP) && ! defined(USE_MPI)
    std::cerr << "exiting\n";
    exit(0);
#endif
}

#if defined(USE_SOCKETS) || defined(USE_UDP) || defined(USE_TCP)
// read the file containing a list of hosts and ports
void ParseHostlistFile(void)
{
    char buffer[256];
    char *tokens[256];
    bool result;
    int count;
    Hosts host;
    DataFile hostListFile;
    hostListFile.SetExitOnError(false);
    
    if (hostListFile.ReadFile(gHostlistFilenamePtr))
    {
        strcpy(host.host, "localhost");
        host.port = 8086;
        gHosts.push_back(host);
        std::cerr << "No hosts file. Using localhost:8086\n";
    }
    else
    {
        do
        {
            result = hostListFile.ReadNextLine(buffer, 256, true, '#');
            count = DataFile::ReturnTokens(buffer, tokens, 256);
            if (count >= 2)
            {
                strcpy(host.host, tokens[0]);
                host.port = strtol(tokens[1], 0, 10);
                gHosts.push_back(host);
            }
        } while (result == false);
    }
    
    if (gHosts.size() == 0)
    {
        std::cerr << "Could not get a list of hosts from " << gHostlistFilenamePtr << "\n";
        exit(-1);
    }
}

#endif



