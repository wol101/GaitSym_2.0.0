# -------------------------------------------------
# Project created by QtCreator 2009-07-04T17:08:56
# -------------------------------------------------
QMAKE_CXXFLAGS_RELEASE += -O3 \
    -ffast-math
ICON = GaitSymQt.icns
DEFINES += USE_OPENGL \
    USE_QT \
    dDOUBLE
INCLUDEPATH += ../src \
    /usr/include/libxml2 \
    ${HOME}/Unix/include
LIBS += -lxml2 \
    ${HOME}/Unix/lib/libode.a
QT += opengl
TARGET = GaitSymQt
TEMPLATE = app
SOURCES += main.cpp \
    mainwindow.cpp \
    glwidget.cpp \
    ../src/Util.cpp \
    ../src/UGMMuscle.cpp \
    ../src/UDP.cpp \
    ../src/TwoPointStrap.cpp \
    ../src/TIFFWrite.cpp \
    ../src/ThreePointStrap.cpp \
    ../src/TCP.cpp \
    ../src/StrokeFont.cpp \
    ../src/Strap.cpp \
    ../src/StepDriver.cpp \
    ../src/SphereGeom.cpp \
    ../src/Simulation.cpp \
    ../src/PlaneGeom.cpp \
    ../src/ObjectiveMain.cpp \
    ../src/NPointStrap.cpp \
    ../src/NamedObject.cpp \
    ../src/Muscle.cpp \
    ../src/MAMuscleExtendedDamped.cpp \
    ../src/MAMuscleExtended.cpp \
    ../src/MAMuscleComplete.cpp \
    ../src/MAMuscle.cpp \
    ../src/Joint.cpp \
    ../src/IrrlichtRoutines.cpp \
    ../src/HingeJoint.cpp \
    ../src/GLUtils.cpp \
    ../src/GLUIRoutines.cpp \
    ../src/Geom.cpp \
    ../src/FloatingHingeJoint.cpp \
    ../src/FixedJoint.cpp \
    ../src/fec.cpp \
    ../src/FacetedSphere.cpp \
    ../src/FacetedPolyline.cpp \
    ../src/FacetedObject.cpp \
    ../src/FacetedConicSegment.cpp \
    ../src/Face.cpp \
    ../src/ErrorHandler.cpp \
    ../src/Environment.cpp \
    ../src/DataTarget.cpp \
    ../src/DataFile.cpp \
    ../src/DampedSpringMuscle.cpp \
    ../src/CylinderWrapStrap.cpp \
    ../src/CyclicDriver.cpp \
    ../src/Contact.cpp \
    ../src/CappedCylinderGeom.cpp \
    ../src/Body.cpp \
    ../src/BallJoint.cpp \
    dialogpreferences.cpp \
    viewcontrolwidget.cpp \
    ../src/DataTargetScalar.cpp \
    ../src/DataTargetQuaternion.cpp \
    dialogoutputselect.cpp \
    ../src/DataTargetVector.cpp \
    ../src/Driver.cpp \
    trackball.cpp \
    ../src/TrimeshGeom.cpp \
    ../src/RayGeom.cpp \
    ../src/Marker.cpp
HEADERS += mainwindow.h \
    glwidget.h \
    ../src/Util.h \
    ../src/UGMMuscle.h \
    ../src/UDP.h \
    ../src/TwoPointStrap.h \
    ../src/TIFFWrite.h \
    ../src/ThreePointStrap.h \
    ../src/TCP.h \
    ../src/StrokeFont.h \
    ../src/Strap.h \
    ../src/StepDriver.h \
    ../src/SphereGeom.h \
    ../src/SocketMessages.h \
    ../src/Simulation.h \
    ../src/SimpleStrap.h \
    ../src/PlaneGeom.h \
    ../src/PGDMath.h \
    ../src/NPointStrap.h \
    ../src/NamedObject.h \
    ../src/Muscle.h \
    ../src/MPIStuff.h \
    ../src/MAMuscleExtendedDamped.h \
    ../src/MAMuscleExtended.h \
    ../src/MAMuscleComplete.h \
    ../src/MAMuscle.h \
    ../src/Joint.h \
    ../src/IrrlichtRoutines.h \
    ../src/HingeJoint.h \
    ../src/GLUtils.h \
    ../src/GLUIRoutines.h \
    ../src/Geom.h \
    ../src/FloatingHingeJoint.h \
    ../src/FixedJoint.h \
    ../src/fec.h \
    ../src/FacetedSphere.h \
    ../src/FacetedPolyline.h \
    ../src/FacetedObject.h \
    ../src/FacetedConicSegment.h \
    ../src/Face.h \
    ../src/ErrorHandler.h \
    ../src/Environment.h \
    ../src/Driver.h \
    ../src/DebugControl.h \
    ../src/DataTarget.h \
    ../src/DataFile.h \
    ../src/DampedSpringMuscle.h \
    ../src/CylinderWrapStrap.h \
    ../src/CyclicDriver.h \
    ../src/Contact.h \
    ../src/CappedCylinderGeom.h \
    ../src/Body.h \
    ../src/BallJoint.h \
    ../src/ObjectiveMain.h \
    dialogpreferences.h \
    viewcontrolwidget.h \
    ../src/DataTargetScalar.h \
    ../src/DataTargetQuaternion.h \
    dialogoutputselect.h \
    ../src/DataTargetVector.h \
    trackball.h \
    ../src/TrimeshGeom.h \
    ../src/RayGeom.h \
    ../src/Marker.h
FORMS += mainwindow.ui \
    dialogpreferences.ui \
    dialogoutputselect.ui
RESOURCES += resources.qrc
