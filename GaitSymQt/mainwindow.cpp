#include <QMessageBox>
#include <QTimer>
#include <QFileDialog>
#include <QBoxLayout>
#include <QDesktopWidget>
#include <QListWidgetItem>
#include <QSettings>
#include <QLineEdit>
#include <QFile>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dialogpreferences.h"
#include "ui_dialogpreferences.h"
#include "dialogoutputselect.h"
#include "ui_dialogoutputselect.h"

#include "glwidget.h"
#include "viewcontrolwidget.h"
#include "ObjectiveMain.h"
#include "Simulation.h"
#include "DataFile.h"
#include "Muscle.h"
#include "Body.h"
#include "Joint.h"
#include "Geom.h"
#include "Driver.h"
#include "DataTarget.h"
#include "FacetedObject.h"

// Simulation global
extern Simulation *gSimulation;

// External display globals
extern int gDrawMuscleForces;
extern int gDrawContactForces;
extern int gBadMesh;
extern int g_ActivationDisplay;

// external settings
extern QSettings *settings;

// external file paths
extern char *gConfigFilenamePtr;
extern char *gGraphicsRoot;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    // initialise some class variables
    skip = 1;
    stepCount = 0;
    stepFlag = false;
    movieFlag = false;
    trackingFlag = false;

    // create the window elements

    ui->setupUi(this);

    // position the elements

    QDesktopWidget *desktop = qApp->desktop();
    QRect available = desktop->availableGeometry(-1);

    // careful about the various Qt size functions
    // Including the window frame:
    // x(), y(), frameGeometry(), pos() and move()
    // Excluding the window frame:
    // geometry(), width(), height(), rect() and size()

#ifdef WINDOWS_NOT_OVERLAPPED
    ui->dockWidgetControls->move(available.left(), available.top());
    ui->dockWidgetVisibility->move(available.right() - ui->dockWidgetVisibility->frameGeometry().width(), available.top());

    move(available.left() + ui->dockWidgetControls->frameGeometry().width(), available.top());
    resize(available.width() - (ui->dockWidgetControls->frameGeometry().width() + ui->dockWidgetVisibility->frameGeometry().width()),
                 available.height() - (frameGeometry().height() - height()));
#else
    move(available.left(), available.top());
    resize(available.width() - (frameGeometry().width() - width()),
                 available.height() - (frameGeometry().height() - height()));

#ifdef __APPLE__
    int verticalSpace = (frameGeometry().height() - height());
#else
    int verticalSpace = (frameGeometry().height() - height() + ui->menuBar->height());
#endif
    ui->dockWidgetControls->move(available.left(), available.top() + verticalSpace);
    ui->dockWidgetVisibility->move(available.right() - ui->dockWidgetVisibility->frameGeometry().width(), available.top() + verticalSpace);
    ui->dockWidgetView->move(available.right() - ui->dockWidgetView->frameGeometry().width(), available.top() + verticalSpace + ui->dockWidgetVisibility->frameGeometry().height());
#endif

    // put GLWidget into centralWidget
    boxLayout = new QBoxLayout(QBoxLayout::LeftToRight, ui->centralWidget);
    boxLayout->setMargin(0);
    glWidget = new GLWidget();
    boxLayout->addWidget(glWidget);

    // put ViewControlWidget into widgetViewFrame
    boxLayout2 = new QBoxLayout(QBoxLayout::LeftToRight, ui->widgetViewFrame);
    boxLayout2->setMargin(0);
    viewControlWidget = new ViewControlWidget();
    boxLayout2->addWidget(viewControlWidget);

    // connect the ViewControlWidget to the GLWidget
    QObject::connect(viewControlWidget, SIGNAL(EmitCameraVec(double, double, double)), glWidget, SLOT(SetCameraVec(double, double, double)));

    // connect the GLWidget to the MainWindow
    QObject::connect(glWidget, SIGNAL(EmitStatusString(QString)), this, SLOT(setStatusString(QString)));
    QObject::connect(glWidget, SIGNAL(EmitCOI(double, double, double)), this, SLOT(setUICOI(double, double, double)));
    QObject::connect(glWidget, SIGNAL(EmitFoV(double)), this, SLOT(setUIFoV(double)));

    // set up the timer
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(processOneThing()));

    // zero the timer
    QString time = QString("%1").arg(double(0), 0, 'f', 5);
    ui->lcdNumberTime->display(time);

    // read the settings
    ReadSettings();

    // last minute widget settings
    glWidget->SetCameraRight();

    statusBar()->showMessage(tr("Ready"));
}

MainWindow::~MainWindow()
{
    settings->sync();
    timer->stop();

    if (gSimulation) delete gSimulation;
    delete timer;
    delete glWidget;
    delete boxLayout;
    delete viewControlWidget;
    delete boxLayout2;
    delete ui;
}

void MainWindow::open()
{
    QFileInfo info = settings->value("LastFileOpened", QString("")).toString();

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Config File"), info.absolutePath(), tr("Config Files (*.xml)"));

    if (fileName.isNull() == false)
    {
        if (gSimulation) delete gSimulation;
        gSimulation = 0;
        stepCount = 0;

        ui->toolButtonPlay->setChecked(false);
        run();

        settings->setValue("LastFileOpened", fileName);
        settings->sync();

        char *readFileName = new char[fileName.length() + 1];
        strcpy(readFileName, fileName.toAscii());
        gConfigFilenamePtr = readFileName;

        QString graphicsPath = settings->value("GraphicsPath", QString("")).toString();
        char *graphicsRoot = new char[graphicsPath.length() + 1];
        strcpy(graphicsRoot, graphicsPath.toAscii());
        gGraphicsRoot = graphicsRoot;

        ReadModel();
        delete [] graphicsRoot;
        delete [] readFileName;

        fillVisibitilityLists();
        ui->doubleSpinBoxTimeMax->setValue(gSimulation->GetTimeLimit());
        QString time = QString("%1").arg(double(0), 0, 'f', 5);
        ui->lcdNumberTime->display(time);
        checkboxTracking(trackingFlag);
        glWidget->updateGL();
        statusBar()->showMessage(fileName + QString(" loaded"));
        configFile.setFile(fileName);

        // finally change the working folder to this one
        QDir::setCurrent ( configFile.absolutePath() );
    }
}

void MainWindow::restart()
{
    if (gSimulation == 0) return;
    delete gSimulation;
    gSimulation = 0;
    stepCount = 0;

    char *readFileName = new char[configFile.absoluteFilePath().length() + 1];
    strcpy(readFileName, configFile.absoluteFilePath().toAscii());
    gConfigFilenamePtr = readFileName;

    QString graphicsPath = settings->value("GraphicsPath", QString("")).toString();
    char *graphicsRoot = new char[graphicsPath.length() + 1];
    strcpy(graphicsRoot, graphicsPath.toAscii());
    gGraphicsRoot = graphicsRoot;

    ReadModel();
    delete [] graphicsRoot;
    delete [] readFileName;

    fillVisibitilityLists();
    ui->doubleSpinBoxTimeMax->setValue(gSimulation->GetTimeLimit());
    QString time = QString("%1").arg(double(0), 0, 'f', 5);
    ui->lcdNumberTime->display(time);
    checkboxTracking(trackingFlag);
    glWidget->updateGL();
    statusBar()->showMessage(configFile.fileName() + QString(" reloaded"));

}

void MainWindow::saveas()
{
    QFileInfo info = settings->value("LastFileOpened", QString("")).toString();

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Model State File"), info.absolutePath(), tr("Config Files (*.xml)"));

    if (fileName.isNull() == false)
    {
        gSimulation->SetModelStateRelative(true);
        gSimulation->SetOutputModelStateFile(fileName.toAscii());
        gSimulation->OutputProgramState();
    }
}

void MainWindow::saveasworld()
{
    QFileInfo info = settings->value("LastFileOpened", QString("")).toString();

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Model State File"), info.absolutePath(), tr("Config Files (*.xml)"));

    if (fileName.isNull() == false)
    {
        gSimulation->SetModelStateRelative(false);
        gSimulation->SetOutputModelStateFile(fileName.toAscii());
        gSimulation->OutputProgramState();
    }
}


void MainWindow::about()
 {
     QMessageBox::about(this, tr("About Menu"),
             tr("<strong>GaitSymQt<strong> Version 0.1<br>Copyright &copy; 2009 Bill Sellers<br><br>Written using Qt and ODE and released under the GNU Copyleft licence.<br><br>Source code and latest version available from http://www.animalsimulation.org"));
 }

void MainWindow::run()
{
    if (ui->toolButtonPlay->isChecked())
    {
        if (gSimulation) timer->start();
        statusBar()->showMessage(tr("Simulation running"));
    }
    else
    {
        timer->stop();
        statusBar()->showMessage(tr("Simulation stopped"));
    }
}

void MainWindow::step()
{
    stepFlag = true;
    if (gSimulation) timer->start();
    statusBar()->showMessage(tr("Simulation stepped"));
}

void MainWindow::processOneThing()
{
    if (gSimulation)
    {
        if (gSimulation->ShouldQuit() || gSimulation->TestForCatastrophy())
        {
            ui->toolButtonPlay->setChecked(false);
            run();
            return;
        }

        if ((stepCount % skip) == 0)
        {
            if (trackingFlag)
            {
                Body *body = gSimulation->GetBody(gSimulation->GetInterface()->TrackBodyID.c_str());
                if (body)
                {
                    const dReal *position = dBodyGetPosition(body->GetBodyID());
                    glWidget->SetCameraCOIX(position[0]);
                    ui->doubleSpinBoxCOIX->setValue(position[0]);
                }
            }
            if (stepFlag)
            {
                stepFlag = false;
                timer->stop();
            }
            glWidget->updateGL();
            if (movieFlag)
            {
                QString movieFullPath = QString("%1/%2").arg(configFile.absolutePath()).arg(movieFolder);
                if (QFile::exists(movieFullPath) == false)
                {
                    QDir dir("/");
                    dir.mkpath(movieFullPath);
                }
                QString filename = QString("%1/%2%3").arg(movieFullPath).arg("Frame").arg(gSimulation->GetTime(), 12, 'f', 7, QChar('0'));
                glWidget->WriteFrame(filename);
            }
            QString time = QString("%1").arg(gSimulation->GetTime(), 0, 'f', 5);
            ui->lcdNumberTime->display(time);
        }
        gSimulation->UpdateSimulation();
        stepCount++;

        if (gSimulation->ShouldQuit())
        {
            statusBar()->showMessage(tr("Simulation ended normally"));
            glWidget->updateGL();
        }
        if (gSimulation->TestForCatastrophy())
        {
            statusBar()->showMessage(tr("Simulation aborted"));
            glWidget->updateGL();
        }
    }
}

void MainWindow::snapshot()
{
    int count = 0;
    QString filename = QString("%1/Snapshop%1").arg(configFile.absolutePath()).arg(count, 5, 10, QChar('0'));
    while (QFile::exists(filename))
    {
        count++;
        filename = QString("%1/Snapshop%1").arg(configFile.absolutePath()).arg(count, 5, 10, QChar('0'));
    }
    glWidget->WriteFrame(filename);
    statusBar()->showMessage(tr("Snapshot taken"));
}

void MainWindow::fillVisibitilityLists()
{
    if (gSimulation == 0) return;

    QListWidgetItem *item;
    int count;
    std::map<std::string, Body *> *bodyList = gSimulation->GetBodyList();
    std::map<std::string, Joint *> *jointList = gSimulation->GetJointList();
    std::map<std::string, Geom *> *geomList = gSimulation->GetGeomList();
    std::map<std::string, Muscle *> *muscleList = gSimulation->GetMuscleList();

    count = 0;
    ui->listWidgetBody->clear();
    std::map<std::string, Body *>::const_iterator bodyIterator;
    for (bodyIterator = bodyList->begin(); bodyIterator != bodyList->end(); bodyIterator++)
    {
        ui->listWidgetBody->addItem(bodyIterator->first.c_str());
        item = ui->listWidgetBody->item(count++);
        item->setCheckState(Qt::Checked);
    }

    count = 0;
    ui->listWidgetJoint->clear();
    std::map<std::string, Joint *>::const_iterator jointIterator;
    for (jointIterator = jointList->begin(); jointIterator != jointList->end(); jointIterator++)
    {
        ui->listWidgetJoint->addItem(jointIterator->first.c_str());
        item = ui->listWidgetJoint->item(count++);
        item->setCheckState(Qt::Checked);
    }

    count = 0;
    ui->listWidgetGeom->clear();
    std::map<std::string, Geom *>::const_iterator geomIterator;
    for (geomIterator = geomList->begin(); geomIterator != geomList->end(); geomIterator++)
    {
        ui->listWidgetGeom->addItem(geomIterator->first.c_str());
        item = ui->listWidgetGeom->item(count++);
        item->setCheckState(Qt::Checked);
    }

    count = 0;
    ui->listWidgetMuscle->clear();
    std::map<std::string, Muscle *>::const_iterator muscleIterator;
    for (muscleIterator = muscleList->begin(); muscleIterator != muscleList->end(); muscleIterator++)
    {
        ui->listWidgetMuscle->addItem(muscleIterator->first.c_str());
        item = ui->listWidgetMuscle->item(count++);
        item->setCheckState(Qt::Checked);
    }


}

void MainWindow::buttonCameraRight()
{
    glWidget->SetCameraRight();
}


void MainWindow::buttonCameraTop()
{
    glWidget->SetCameraTop();
}


void MainWindow::buttonCameraFront()
{
    glWidget->SetCameraFront();
}


void MainWindow::spinboxDistanceChanged(double v)
{
    glWidget->SetCameraDistance(v);
}


void MainWindow::spinboxFoVChanged(double v)
{
    glWidget->SetCameraFoV(v);
}


void MainWindow::spinboxCOIXChanged(double v)
{
    glWidget->SetCameraCOIX(v);
}


void MainWindow::spinboxCOIYChanged(double v)
{
    glWidget->SetCameraCOIY(v);
}


void MainWindow::spinboxCOIZChanged(double v)
{
    glWidget->SetCameraCOIZ(v);
}


void MainWindow::checkboxTracking(int v)
{
    trackingFlag = v;
    if (gSimulation)
    {
        if (trackingFlag)
        {
            Body *body = gSimulation->GetBody(gSimulation->GetInterface()->TrackBodyID.c_str());
            if (body)
            {
                const dReal *position = dBodyGetPosition(body->GetBodyID());
                glWidget->SetCameraCOIX(position[0]);
                ui->doubleSpinBoxCOIX->setValue(position[0]);
            }
        }
    }
}


void MainWindow::checkboxOverlay(int v)
{
    glWidget->SetOverlay(v);
}


void MainWindow::checkboxContactForce(int v)
{
    gDrawContactForces = v;
    glWidget->updateGL();
}


void MainWindow::checkboxMuscleForce(int v)
{
    gDrawMuscleForces = v;
    glWidget->updateGL();
}


void MainWindow::checkboxWhiteBackground(int v)
{
    glWidget->SetWhiteBackground(v);
}


void MainWindow::checkboxBadMesh(int v)
{
    gBadMesh = v;
    if (gSimulation)
    {
        FacetedObject *facettedObject;
        std::map<std::string, Body *> *bodyList = gSimulation->GetBodyList();
        std::map<std::string, Body *>::const_iterator bodyIterator;
        for (bodyIterator = bodyList->begin(); bodyIterator != bodyList->end(); bodyIterator++)
        {
            facettedObject = bodyIterator->second->GetFacetedObject();
            facettedObject->SetValidDisplayList(false);
        }
    }
    glWidget->updateGL();
}


void MainWindow::checkboxActivationColours(int v)
{
    g_ActivationDisplay = v;
    glWidget->updateGL();
}


void MainWindow::lineeditMovieFolder(QString folder)
{
    movieFolder = folder;
}


void MainWindow::checkboxRecordMovie(int v)
{
    movieFlag = v;
}


void MainWindow::radioPPM(bool v)
{
    if (v)
    {
        glWidget->SetMovieFormat(GLWidget::PPM);
    }
}


void MainWindow::radioTIFF(bool v)
{
   if (v)
    {
        glWidget->SetMovieFormat(GLWidget::TIFF);
    }
}


void MainWindow::radioPOVRay(bool v)
{
   if (v)
    {
        glWidget->SetMovieFormat(GLWidget::POVRay);
    }
}


void MainWindow::radioOBJ(bool v)
{
   if (v)
    {
        glWidget->SetMovieFormat(GLWidget::OBJ);
    }
}


void MainWindow::spinboxSkip(int v)
{
    skip = v;
}


void MainWindow::spinboxTimeMax(double v)
{
    gSimulation->SetTimeLimit(v);
}


void MainWindow::listMuscleChecked(QListWidgetItem* item)
{
    bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetMuscleList())[std::string(item->text().toAscii())]->SetAllVisible(visible);
    glWidget->updateGL();
}


void MainWindow::listBodyChecked(QListWidgetItem* item)
{
    bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetBodyList())[std::string(item->text().toAscii())]->SetVisible(visible);
    glWidget->updateGL();
}


void MainWindow::listJointChecked(QListWidgetItem* item)
{
    bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetJointList())[std::string(item->text().toAscii())]->SetVisible(visible);
    glWidget->updateGL();
}


void MainWindow::listGeomChecked(QListWidgetItem* item)
{
   bool visible = true;
    if (item->checkState() == Qt::Unchecked) visible = false;
    (*gSimulation->GetGeomList())[std::string(item->text().toAscii())]->SetVisible(visible);
    glWidget->updateGL();
}

void MainWindow::menuPreferences()
{
    DialogPreferences dialogPreferences(this);
    Ui::DialogPreferences *dialogUI = dialogPreferences.ui();

    dialogUI->lineEditGraphicPath->setText(settings->value("GraphicsPath", QString("")).toString());
    dialogUI->lineEditMovieFolder->setText(settings->value("MoviePath", QString("movie")).toString());

    dialogUI->doubleSpinBoxDistance->setValue(settings->value("CameraDistance", 50).toDouble());
    dialogUI->doubleSpinBoxFoV->setValue(settings->value("CameraFoV", 5).toDouble());
    dialogUI->doubleSpinBoxCOIX->setValue(settings->value("CameraCOIX", 0).toDouble());
    dialogUI->doubleSpinBoxCOIY->setValue(settings->value("CameraCOIY", 0).toDouble());
    dialogUI->doubleSpinBoxCOIZ->setValue(settings->value("CameraCOIZ", 0).toDouble());
    dialogUI->doubleSpinBoxCursorRadius->setValue(settings->value("CursorRadius", 0.001).toDouble());
    dialogUI->doubleSpinBoxNudge->setValue(settings->value("Nudge", 0.001).toDouble());

    dialogUI->spinBoxSkip->setValue(settings->value("MovieSkip", 1).toInt());

    dialogUI->checkBoxYUp->setChecked(settings->value("YUp", 0).toBool());
    dialogUI->checkBoxTracking->setChecked(settings->value("DisplayTracking", 0).toBool());
    dialogUI->checkBoxOverlay->setChecked(settings->value("DisplayOverlay", 0).toBool());
    dialogUI->checkBoxContactForce->setChecked(settings->value("DisplayContactForces", 0).toBool());
    dialogUI->checkBoxMuscleForce->setChecked(settings->value("DisplayMuscleForces", 0).toBool());
    dialogUI->checkBoxWhiteBackground->setChecked(settings->value("DisplayWhiteBackground", 0).toBool());
    dialogUI->checkBoxBadMesh->setChecked(settings->value("DisplayBadMesh", 0).toBool());
    dialogUI->checkBoxActivationColours->setChecked(settings->value("DisplayActivation", 0).toBool());

    dialogUI->radioButtonPPM->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::PPM);
    dialogUI->radioButtonTIFF->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::TIFF);
    dialogUI->radioButtonPOVRay->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::POVRay);
    dialogUI->radioButtonOBJ->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::OBJ);



    int status = dialogPreferences.exec();

    if (status == QDialog::Accepted) // write the new settings
    {
        settings->setValue("GraphicsPath", dialogUI->lineEditGraphicPath->text());
        settings->setValue("MoviePath", dialogUI->lineEditMovieFolder->text());

        settings->setValue("CameraDistance", dialogUI->doubleSpinBoxDistance->value());
        settings->setValue("CameraFoV", dialogUI->doubleSpinBoxFoV->value());
        settings->setValue("CameraCOIX", dialogUI->doubleSpinBoxCOIX->value());
        settings->setValue("CameraCOIY", dialogUI->doubleSpinBoxCOIY->value());
        settings->setValue("CameraCOIZ", dialogUI->doubleSpinBoxCOIZ->value());
        settings->setValue("CursorRadius", dialogUI->doubleSpinBoxCursorRadius->value());
        settings->setValue("Nudge", dialogUI->doubleSpinBoxNudge->value());

        settings->setValue("MovieSkip", dialogUI->spinBoxSkip->value());

        settings->setValue("YUp", dialogUI->checkBoxYUp->isChecked());
        settings->setValue("DisplayTracking", dialogUI->checkBoxTracking->isChecked());
        settings->setValue("DisplayOverlay", dialogUI->checkBoxOverlay->isChecked());
        settings->setValue("DisplayContactForces", dialogUI->checkBoxContactForce->isChecked());
        settings->setValue("DisplayMuscleForces", dialogUI->checkBoxMuscleForce->isChecked());
        settings->setValue("DisplayWhiteBackground", dialogUI->checkBoxWhiteBackground->isChecked());
        settings->setValue("DisplayBadMesh", dialogUI->checkBoxBadMesh->isChecked());
        settings->setValue("DisplayActivation", dialogUI->checkBoxActivationColours->isChecked());

        if (dialogUI->radioButtonPPM->isChecked()) settings->setValue("MovieFormat", static_cast<int>(GLWidget::PPM));
        if (dialogUI->radioButtonTIFF->isChecked()) settings->setValue("MovieFormat", static_cast<int>(GLWidget::TIFF));
        if (dialogUI->radioButtonPOVRay->isChecked()) settings->setValue("MovieFormat", static_cast<int>(GLWidget::POVRay));
        if (dialogUI->radioButtonOBJ->isChecked()) settings->setValue("MovieFormat", static_cast<int>(GLWidget::OBJ));

        settings->sync();

        // these settings have immediate effect
        glWidget->SetYUp(settings->value("YUp", 0).toBool());
        glWidget->Set3DCursorRadius(settings->value("CursorRadius", 0).toDouble());
        glWidget->Set3DCursorNudge(settings->value("Nudge", 0).toDouble());
    }

}

void MainWindow::menuOutputs()
{
    if (gSimulation == 0) return;

    DialogOutputSelect dialogOutputSelect(this);
    Ui::DialogOutputSelect *dialogUI = dialogOutputSelect.ui();

    int status = dialogOutputSelect.exec();
    QListWidgetItem *item;
    int i;
    bool dump;

    if (status == QDialog::Accepted) // write the new settings
    {
        for (i = 0; i < dialogUI->listWidgetBody->count(); i++)
        {
            item = dialogUI->listWidgetBody->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetBodyList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetMuscle->count(); i++)
        {
            item = dialogUI->listWidgetMuscle->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetMuscleList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetGeom->count(); i++)
        {
            item = dialogUI->listWidgetGeom->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetGeomList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetJoint->count(); i++)
        {
            item = dialogUI->listWidgetJoint->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetJointList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetDriver->count(); i++)
        {
            item = dialogUI->listWidgetDriver->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetDriverList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

        for (i = 0; i < dialogUI->listWidgetDataTarget->count(); i++)
        {
            item = dialogUI->listWidgetDataTarget->item(i);
            if (item->checkState() == Qt::Unchecked) dump = false; else dump = true;
            (*gSimulation->GetDataTargetList())[std::string(item->text().toAscii())]->SetDump(dump);
        }

    }
}


void MainWindow::ReadSettings()
{
    glWidget->SetCameraDistance(settings->value("CameraDistance", 50).toDouble());
    glWidget->SetCameraFoV(settings->value("CameraFoV", 5).toDouble());
    glWidget->SetCameraCOIX(settings->value("CameraCOIX", 0).toDouble());
    glWidget->SetCameraCOIY(settings->value("CameraCOIY", 0).toDouble());
    glWidget->SetCameraCOIZ(settings->value("CameraCOIZ", 0).toDouble());

    trackingFlag = settings->value("DisplayTracking", 0).toBool();
    glWidget->SetOverlay(settings->value("DisplayOverlay", 0).toBool());
    gDrawContactForces = settings->value("DisplayContactForces", 0).toBool();
    gDrawMuscleForces = settings->value("DisplayMuscleForces", 0).toBool();
    glWidget->SetWhiteBackground(settings->value("DisplayWhiteBackground", 0).toBool());
    gBadMesh = settings->value("DisplayBadMesh", 0).toBool();
    g_ActivationDisplay = settings->value("DisplayActivation", 0).toBool();

    movieFolder = settings->value("MoviePath", QString("movie")).toString();
    skip = settings->value("MovieSkip", 1).toInt();
    glWidget->SetMovieFormat(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()));

    glWidget->SetYUp(settings->value("YUp", 0).toBool());

    glWidget->Set3DCursorRadius(settings->value("CursorRadius", 0).toDouble());
    glWidget->Set3DCursorNudge(settings->value("Nudge", 0).toDouble());

    ui->doubleSpinBoxDistance->setValue(settings->value("CameraDistance", 50).toDouble());
    ui->doubleSpinBoxFoV->setValue(settings->value("CameraFoV", 5).toDouble());
    ui->doubleSpinBoxCOIX->setValue(settings->value("CameraCOIX", 0).toDouble());
    ui->doubleSpinBoxCOIY->setValue(settings->value("CameraCOIY", 0).toDouble());
    ui->doubleSpinBoxCOIZ->setValue(settings->value("CameraCOIZ", 0).toDouble());

    ui->checkBoxTracking->setChecked(settings->value("DisplayTracking", 0).toBool());
    ui->checkBoxOverlay->setChecked(settings->value("DisplayOverlay", 0).toBool());
    ui->checkBoxContactForce->setChecked(settings->value("DisplayContactForces", 0).toBool());
    ui->checkBoxMuscleForce->setChecked(settings->value("DisplayMuscleForces", 0).toBool());
    ui->checkBoxWhiteBackground->setChecked(settings->value("DisplayWhiteBackground", 0).toBool());
    ui->checkBoxBadMesh->setChecked(settings->value("DisplayBadMesh", 0).toBool());
    ui->checkBoxActivationColours->setChecked(settings->value("DisplayActivation", 0).toBool());

    ui->lineEditMovieFolder->setText(settings->value("MoviePath", QString("movie")).toString());
    ui->spinBoxSkip->setValue(settings->value("MovieSkip", 1).toInt());
    ui->radioButtonPPM->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::PPM);
    ui->radioButtonTIFF->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::TIFF);
    ui->radioButtonPOVRay->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::POVRay);
    ui->radioButtonOBJ->setChecked(static_cast<GLWidget::MovieFormat>(settings->value("MovieFormat", static_cast<int>(GLWidget::TIFF)).toInt()) == GLWidget::OBJ);
}

void MainWindow::menuRequestMuscle(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetMuscle->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetMuscle->count(); i++)
        {
            item = ui->listWidgetMuscle->item(i);
            item->setCheckState(state);
            (*gSimulation->GetMuscleList())[std::string(item->text().toAscii())]->SetAllVisible(visible);
        }
        glWidget->updateGL();
    }
}

void MainWindow::menuRequestBody(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetBody->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetBody->count(); i++)
        {
            item = ui->listWidgetBody->item(i);
            item->setCheckState(state);
            (*gSimulation->GetBodyList())[std::string(item->text().toAscii())]->SetVisible(visible);
        }
        glWidget->updateGL();
    }
}

void MainWindow::menuRequestJoint(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetJoint->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetJoint->count(); i++)
        {
            item = ui->listWidgetJoint->item(i);
            item->setCheckState(state);
            (*gSimulation->GetJointList())[std::string(item->text().toAscii())]->SetVisible(visible);
        }
        glWidget->updateGL();
    }
}

void MainWindow::menuRequestGeom(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = ui->listWidgetGeom->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool visible;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            visible = true;
        }
        else
        {
            state = Qt::Unchecked;
            visible = false;
        }
        for (i = 0; i < ui->listWidgetGeom->count(); i++)
        {
            item = ui->listWidgetGeom->item(i);
            item->setCheckState(state);
            (*gSimulation->GetGeomList())[std::string(item->text().toAscii())]->SetVisible(visible);
        }
        glWidget->updateGL();
    }
}

void MainWindow::menuDefaultView()
{
    glWidget->SetCameraDistance(settings->value("CameraDistance", 50).toDouble());
    glWidget->SetCameraFoV(settings->value("CameraFoV", 5).toDouble());
    glWidget->SetCameraCOIX(settings->value("CameraCOIX", 0).toDouble());
    glWidget->SetCameraCOIY(settings->value("CameraCOIY", 0).toDouble());
    glWidget->SetCameraCOIZ(settings->value("CameraCOIZ", 0).toDouble());

    ui->doubleSpinBoxDistance->setValue(settings->value("CameraDistance", 50).toDouble());
    ui->doubleSpinBoxFoV->setValue(settings->value("CameraFoV", 5).toDouble());
    ui->doubleSpinBoxCOIX->setValue(settings->value("CameraCOIX", 0).toDouble());
    ui->doubleSpinBoxCOIY->setValue(settings->value("CameraCOIY", 0).toDouble());
    ui->doubleSpinBoxCOIZ->setValue(settings->value("CameraCOIZ", 0).toDouble());
}

void MainWindow::setStatusString(QString s)
{
    statusBar()->showMessage(s);
}

void MainWindow::setUICOI(double x, double y, double z)
{
    ui->doubleSpinBoxCOIX->setValue(x);
    ui->doubleSpinBoxCOIY->setValue(y);
    ui->doubleSpinBoxCOIZ->setValue(z);
}

void MainWindow::setUIFoV(double v)
{
    ui->doubleSpinBoxFoV->setValue(v);
}

