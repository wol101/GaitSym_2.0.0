#include <QMenu>

#include "dialogoutputselect.h"
#include "ui_dialogoutputselect.h"

#include "Simulation.h"
#include "Muscle.h"
#include "Body.h"
#include "Joint.h"
#include "Geom.h"
#include "Driver.h"
#include "DataTarget.h"

// Simulation global
extern Simulation *gSimulation;

DialogOutputSelect::DialogOutputSelect(QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::DialogOutputSelect)
{
    m_ui->setupUi(this);

    fillLists();
}

DialogOutputSelect::~DialogOutputSelect()
{
    delete m_ui;
}

void DialogOutputSelect::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void DialogOutputSelect::fillLists()
{
    if (gSimulation == 0) return;

    QListWidgetItem *item;
    int count;
    std::map<std::string, Body *> *bodyList = gSimulation->GetBodyList();
    std::map<std::string, Joint *> *jointList = gSimulation->GetJointList();
    std::map<std::string, Geom *> *geomList = gSimulation->GetGeomList();
    std::map<std::string, Muscle *> *muscleList = gSimulation->GetMuscleList();
    std::map<std::string, Driver *> *driverList = gSimulation->GetDriverList();
    std::map<std::string, DataTarget *> *dataTargetList = gSimulation->GetDataTargetList();

    count = 0;
    m_ui->listWidgetBody->clear();
    std::map<std::string, Body *>::const_iterator bodyIterator;
    for (bodyIterator = bodyList->begin(); bodyIterator != bodyList->end(); bodyIterator++)
    {
        m_ui->listWidgetBody->addItem(bodyIterator->first.c_str());
        item = m_ui->listWidgetBody->item(count++);
        if (bodyIterator->second->GetDump()) item->setCheckState(Qt::Checked);
        else item->setCheckState(Qt::Unchecked);
    }

    count = 0;
    m_ui->listWidgetJoint->clear();
    std::map<std::string, Joint *>::const_iterator jointIterator;
    for (jointIterator = jointList->begin(); jointIterator != jointList->end(); jointIterator++)
    {
        m_ui->listWidgetJoint->addItem(jointIterator->first.c_str());
        item = m_ui->listWidgetJoint->item(count++);
        if (jointIterator->second->GetDump()) item->setCheckState(Qt::Checked);
        else item->setCheckState(Qt::Unchecked);
    }

    count = 0;
    m_ui->listWidgetGeom->clear();
    std::map<std::string, Geom *>::const_iterator geomIterator;
    for (geomIterator = geomList->begin(); geomIterator != geomList->end(); geomIterator++)
    {
        m_ui->listWidgetGeom->addItem(geomIterator->first.c_str());
        item = m_ui->listWidgetGeom->item(count++);
        if (geomIterator->second->GetDump()) item->setCheckState(Qt::Checked);
        else item->setCheckState(Qt::Unchecked);
    }

    count = 0;
    m_ui->listWidgetMuscle->clear();
    std::map<std::string, Muscle *>::const_iterator muscleIterator;
    for (muscleIterator = muscleList->begin(); muscleIterator != muscleList->end(); muscleIterator++)
    {
        m_ui->listWidgetMuscle->addItem(muscleIterator->first.c_str());
        item = m_ui->listWidgetMuscle->item(count++);
        if (muscleIterator->second->GetDump()) item->setCheckState(Qt::Checked);
        else item->setCheckState(Qt::Unchecked);
    }

    count = 0;
    m_ui->listWidgetDriver->clear();
    std::map<std::string, Driver *>::const_iterator driverIterator;
    for (driverIterator = driverList->begin(); driverIterator != driverList->end(); driverIterator++)
    {
        m_ui->listWidgetDriver->addItem(driverIterator->first.c_str());
        item = m_ui->listWidgetDriver->item(count++);
        if (driverIterator->second->GetDump()) item->setCheckState(Qt::Checked);
        else item->setCheckState(Qt::Unchecked);
    }

    count = 0;
    m_ui->listWidgetDataTarget->clear();
    std::map<std::string, DataTarget *>::const_iterator dataTargetIterator;
    for (dataTargetIterator = dataTargetList->begin(); dataTargetIterator != dataTargetList->end(); dataTargetIterator++)
    {
        m_ui->listWidgetDataTarget->addItem(dataTargetIterator->first.c_str());
        item = m_ui->listWidgetDataTarget->item(count++);
        if (dataTargetIterator->second->GetDump()) item->setCheckState(Qt::Checked);
        else item->setCheckState(Qt::Unchecked);
    }

}

void DialogOutputSelect::menuRequestMuscle(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = m_ui->listWidgetMuscle->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool dump;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            dump = true;
        }
        else
        {
            state = Qt::Unchecked;
            dump = false;
        }
        for (i = 0; i < m_ui->listWidgetMuscle->count(); i++)
        {
            item = m_ui->listWidgetMuscle->item(i);
            item->setCheckState(state);
            (*gSimulation->GetMuscleList())[std::string(item->text().toAscii())]->SetAllVisible(dump);
        }
    }
}

void DialogOutputSelect::menuRequestBody(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = m_ui->listWidgetBody->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool dump;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            dump = true;
        }
        else
        {
            state = Qt::Unchecked;
            dump = false;
        }
        for (i = 0; i < m_ui->listWidgetBody->count(); i++)
        {
            item = m_ui->listWidgetBody->item(i);
            item->setCheckState(state);
            (*gSimulation->GetBodyList())[std::string(item->text().toAscii())]->SetDump(dump);
        }
    }
}

void DialogOutputSelect::menuRequestJoint(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = m_ui->listWidgetJoint->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool dump;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            dump = true;
        }
        else
        {
            state = Qt::Unchecked;
            dump = false;
        }
        for (i = 0; i < m_ui->listWidgetJoint->count(); i++)
        {
            item = m_ui->listWidgetJoint->item(i);
            item->setCheckState(state);
            (*gSimulation->GetJointList())[std::string(item->text().toAscii())]->SetDump(dump);
        }
    }
}

void DialogOutputSelect::menuRequestGeom(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = m_ui->listWidgetGeom->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool dump;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            dump = true;
        }
        else
        {
            state = Qt::Unchecked;
            dump = false;
        }
        for (i = 0; i < m_ui->listWidgetGeom->count(); i++)
        {
            item = m_ui->listWidgetGeom->item(i);
            item->setCheckState(state);
            (*gSimulation->GetGeomList())[std::string(item->text().toAscii())]->SetDump(dump);
        }
    }
}

void DialogOutputSelect::menuRequestDriver(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = m_ui->listWidgetDriver->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool dump;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            dump = true;
        }
        else
        {
            state = Qt::Unchecked;
            dump = false;
        }
        for (i = 0; i < m_ui->listWidgetDriver->count(); i++)
        {
            item = m_ui->listWidgetDriver->item(i);
            item->setCheckState(state);
            (*gSimulation->GetDriverList())[std::string(item->text().toAscii())]->SetDump(dump);
        }
    }
}

void DialogOutputSelect::menuRequestDataTarget(QPoint p)
{
    QMenu menu(this);
    menu.addAction(tr("All On"));
    menu.addAction(tr("All Off"));

    QPoint gp = m_ui->listWidgetDataTarget->mapToGlobal(p);

    QAction *action = menu.exec(gp);
    QListWidgetItem *item;
    Qt::CheckState state;
    int i;
    bool dump;
    if (action)
    {
        if (action->text() == tr("All On"))
        {
            state = Qt::Checked;
            dump = true;
        }
        else
        {
            state = Qt::Unchecked;
            dump = false;
        }
        for (i = 0; i < m_ui->listWidgetDataTarget->count(); i++)
        {
            item = m_ui->listWidgetDataTarget->item(i);
            item->setCheckState(state);
            (*gSimulation->GetDataTargetList())[std::string(item->text().toAscii())]->SetDump(dump);
        }
    }
}


