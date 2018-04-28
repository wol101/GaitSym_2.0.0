#include "dialogpreferences.h"
#include "ui_dialogpreferences.h"

#include <QString>
#include <QFileDialog>

DialogPreferences::DialogPreferences(QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::DialogPreferences)
{
    m_ui->setupUi(this);
}

DialogPreferences::~DialogPreferences()
{
    delete m_ui;
}

void DialogPreferences::changeEvent(QEvent *e)
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

void DialogPreferences::graphicsPathBrowse()
{
    QString folder = QFileDialog::getExistingDirectory(this, tr("Select Folder"),
                                                       "",
                                                       QFileDialog::ShowDirsOnly
                                                       | QFileDialog::DontResolveSymlinks);
    if (folder.isNull() == false)
    {
        m_ui->lineEditGraphicPath->setText(folder);
    }
}
