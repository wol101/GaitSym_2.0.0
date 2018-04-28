/********************************************************************************
** Form generated from reading ui file 'dialogtest.ui'
**
** Created: Wed Jul 8 13:55:15 2009
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_DIALOGTEST_H
#define UI_DIALOGTEST_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QHeaderView>

QT_BEGIN_NAMESPACE

class Ui_DialogTest
{
public:
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *DialogTest)
    {
        if (DialogTest->objectName().isEmpty())
            DialogTest->setObjectName(QString::fromUtf8("DialogTest"));
        DialogTest->resize(400, 300);
        buttonBox = new QDialogButtonBox(DialogTest);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(30, 240, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        retranslateUi(DialogTest);
        QObject::connect(buttonBox, SIGNAL(accepted()), DialogTest, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), DialogTest, SLOT(reject()));

        QMetaObject::connectSlotsByName(DialogTest);
    } // setupUi

    void retranslateUi(QDialog *DialogTest)
    {
        DialogTest->setWindowTitle(QApplication::translate("DialogTest", "Dialog", 0, QApplication::UnicodeUTF8));
        Q_UNUSED(DialogTest);
    } // retranslateUi

};

namespace Ui {
    class DialogTest: public Ui_DialogTest {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOGTEST_H
