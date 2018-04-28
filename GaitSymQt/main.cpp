#include <QtGui/QApplication>
#include <QCoreApplication>
#include <QSettings>
#include "mainwindow.h"

QSettings *settings;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QCoreApplication::setOrganizationName("Animal Simulation Laboratory");
    QCoreApplication::setOrganizationDomain("animalsimulation.org");
    QCoreApplication::setApplicationName("GaitSym");

    settings = new QSettings();

    MainWindow w;
    w.show();
    return a.exec();
}
