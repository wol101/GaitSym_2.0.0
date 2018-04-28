#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QFileInfo>

namespace Ui
{
    class MainWindow;
}

class GLWidget;
class DialogVisibility;
class QBoxLayout;
class QListWidgetItem;
class ViewControlWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();


public slots:
    void ReadSettings();
    void processOneThing();

    void buttonCameraRight();
    void buttonCameraTop();
    void buttonCameraFront();
    void spinboxDistanceChanged(double);
    void spinboxFoVChanged(double);
    void spinboxCOIXChanged(double);
    void spinboxCOIYChanged(double);
    void spinboxCOIZChanged(double);
    void checkboxTracking(int);
    void checkboxOverlay(int);
    void checkboxContactForce(int);
    void checkboxMuscleForce(int);
    void checkboxWhiteBackground(int);
    void checkboxBadMesh(int);
    void checkboxActivationColours(int);
    void lineeditMovieFolder(QString);
    void checkboxRecordMovie(int);
    void radioPPM(bool);
    void radioTIFF(bool);
    void radioPOVRay(bool);
    void radioOBJ(bool);
    void spinboxSkip(int);
    void spinboxTimeMax(double);
    void listMuscleChecked(QListWidgetItem*);
    void listBodyChecked(QListWidgetItem*);
    void listJointChecked(QListWidgetItem*);
    void listGeomChecked(QListWidgetItem*);
    void menuPreferences();
    void menuOutputs();
    void open();
    void about();
    void snapshot();
    void step();
    void run();
    void restart();
    void saveas();
    void saveasworld();
    void menuDefaultView();
    void menuRequestMuscle(QPoint);
    void menuRequestBody(QPoint);
    void menuRequestJoint(QPoint);
    void menuRequestGeom(QPoint);

    void setStatusString(QString s);
    void setUICOI(double x, double y, double z);
    void setUIFoV(double v);

private:

    void fillVisibitilityLists();

    Ui::MainWindow *ui;

    GLWidget *glWidget;
    QBoxLayout *boxLayout;

    ViewControlWidget *viewControlWidget;
    QBoxLayout *boxLayout2;

    QTimer *timer;
    int skip;
    long long stepCount;
    bool stepFlag;
    bool trackingFlag;

    bool movieFlag;
    QString movieFolder;

    QFileInfo configFile;
};

#endif // MAINWINDOW_H
