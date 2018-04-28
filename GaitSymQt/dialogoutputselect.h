#ifndef DIALOGOUTPUTSELECT_H
#define DIALOGOUTPUTSELECT_H

#include <QtGui/QDialog>

namespace Ui {
    class DialogOutputSelect;
}

class DialogOutputSelect : public QDialog {
    Q_OBJECT
public:
    DialogOutputSelect(QWidget *parent = 0);
    ~DialogOutputSelect();

    Ui::DialogOutputSelect *ui() { return m_ui; }

public slots:
    void menuRequestMuscle(QPoint);
    void menuRequestBody(QPoint);
    void menuRequestJoint(QPoint);
    void menuRequestGeom(QPoint);
    void menuRequestDriver(QPoint);
    void menuRequestDataTarget(QPoint);


protected:
    void changeEvent(QEvent *e);

private:

    void fillLists();

    Ui::DialogOutputSelect *m_ui;
};

#endif // DIALOGOUTPUTSELECT_H
