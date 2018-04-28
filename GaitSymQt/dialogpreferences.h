#ifndef DIALOGPREFERENCES_H
#define DIALOGPREFERENCES_H

#include <QtGui/QDialog>

namespace Ui {
    class DialogPreferences;
}

class DialogPreferences : public QDialog {
    Q_OBJECT
public:
    DialogPreferences(QWidget *parent = 0);
    ~DialogPreferences();

    Ui::DialogPreferences *ui() { return m_ui; }

public slots:

    void graphicsPathBrowse();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::DialogPreferences *m_ui;
};

#endif // DIALOGPREFERENCES_H
