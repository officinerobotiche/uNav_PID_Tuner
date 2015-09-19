#ifndef QMOTORPIDWIDGET_H
#define QMOTORPIDWIDGET_H

#include <QWidget>

namespace Ui {
class QMotorPidWidget;
}

class QMotorPidWidget : public QWidget
{
    Q_OBJECT

public:
    explicit QMotorPidWidget(QWidget *parent = 0);
    ~QMotorPidWidget();

private:
    Ui::QMotorPidWidget *ui;
};

#endif // QMOTORPIDWIDGET_H
