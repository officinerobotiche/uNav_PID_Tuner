#include "qmotorpidwidget.h"
#include "ui_qmotorpidwidget.h"

QMotorPidWidget::QMotorPidWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::QMotorPidWidget)
{
    ui->setupUi(this);
}

QMotorPidWidget::~QMotorPidWidget()
{
    delete ui;
}
