#include "robotparamscalculatedialog.h"
#include "ui_robotparamscalculatedialog.h"

#include "QMessageBox"
#include "qmath.h"

RobotParamsCalculateDialog::RobotParamsCalculateDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RobotParamsCalculateDialog)
{
    ui->setupUi(this);

    _k_ang = 0.0;
    _k_vel = 0.0;
    _calculated = false;
}

RobotParamsCalculateDialog::~RobotParamsCalculateDialog()
{
    delete ui;
}

bool RobotParamsCalculateDialog::getParams(double& k_ang, double& k_vel)
{
    if( !_calculated )
        return false;

    k_ang = _k_ang;
    k_vel = _k_vel;

    return true;
}

void RobotParamsCalculateDialog::on_pushButton_clicked()
{
    _k_ang = 0.0;
    _k_vel = 0.0;
    _calculated = false;

    double freq = 80e6;

    bool ok;
    double cpr = ui->lineEdit_enc_cpr->text().toDouble( &ok );

    if(!ok)
    {
        QMessageBox::warning( this, tr("Warning"), tr("Please insert a correct value for Encoder CPR"));
        return;
    }

    double ratio = ui->lineEdit_motor_ratio->text().toDouble( &ok);

    if(!ok)
    {
        QMessageBox::warning( this, tr("Warning"), tr("Please insert a correct value for Motor Reduction ratio"));
        return;
    }

    double k_ang = (2.0*M_PI)/(4.0*ratio*cpr);

    double d_rad = k_ang * 2.0;

    double k_vel = 1000.0*d_rad*freq;

    ui->lineEdit_k_ang_left->setText( tr("%1").arg(k_ang,12,'f') );
    ui->lineEdit_k_ang_right->setText( tr("%1").arg(k_ang,12,'f') );

    ui->lineEdit_k_vel_left->setText( tr("%1").arg(k_vel,12,'f') );
    ui->lineEdit_k_vel_right->setText( tr("%1").arg(k_vel,12,'f') );

    _k_ang = k_ang;
    _k_vel = k_vel;
    _calculated = true;
}
