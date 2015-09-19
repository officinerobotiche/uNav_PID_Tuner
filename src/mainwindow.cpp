#include "mainwindow.h"
#include "ui_mainwindow.h"


#include <QtSerialPort/QSerialPortInfo>
#include <QList>
#include <QDebug>
#include <QMessageBox>
#include <string>
#include <boost/asio.hpp>




MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _uNav(NULL)
{
    ui->setupUi(this);

    ui->widget_motor_0->setMotorIdx(0);
    ui->widget_motor_1->setMotorIdx(1);

    updateSerialPortList();

    connect( ui->widget_motor_0, SIGNAL(newSetPoint(quint8,double)),
             this, SLOT(sendSetpoint(quint8,double)) );
    connect( ui->widget_motor_1, SIGNAL(newSetPoint(quint8,double)),
             this, SLOT(sendSetpoint(quint8,double)) );

    connect( ui->widget_motor_0, SIGNAL(newMotorConfig(quint8,double,double,qint8,quint8)),
             this, SLOT(sendMotorParams(quint8,double,double,qint8,quint8)) );
    connect( ui->widget_motor_1, SIGNAL(newMotorConfig(quint8,double,double,qint8,quint8)),
             this, SLOT(sendMotorParams(quint8,double,double,qint8,quint8)) );

    connect( ui->widget_motor_0, SIGNAL(newPidParams(quint8,double,double,double)),
             this, SLOT(sendPIDGains(quint8,double,double,double)) );
    connect( ui->widget_motor_1, SIGNAL(newPidParams(quint8,double,double,double)),
             this, SLOT(sendPIDGains(quint8,double,double,double)) );

    connect( ui->widget_motor_0, SIGNAL(reqStatus(quint8)),
             this, SLOT(requestStatus(quint8)) );
    connect( ui->widget_motor_1, SIGNAL(reqStatus(quint8)),
             this, SLOT(requestStatus(quint8)) );

    connect( ui->widget_motor_0, SIGNAL(startMotor(quint8)),
             this, SLOT(startMotor(quint8)) );
    connect( ui->widget_motor_1, SIGNAL(startMotor(quint8)),
             this, SLOT(startMotor(quint8)) );

    connect( ui->widget_motor_0, SIGNAL(stopMotor(quint8)),
             this, SLOT(stopMotor(quint8)) );
    connect( ui->widget_motor_1, SIGNAL(stopMotor(quint8)),
             this, SLOT(stopMotor(quint8)) );

    connect( ui->widget_motor_0, SIGNAL(reqPidParams(quint8)),
             this, SLOT(requestPidGains(quint8)) );
    connect( ui->widget_motor_1, SIGNAL(reqPidParams(quint8)),
             this, SLOT(requestPidGains(quint8)) );
}

MainWindow::~MainWindow()
{
    if(_uNav)
    {
        delete _uNav;
        _uNav = NULL;
    }
    delete ui;
}



void MainWindow::updateSerialPortList()
{
    ui->comboBox_serial_port->clear();

    QList<QSerialPortInfo> serialList;
    serialList = QSerialPortInfo::availablePorts();

    foreach( QSerialPortInfo info, serialList)
    {
        ui->comboBox_serial_port->addItem( info.systemLocation() );
    }
}

void MainWindow::on_pushButton_update_serial_list_clicked()
{
    updateSerialPortList();
}

void MainWindow::disconnectSerial()
{
    //if( !_uNav || !_connected )
    //    return;

    try
    {
        stopMotor(0);
        stopMotor(1);

        ui->widget_motor_0->stopMotor();
        ui->widget_motor_1->stopMotor();

        sendEnable(0, false );
        sendEnable(1, false );

        _uNav->disconnect();
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Disconnection error: %1").arg(e.what());

        throw e;
        return;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Disconnection error: %1").arg( e.what() );

        throw e;
        return;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Disconnection error: Unknown error");

        throw;
        return;
    }

    delete _uNav;
    _uNav = NULL;

    ui->widget_motor_0->enableControls( false );
    ui->widget_motor_1->enableControls( false );
}

bool MainWindow::connectSerial()
{
    if( _uNav )
        delete _uNav;

    string serialPort = ui->comboBox_serial_port->currentText().toStdString();

    try
    {
        _uNav = new UNavInterface();
        _uNav->connect( serialPort, 115200 );

        //_connected = true;

        /*if( !sendParams( 0 ) )
            return false;
        if( !sendParams( 1 ) )
            return false;*/

        /*int motorIdx = ui->tabWidget_motors->currentIndex();
        if( motorIdx==0 )
        {
            if( !sendEnable(0, true ) )
                return false;
            if( !sendEnable(1, false ) )
                return false;
        }
        else if( motorIdx==1 )
        {
            if( !sendEnable(0, false ) )
                return false;
            if( !sendEnable(1, true ) )
                return false;
        }*/

        if( !requestPidGains(0) )
            return false;
        if( !requestPidGains(1) )
            return false;
    }
    catch( parser_exception& e)
    {
        qDebug() << tr("Connection error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << tr("Connection error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << tr("Connection error: Unknown error");

        throw;
        return false;
    }

    ui->widget_motor_0->enableControls( true );
    ui->widget_motor_1->enableControls( true );

    //_connected = true;

    return true;
}

bool MainWindow::startMotor( quint8 motorIdx )
{
    if( !sendEnable(motorIdx, true ) )
        return false;

    _timer.start();

    return true;
}

bool MainWindow::stopMotor( quint8 motorIdx )
{
    return _uNav->sendMotorSpeed( motorIdx, 0 );
}

bool MainWindow::sendEnable(int motIdx, bool enable )
{
    return _uNav->enableSpeedControl( motIdx, enable);
}

bool MainWindow::sendSetpoint( quint8 motorIdx, double setPoint )
{
    return _uNav->sendMotorSpeed( motorIdx, setPoint );
}

bool MainWindow::sendPIDGains( quint8 motorIdx, double kp, double ki, double kd )
{
    return _uNav->sendPIDGains( motorIdx, kp, ki, kd );
}


bool MainWindow::requestPidGains( quint8 motIdx )
{
    //if( !_connected )
    //    return false;

    double kp,ki,kd;

    if( !_uNav->getPIDGains( motIdx, kp, ki, kd ) )
        return false;

    if( motIdx==0 )
        ui->widget_motor_0->setPidParams( kp, ki, kd );
    else
        ui->widget_motor_1->setPidParams( kp, ki, kd );

    return true;
}

bool MainWindow::requestStatus(quint8 motIdx)
{
    double speed, speedRef;
    bool speedOK = _uNav->getMotorSpeed( motIdx, speed );
    bool refOK = _uNav->getSpeedRef( motIdx, speedRef );

    if( !speedOK || !refOK )
        return false;

    // >>>>> Time
    int elapsed = _timer.elapsed();
    _timer.restart();

    _curr_time_msec += elapsed;
    // <<<<< Time

    switch( motIdx )
    {
    case 0:
        _current_value_0 = speed;
        _current_setPoint_0 = speedRef;
        _current_error_0 = _current_setPoint_0-_current_value_0;

        ui->widget_motor_0->setStatus( _curr_time_msec, _current_value_0, _current_setPoint_0, _current_error_0 );

        break;

    case 1:
        _current_value_1 = speed;
        _current_setPoint_1 = speedRef;
        _current_error_1 = _current_setPoint_1-_current_value_1;

        ui->widget_motor_1->setStatus( _curr_time_msec, _current_value_1, _current_setPoint_1, _current_error_1 );

        break;
    }

    return true;
}

bool MainWindow::sendMotorParams( quint8 motIdx, quint16 cpr, float ratio,
                                  qint8 versus, quint8 enable_mode,
                                  quint8 enc_pos, qint16 bridge_volt )
{
    return _uNav->sendMotorParams( motIdx, cpr, ratio, versus, enable_mode, enc_pos, bridge_volt );
}

void MainWindow::on_pushButton_connect_clicked(bool checked)
{
    if( checked )
    {
        try
        {
            if( !connectSerial() )
            {
                ui->pushButton_connect->setChecked(false);
                QMessageBox::warning( this, tr("Error"), tr("Please verify the correctness of the connection to the board") );
                return;
            }
        }
        catch( parser_exception& e)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), e.what() );
            return;
        }
        catch( boost::system::system_error& e)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), e.what() );
            return;
        }
        catch(...)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), tr("Unknown error") );
            return;
        }

        ui->pushButton_connect->setText( tr("Disconnect") );
    }
    else
    {
        try
        {
            disconnectSerial();
        }
        catch( parser_exception& e)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), e.what() );
            return;
        }
        catch( boost::system::system_error& e)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), e.what() );
            return;
        }
        catch(...)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), tr("Unknown error") );
            return;
        }

        ui->pushButton_connect->setText( tr("Connect") );
    }
}

void MainWindow::on_pushButton_reset_zoom_clicked()
{
    ui->widget_motor_0->resetZoom();
    ui->widget_motor_1->resetZoom();
}

void MainWindow::on_pushButton_reset_clicked()
{
    ui->widget_motor_0->clearPlots();
    ui->widget_motor_1->clearPlots();
}

void MainWindow::on_tabWidget_motors_destroyed()
{

}
