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

        _uNav->close();
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
        _uNav = new ParserPacket( serialPort, 115200 );

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
    ui->widget_motor_1->enableControls( true);

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
    //if( !_connected )
    //    return false;

    try
    {
        motor_control_t motor_ref = (int16_t) 0;

        quint8 command;
        if(motorIdx==0)
            command = VEL_MOTOR_L;
        else
            command = VEL_MOTOR_R;

        _uNav->parserSendPacket(_uNav->createDataPacket( command, HASHMAP_MOTION, (abstract_message_u*) & motor_ref), 3, boost::posix_time::millisec(200));

        if( !sendEnable(motorIdx, false ) )
            return false;
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}

bool MainWindow::sendEnable(int motIdx, bool enable )
{
    //if( !_connected )
    //    return false;

    try
    {
        motor_control_t enable_val = enable ? STATE_CONTROL_VELOCITY : STATE_CONTROL_DISABLE;

        quint8 command;
        if(motIdx==0)
            command = ENABLE_MOTOR_L;
        else
            command = ENABLE_MOTOR_R;

        _uNav->parserSendPacket( _uNav->createDataPacket( command, HASHMAP_MOTION, (abstract_message_u*)&enable_val ) );
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}

bool MainWindow::sendSetpoint( quint8 motorIdx, double setPoint )
{
    //if( !_connected )
    //    return false;

    try
    {
        motor_control_t motor_ref = (int16_t) (setPoint*1000.0); //Convert in centirad/s

        quint8 command;
        if(motorIdx==0)
            command = VEL_MOTOR_L;
        else
            command = VEL_MOTOR_R;

        _uNav->parserSendPacket(_uNav->createDataPacket( command, HASHMAP_MOTION, (abstract_message_u*) & motor_ref), 3, boost::posix_time::millisec(200));
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}



bool MainWindow::sendPIDGains( quint8 motorIdx, double kp, double ki, double kd )
{
    //if( !_connected )
    //    return false;

    try
    {
        pid_control_t pid;
        pid.kp = kp;
        pid.ki = ki;
        pid.kd = kd;

        quint8 command;
        if(motorIdx==0)
            command = PID_CONTROL_L;
        else
            command = PID_CONTROL_R;


        _uNav->parserSendPacket(_uNav->createDataPacket( command, HASHMAP_MOTION, (abstract_message_u*) & pid), 3, boost::posix_time::millisec(200));
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}


bool MainWindow::requestPidGains( quint8 motIdx )
{
    //if( !_connected )
    //    return false;

    try
    {
        quint8 command=PID_CONTROL_L;
        if(motIdx==0)
            command = PID_CONTROL_L;
        else
            command = PID_CONTROL_R;

        information_packet_t send = _uNav->createPacket( command, REQUEST, HASHMAP_MOTION);
        packet_t received = _uNav->sendSyncPacket( _uNav->encoder(send), 3, boost::posix_time::millisec(200) );

        // parse packet
        vector<information_packet_t> list = _uNav->parsing(received);
        //get first packet
        information_packet_t first = list.at(0);

        if(first.option == DATA)
        {
            if(first.type == HASHMAP_MOTION)
            {
                pid_control_t pid0, pid1;

                switch(first.command)
                {
                case PID_CONTROL_L:
                    pid0 = first.packet.pid;

                    ui->widget_motor_0->setPidParams( pid0.kp, pid0.ki, pid0.kd );
                    break;
                case PID_CONTROL_R:
                    pid1 = first.packet.pid;

                    ui->widget_motor_1->setPidParams( pid1.kp, pid1.ki, pid1.kd );
                    break;
                }
            }
        }

    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}

bool MainWindow::requestStatus(quint8 motIdx)
{
    //if( !_connected )
    //    return false;

    try
    {
        quint8 command=MOTOR_L;
        if(motIdx==0)
            command = MOTOR_L;
        else
            command = MOTOR_R;

        information_packet_t send = _uNav->createPacket( command, REQUEST, HASHMAP_MOTION);
        packet_t received = _uNav->sendSyncPacket( _uNav->encoder(send), 3, boost::posix_time::millisec(200) );

        // >>>>> Time
        int elapsed = _timer.elapsed();
        _timer.restart();

        _curr_time_msec += elapsed;
        // <<<<< Time

        // parse packet
        vector<information_packet_t> list = _uNav->parsing(received);
        //get first packet
        information_packet_t first = list.at(0);

        if(first.option == DATA)
        {
            if(first.type == HASHMAP_MOTION)
            {
                motor_t motor0, motor1;

                switch(first.command)
                {
                case MOTOR_L:
                    motor0 = first.packet.motor;

                    _current_value_0 = ((double)motor0.measure_vel)/1000.0;
                    _current_setPoint_0 = ((double)motor0.refer_vel)/1000.0;
                    _current_error_0 = _current_setPoint_0-_current_value_0;
                    _current_control_0 = ((double)motor0.control_vel)/1000.0;

                    ui->widget_motor_0->setStatus( _curr_time_msec, _current_value_0, _current_setPoint_0, _current_error_0, _current_control_0 );

                    //qDebug() << tr("Time: %1").arg( _curr_time_msec );

                    break;

                case MOTOR_R:
                    motor1 = first.packet.motor;

                    _current_value_1 = ((double)motor1.measure_vel)/1000.0;
                    _current_setPoint_1 = ((double)motor1.refer_vel)/1000.0;
                    _current_error_1 = _current_setPoint_1-_current_value_1;
                    _current_control_1 = ((double)motor1.control_vel)/1000.0;

                    ui->widget_motor_1->setStatus( _curr_time_msec, _current_value_1, _current_setPoint_1, _current_error_1, _current_control_1 );

                    break;
                }
            }
        }

    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}

bool MainWindow::sendMotorParams(quint8 motIdx, double k_vel, double k_ang,
                                 qint8 versus, quint8 enable_mode )
{
    //if( !_connected )
    //    return false;

    try
    {
        parameter_motor_t param;
        param.enable_set = enable_mode;
        param.k_ang = k_ang;
        param.k_vel = k_vel;
        param.versus = versus;

        quint8 command;
        if(motIdx==0)
            command = PARAMETER_MOTOR_L;
        else
            command = PARAMETER_MOTOR_R;

        _uNav->parserSendPacket(_uNav->createDataPacket(command, HASHMAP_MOTION, (abstract_message_u*) & param), 3, boost::posix_time::millisec(200));
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
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
