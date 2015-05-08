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
        motor_command_map_t motor_command;
        motor_command.bitset.motor = motorIdx;
        motor_command.bitset.command = MOTOR_VEL_PID;

        _uNav->parserSendPacket(_uNav->createDataPacket( motor_command.command_message, HASHMAP_MOTOR, (message_abstract_u*) & motor_ref), 3, boost::posix_time::millisec(200));

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
        motor_state_t enable_val = enable ? STATE_CONTROL_VELOCITY : STATE_CONTROL_DISABLE;
        motor_command_map_t motor_command;
        motor_command.bitset.motor = motIdx;
        motor_command.bitset.command = MOTOR_STATE;

        _uNav->parserSendPacket( _uNav->createDataPacket( motor_command.command_message, HASHMAP_MOTOR, (message_abstract_u*)&enable_val ) );
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
        motor_command_map_t motor_command;
        motor_command.bitset.motor = motorIdx;
        motor_command.bitset.command = MOTOR_VEL_REF;

        _uNav->parserSendPacket(_uNav->createDataPacket( motor_command.command_message, HASHMAP_MOTOR, (message_abstract_u*) & motor_ref), 3, boost::posix_time::millisec(200));
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
        motor_pid_t pid;
        pid.kp = kp;
        pid.ki = ki;
        pid.kd = kd;
        motor_command_map_t motor_command;
        motor_command.bitset.motor = motorIdx;
        motor_command.bitset.command = MOTOR_VEL_PID;

        _uNav->parserSendPacket(_uNav->createDataPacket( motor_command.command_message, HASHMAP_MOTOR, (message_abstract_u*) & pid), 3, boost::posix_time::millisec(200));
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
        motor_command_map_t motor_command;
        motor_command.bitset.motor = motIdx;
        motor_command.bitset.command = MOTOR_VEL_PID;

        packet_information_t send = _uNav->createPacket( motor_command.command_message, PACKET_REQUEST, HASHMAP_MOTOR);
        packet_t received = _uNav->sendSyncPacket( _uNav->encoder(send), 3, boost::posix_time::millisec(200) );

        // parse packet
        vector<packet_information_t> list = _uNav->parsing(received);
        //get first packet
        packet_information_t first = list.at(0);

        if(first.option == PACKET_DATA)
        {
            if(first.type == HASHMAP_MOTOR)
            {
                motor_pid_t pid0, pid1;
                motor_command_map_t command;
                command.command_message = first.command;
                switch(command.bitset.command)
                {
                case MOTOR_VEL_PID:
                    switch(command.bitset.motor) {
                    case 0:
                        pid0 = first.message.motor_pid;
                        ui->widget_motor_0->setPidParams( pid0.kp, pid0.ki, pid0.kd );
                        break;
                    case 1:
                        pid1 = first.message.motor_pid;
                        ui->widget_motor_1->setPidParams( pid1.kp, pid1.ki, pid1.kd );
                        break;
                    }
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
        motor_command_map_t motor_command;
        motor_command.bitset.motor = motIdx;
        motor_command.bitset.command = MOTOR;
        motor_command_map_t motor_command2;
        motor_command2.bitset.motor = motIdx;
        motor_command2.bitset.command = MOTOR_VEL_REF;

        vector<packet_information_t> list_send;
        list_send.push_back(_uNav->createPacket( motor_command.command_message, PACKET_REQUEST, HASHMAP_MOTOR));
        list_send.push_back(_uNav->createPacket( motor_command2.command_message, PACKET_REQUEST, HASHMAP_MOTOR));

        packet_t received = _uNav->sendSyncPacket( _uNav->encoder(list_send), 3, boost::posix_time::millisec(200) );

        // >>>>> Time
        int elapsed = _timer.elapsed();
        _timer.restart();

        _curr_time_msec += elapsed;
        // <<<<< Time

        // parse packet
        vector<packet_information_t> list = _uNav->parsing(received);
        //get first packet
        packet_information_t first = list.at(0);

        if(first.option == PACKET_DATA)
        {
            if(first.type == HASHMAP_MOTION)
            {
                motor_t motor0, motor1;
                motor_control_t ref0, ref1;
                motor_command_map_t command;
                command.command_message = first.command;
                switch(command.bitset.command)
                {
                case MOTOR:
                    switch(command.bitset.motor) {
                    case 0:
                        motor0 = first.message.motor;

                        _current_value_0 = ((double)motor0.velocity)/1000.0;
                        _current_error_0 = _current_setPoint_0-_current_value_0;
                        _current_control_0 = ((double)motor0.volt)/1000.0;

                        ui->widget_motor_0->setStatus( _curr_time_msec, _current_value_0, _current_setPoint_0, _current_error_0, _current_control_0 );
                        //qDebug() << tr("Time: %1").arg( _curr_time_msec );
                        break;
                    case 1:
                        motor1 = first.message.motor;

                        _current_value_1 = ((double)motor1.velocity)/1000.0;
                        _current_error_1 = _current_setPoint_1-_current_value_1;
                        _current_control_1 = ((double)motor1.volt)/1000.0;

                        ui->widget_motor_1->setStatus( _curr_time_msec, _current_value_1, _current_setPoint_1, _current_error_1, _current_control_1 );

                        break;
                    }
                    break;
                case MOTOR_VEL_REF:
                    switch(command.bitset.motor) {
                    case 0:
                        ref0 = first.message.motor_control;
                        _current_setPoint_0 = ((double)ref0)/1000.0;
                        break;
                    case 1:
                        ref1 = first.message.motor_control;
                        _current_setPoint_1 = ((double)ref1)/1000.0;
                        break;
                    }
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
        motor_parameter_t param;
        param.enable_set = enable_mode;
//        param.k_ang = k_ang; ///< TODO
//        param.k_vel = k_vel; ///< TODO
        param.rotation = versus;

        motor_command_map_t motor_command;
        motor_command.bitset.motor = motIdx;
        motor_command.bitset.command = MOTOR_PARAMETER;

        _uNav->parserSendPacket(_uNav->createDataPacket(motor_command.command_message, HASHMAP_MOTOR, (message_abstract_u*) & param), 3, boost::posix_time::millisec(200));
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
