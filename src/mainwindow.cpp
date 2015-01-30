#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "robotparamscalculatedialog.h"

#include <QtSerialPort/QSerialPortInfo>
#include <QList>
#include <QDebug>
#include <QMessageBox>
#include <string>
#include <boost/asio.hpp>

using namespace std;

#define DEFAULT_TIME_RANGE 10.0
#define DEFAULT_VALS_RANGE 10.0


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _uNav(NULL)
{
    ui->setupUi(this);

    updateSerialPortList();
    initPlots();

    _updateTimeMsec = 10;

    _graphRange0=DEFAULT_TIME_RANGE;
    _graphRange1=DEFAULT_TIME_RANGE;

    connect( &_setPointUpdateTimer, SIGNAL(timeout()),
             this, SLOT(onSetPointUpdateTimerTimeout()) );

    //_connected = false;

    _current_value0=0.0;
    _current_value1=0.0;
    _current_setPoint0=0.0;
    _current_setPoint1=0.0;
    _current_error0=0.0;
    _current_error1=0.0;
    _current_control0=0.0;
    _current_control1=0.0;

    ui->lcdNumber_value_0->display( tr("%1").arg(_current_value0, 9,'f', 3) );
    ui->lcdNumber_setpoint_0->display( tr("%1").arg(_current_setPoint0, 9,'f', 3) );
    ui->lcdNumber_error_0->display( tr("%1").arg(_current_error0, 9,'f', 3) );

    ui->lcdNumber_value_1->display( tr("%1").arg(_current_value1, 9,'f', 3) );
    ui->lcdNumber_setpoint_1->display( tr("%1").arg(_current_setPoint1, 9,'f', 3) );
    ui->lcdNumber_error_1->display( tr("%1").arg(_current_error0, 1,'f', 3) );

    _enablePolarity = ui->checkBox_enable_mode->isChecked()?1:0;

    connect(ui->widget_plot_data_0->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_error_0->xAxis, SLOT(setRange(QCPRange)) );
    connect(ui->widget_plot_data_1->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_error_1->xAxis, SLOT(setRange(QCPRange)) );
    connect(ui->widget_plot_error_0->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_data_0->xAxis, SLOT(setRange(QCPRange)) );
    connect(ui->widget_plot_error_1->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_data_1->xAxis, SLOT(setRange(QCPRange)) );

    connect(ui->widget_plot_error_0->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_error_0->xAxis2, SLOT(setRange(QCPRange)) );
    connect(ui->widget_plot_error_1->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_error_1->xAxis2, SLOT(setRange(QCPRange)) );

    connect(ui->widget_plot_data_0->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_data_0->xAxis2, SLOT(setRange(QCPRange)) );
    connect(ui->widget_plot_data_1->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_data_1->xAxis2, SLOT(setRange(QCPRange)) );
    connect(ui->widget_plot_data_0->yAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_data_0->yAxis2, SLOT(setRange(QCPRange)) );
    connect(ui->widget_plot_data_1->yAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_data_1->yAxis2, SLOT(setRange(QCPRange)) );
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

void MainWindow::initPlots()
{
    QPen valPen(Qt::darkGreen);
    QPen spPen(Qt::darkBlue);
    QPen errPen(Qt::darkRed);
    QPen ctrlPen(Qt::darkCyan);

    // >>>>> Motor 0
    _graphRange0 = DEFAULT_TIME_RANGE;

    ui->widget_plot_data_0->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_data_0->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_data_0->legend->setVisible( true );
    ui->widget_plot_data_0->legend->setFont( QFont("Arial",8) );
    ui->widget_plot_data_0->addGraph();
    ui->widget_plot_data_0->addGraph();
    ui->widget_plot_data_0->graph(0)->setName( tr("Value") );
    ui->widget_plot_data_0->graph(0)->setPen( valPen );
    ui->widget_plot_data_0->graph(1)->setName( tr("SetPoint") );
    ui->widget_plot_data_0->graph(1)->setPen( spPen );
    ui->widget_plot_data_0->xAxis->setLabel( tr("time (msec)") );
    ui->widget_plot_data_0->yAxis->setLabel( tr("values (rad)") );
    ui->widget_plot_data_0->yAxis2->setLabel( tr("values (rad)") );
    ui->widget_plot_data_0->xAxis->setRange( 0, _graphRange0 );
    ui->widget_plot_data_0->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );

    ui->widget_plot_error_0->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error_0->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error_0->legend->setVisible( true );
    ui->widget_plot_error_0->legend->setFont( QFont("Arial",8) );
    ui->widget_plot_error_0->addGraph();
    ui->widget_plot_error_0->addGraph(ui->widget_plot_error_0->xAxis2, ui->widget_plot_error_0->yAxis2);
    ui->widget_plot_error_0->graph(0)->setName( tr("error") );
    ui->widget_plot_error_0->graph(0)->setPen( errPen );
    ui->widget_plot_error_0->graph(1)->setName( tr("control") );
    ui->widget_plot_error_0->graph(1)->setPen( ctrlPen );
    ui->widget_plot_error_0->xAxis->setLabel( tr("time (msec)") );
    ui->widget_plot_error_0->yAxis->setLabel( tr("error (rad)") );
    ui->widget_plot_error_0->yAxis2->setLabel( tr("control (V*sec/rad)") );
    ui->widget_plot_error_0->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );
    ui->widget_plot_error_0->yAxis2->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );

    ui->widget_plot_data_0->yAxis2->setVisible(true);
    ui->widget_plot_error_0->yAxis2->setVisible(true);
    // <<<<< Motor 0

    // >>>>> Motor 1
    _graphRange1 = DEFAULT_TIME_RANGE;

    ui->widget_plot_data_1->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_data_1->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_data_1->legend->setVisible( true );
    ui->widget_plot_data_1->legend->setFont( QFont("Arial",8) );
    ui->widget_plot_data_1->addGraph();
    ui->widget_plot_data_1->addGraph();
    ui->widget_plot_data_1->graph(0)->setName( tr("Value") );
    ui->widget_plot_data_1->graph(0)->setPen( valPen );
    ui->widget_plot_data_1->graph(1)->setName( tr("SetPoint") );
    ui->widget_plot_data_1->graph(1)->setPen( spPen );
    ui->widget_plot_data_1->xAxis->setLabel( tr("time (msec)") );
    ui->widget_plot_data_1->yAxis->setLabel( tr("values (rad)") );
    ui->widget_plot_data_1->yAxis2->setLabel( tr("values (rad)") );
    ui->widget_plot_data_1->xAxis->setRange( 0, _graphRange1 );
    ui->widget_plot_data_1->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );

    ui->widget_plot_error_1->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error_1->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error_1->legend->setVisible( true );
    ui->widget_plot_error_1->legend->setFont( QFont("Arial",8) );
    ui->widget_plot_error_1->addGraph();
    ui->widget_plot_error_1->addGraph(ui->widget_plot_error_1->xAxis2, ui->widget_plot_error_1->yAxis2);
    ui->widget_plot_error_1->graph(0)->setName( tr("error") );
    ui->widget_plot_error_1->graph(0)->setPen( errPen );
    ui->widget_plot_error_1->graph(1)->setName( tr("control") );
    ui->widget_plot_error_1->graph(1)->setPen( ctrlPen );
    ui->widget_plot_error_1->xAxis->setLabel( tr("time (msec)") );
    ui->widget_plot_error_1->yAxis->setLabel( tr("error (rad)") );
    ui->widget_plot_error_1->yAxis2->setLabel( tr("control (V*sec/rad)") );
    ui->widget_plot_error_1->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );
    ui->widget_plot_error_1->yAxis2->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );

    ui->widget_plot_data_1->yAxis2->setVisible(true);
    ui->widget_plot_error_1->yAxis2->setVisible(true);
    // <<<<< Motor 1
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
        _setPointUpdateTimer.stop();

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

    ui->groupBox_motor_0->setEnabled( false );
    ui->groupBox_motor_1->setEnabled( false );
    ui->groupBox_controls->setEnabled( false );

    //_connected = false;

    ui->widget_plot_data_0->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_data_0->setInteraction( QCP::iRangeZoom, true );
    ui->widget_plot_error_0->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_error_0->setInteraction( QCP::iRangeZoom, true );

    ui->widget_plot_data_1->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_data_1->setInteraction( QCP::iRangeZoom, true );
    ui->widget_plot_error_1->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_error_1->setInteraction( QCP::iRangeZoom, true );
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

        if( !sendParams( 0 ) )
            return false;
        if( !sendParams( 1 ) )
            return false;

        if( ui->checkBox_enable_0->isChecked() )
        {
            if( !sendEnable(0, true ) )
                return false;
        }
        else
        {
            if( !sendEnable(0, false ) )
                return false;
        }

        if( ui->checkBox_enable_1->isChecked() )
        {
            if( !sendEnable(1, true ) )
                return false;
        }
        else
        {
            if( !sendEnable(1, false ) )
                return false;
        }

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

    ui->groupBox_motor_0->setEnabled( true );
    ui->groupBox_motor_1->setEnabled( true );
    ui->groupBox_controls->setEnabled( true );
    ui->pushButton_start_motors->setEnabled( true );

    //_connected = true;


    return true;
}

bool MainWindow::stopMotors()
{
    //if( !_connected )
    //    return false;

    try
    {
        motor_control_t motor_ref = (int16_t) 0; //Convert in millirad/s

        std::vector<information_packet_t> list_send;

        list_send.push_back( _uNav->createDataPacket(VEL_MOTOR_L, HASHMAP_MOTION, (abstract_message_u*) & motor_ref) );
        list_send.push_back( _uNav->createDataPacket(VEL_MOTOR_R, HASHMAP_MOTION, (abstract_message_u*) & motor_ref) );

        _uNav->parserSendPacket( list_send, 3, boost::posix_time::millisec(200));
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

bool MainWindow::sendSetpoint0( double setPoint )
{
    //if( !_connected )
    //    return false;

    try
    {
        motor_control_t motor_ref = (int16_t) (setPoint*1000.0); //Convert in centirad/s
        _uNav->parserSendPacket(_uNav->createDataPacket(VEL_MOTOR_L, HASHMAP_MOTION, (abstract_message_u*) & motor_ref), 3, boost::posix_time::millisec(200));
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

bool MainWindow::sendSetpoint1( double setPoint )
{
    //if( !_connected )
    //    return false;

    try
    {
        motor_control_t motor_ref = (int16_t) (setPoint*1000.0); //Convert in millirad/s
        _uNav->parserSendPacket(_uNav->createDataPacket(VEL_MOTOR_R, HASHMAP_MOTION, (abstract_message_u*) & motor_ref), 3, boost::posix_time::millisec(200));
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

bool MainWindow::sendPIDGains0(float kp, float ki, float kd )
{
    //if( !_connected )
    //    return false;

    try
    {
        pid_control_t pid;
        pid.kp = kp;
        pid.ki = ki;
        pid.kd = kd;

        _uNav->parserSendPacket(_uNav->createDataPacket(PID_CONTROL_L, HASHMAP_MOTION, (abstract_message_u*) & pid), 3, boost::posix_time::millisec(200));
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

bool MainWindow::sendPIDGains1(float kp, float ki, float kd )
{
    //if( !_connected )
    //    return false;

    try
    {
        pid_control_t pid;
        pid.kp = kp;
        pid.ki = ki;
        pid.kd = kd;

        _uNav->parserSendPacket(_uNav->createDataPacket(PID_CONTROL_R, HASHMAP_MOTION, (abstract_message_u*) & pid), 3, boost::posix_time::millisec(200));
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

bool MainWindow::sendParams( int motIdx )
{
    bool ok;

    float k_vel = ui->lineEdit_k_vel->text().toFloat( &ok );

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_vel.\r\nRemember that decimal values use dot as separator.") );
        return false;
    }

    float k_ang = ui->lineEdit_k_ang->text().toFloat( &ok );

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_ang.\r\nRemember that decimal values use dot as separator.") );
        return false;
    }

    int8_t versus = 1;

    if(motIdx==0)
        versus = ui->checkBox_invert_mot_0->isChecked()?-1:1;
    else
        versus = ui->checkBox_invert_mot_1->isChecked()?-1:1;

    _enablePolarity = ui->checkBox_enable_mode->isChecked()?1:0;

    return sendMotorParams( motIdx, k_vel, k_ang, versus, _enablePolarity );
}

bool MainWindow::requestPidGains( int motIdx )
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

                    ui->doubleSpinBox_kp_0->setValue( pid0.kp);
                    ui->doubleSpinBox_ki_0->setValue( pid0.ki);
                    ui->doubleSpinBox_kd_0->setValue( pid0.kd);

                    break;
                case PID_CONTROL_R:
                    pid1 = first.packet.pid;

                    ui->doubleSpinBox_kp_1->setValue( pid1.kp);
                    ui->doubleSpinBox_ki_1->setValue( pid1.ki);
                    ui->doubleSpinBox_kd_1->setValue( pid1.kd);
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

bool MainWindow::requestStatus(int motIdx)
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

                    _current_value0 = ((double)motor0.measure_vel)/1000.0;
                    _current_setPoint0 = ((double)motor0.refer_vel)/1000.0;
                    _current_error0 = _current_setPoint0-_current_value0;
                    _current_control0 = ((double)motor0.control_vel)/1000.0;

                    break;
                case MOTOR_R:
                    motor1 = first.packet.motor;

                    _current_value1 = ((double)motor1.measure_vel)/1000.0;
                    _current_setPoint1 = ((double)motor1.refer_vel)/1000.0;
                    _current_error1 = _current_setPoint1-_current_value1;
                    _current_control1 = ((double)motor1.control_vel)/1000.0;

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

bool MainWindow::sendMotorParams(int motIdx, float k_vel, float k_ang,
                                 int8_t versus, uint8_t enable_mode )
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

void MainWindow::on_pushButton_send_gains_0_clicked()
{
    bool ok;

    double kp = ui->doubleSpinBox_kp_0->text().toDouble(&ok);

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_p.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    double ki = ui->doubleSpinBox_ki_0->text().toDouble(&ok);

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_i.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    double kd = ui->doubleSpinBox_kd_0->text().toDouble(&ok);

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_d.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    if( (kp+ki+kd) > 1 )
    {
        QMessageBox::warning( this, tr("Incorrect values" ),
                              tr("You should respect the relation\r\nKp + Ki + Kd < 1") );
        return;
    }

    if( (kp + 2*kd) > 1 )
    {
        QMessageBox::warning( this, tr("Incorrect values" ),
                              tr("You should respect the relation\r\nKp + 2*Kd < 1") );
        return;
    }

    sendPIDGains0( kp, ki, kd );
}

void MainWindow::on_pushButton_get_gains_0_clicked()
{
    requestPidGains(0);
}

void MainWindow::on_pushButton_get_gains_1_clicked()
{
    requestPidGains(1);
}

void MainWindow::on_pushButton_send_gains_1_clicked()
{
    bool ok;

    double kp = ui->doubleSpinBox_kp_1->text().toDouble(&ok);

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_p.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    double ki = ui->doubleSpinBox_ki_1->text().toDouble(&ok);

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_i.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    double kd = ui->doubleSpinBox_kd_1->text().toDouble(&ok);

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_d.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    sendPIDGains1( kp, ki, kd );
}

void MainWindow::onSetPointUpdateTimerTimeout()
{
    int elapsed = _timer.elapsed();
    _timer.restart();

    _curr_time_msec += elapsed;

    double setPoint;

    if( ui->radioButton_setpoint_fixed->isChecked() )
    {
        setPoint = _setPoint_fixed;
    }
    else if( ui->radioButton_setpoint_dynamic->isChecked() )
    {
        double cycleTime = (double)(_curr_time_msec%((quint64)_tot_dyn_cycle_msec));

        if( cycleTime > _t_raise_msec + _t_up_msec + _t_fall_msec ) // T_down phase
        {
            setPoint = _setPoint_dyn_down;

            //qDebug() << tr( "T_down --- cycleTime: %1 msec - setpoint: %2").arg(cycleTime).arg(setPoint);
        }
        else if( cycleTime > _t_raise_msec + _t_up_msec ) // T_fall phase
        {
            double y0 = _setPoint_dyn_up;
            double y1 = _setPoint_dyn_down;
            setPoint = y0 + (y1-y0)*((double)cycleTime-((double)(_t_raise_msec + _t_up_msec)) )/((double)_t_fall_msec);

            //qDebug() << tr( "T_fall --- cycleTime: %1 msec - setpoint: %2").arg(cycleTime).arg(setPoint);
        }
        else if( cycleTime > _t_raise_msec ) // T_up phase
        {
            setPoint = _setPoint_dyn_up;

            //qDebug() << tr( "T_up --- cycleTime: %1 msec - setpoint: %2").arg(cycleTime).arg(setPoint);
        }
        else // T_raise
        {
            double y0 = _setPoint_dyn_down;
            double y1 = _setPoint_dyn_up;
            setPoint = y0 + (y1-y0)*((double)cycleTime )/((double)_t_raise_msec);

            //qDebug() << tr( "T_raise --- cycleTime: %1 msec - setpoint: %2").arg(cycleTime).arg(setPoint);
        }
    }

    try
    {

        if( ui->checkBox_enable_0->isChecked() )
        {
            if(!sendSetpoint0( setPoint ) )
                return;

            if( !requestStatus( 0 ) )
                return;

            _setPointVec0 << _current_setPoint0;
            _timeVec0 << _curr_time_msec;
            _currMotorValVec0 << _current_value0;
            _errorVec0 << _current_error0;
            _controlVec0 << _current_control0;
            _controlVec1 << _current_control1;

            ui->lcdNumber_value_0->display( tr("%1").arg(_current_value0,9,'f', 3) );
            ui->lcdNumber_setpoint_0->display( tr("%1").arg(_current_setPoint0,9,'f', 3) );
            ui->lcdNumber_error_0->display( tr("%1").arg(_current_error0,9,'f', 3) );

            updatePlots0();
        }

        if( ui->checkBox_enable_1->isChecked() )
        {
            if(!sendSetpoint1( setPoint ) )
                return;

            if( !requestStatus( 1 ) )
                return;

            _setPointVec1 << _current_setPoint1;
            _timeVec1 << _curr_time_msec;
            _currMotorValVec1 << _current_value1;
            _errorVec1 << _current_error1;

            ui->lcdNumber_value_1->display( tr("%1").arg(_current_value1,9,'f', 3) );
            ui->lcdNumber_setpoint_1->display( tr("%1").arg(_current_setPoint1,9,'f', 3) );
            ui->lcdNumber_error_1->display( tr("%1").arg(_current_error1,9,'f', 3) );

            updatePlots1();
        }
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
    }
}

void MainWindow::on_pushButton_set_dynamic_setpoint_clicked()
{
    bool ok;
    _t_raise_msec = ui->doubleSpinBox_t_raise->text().toDouble( &ok )*1000.0;

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for T_raise.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    _t_up_msec = ui->doubleSpinBox_t_up->text().toDouble( &ok )*1000.0;

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for T_up.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    _t_fall_msec = ui->doubleSpinBox_t_fall->text().toDouble( &ok )*1000.0;

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for T_fall.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    _t_down_msec = ui->doubleSpinBox_t_down->text().toDouble( &ok )*1000.0;

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for T_down.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    _setPoint_dyn_up = ui->doubleSpinBox_val_up->text().toDouble( &ok );

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for Val_up.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    _setPoint_dyn_down = ui->doubleSpinBox_val_down->text().toDouble( &ok );

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for Val_down.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    _tot_dyn_cycle_msec = _t_raise_msec + _t_up_msec + _t_fall_msec + _t_down_msec;

    _current_value0 = 0.0;
    _current_value1 = 0.0;

    ui->widget_plot_data_0->yAxis->setRange( _setPoint_dyn_down*1.1, _setPoint_dyn_up*1.1 );
    ui->widget_plot_data_1->yAxis->setRange( _setPoint_dyn_down*1.1, _setPoint_dyn_up*1.1 );
}


void MainWindow::on_pushButton_set_fixed_setpoint_clicked()
{
    bool ok;
    _setPoint_fixed = ui->doubleSpinBox_fixed_setpoint->text().toDouble( &ok );

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for SetPoint.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    _current_value0 = 0.0;
    _current_value1 = 0.0;
}

void MainWindow::updatePlots0()
{
    if( _timeVec0.isEmpty() )
    {
        ui->widget_plot_data_0->graph(0)->clearData();
        ui->widget_plot_data_0->graph(1)->clearData();
        ui->widget_plot_error_0->graph(0)->clearData();

        _graphRange0 = DEFAULT_TIME_RANGE;
        ui->widget_plot_data_0->xAxis->setRange(0, _graphRange0 );
        ui->widget_plot_error_0->xAxis->setRange(0, _graphRange0 );

        ui->widget_plot_data_0->yAxis->setRange(-DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );
        ui->widget_plot_error_0->yAxis->setRange(-DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );


        ui->widget_plot_data_0->replot();
        ui->widget_plot_error_0->replot();
        return;
    }

    qreal time = (qreal)(_timeVec0.last()-_time_bias)/1000.0;
    qreal setPoint = _setPointVec0.last();
    qreal motorVal = _currMotorValVec0.last();
    qreal error = _errorVec0.last();
    qreal control = _controlVec0.last();

    ui->widget_plot_data_0->graph(0)->addData( time, motorVal );
    ui->widget_plot_data_0->graph(1)->addData( time, setPoint );
    ui->widget_plot_error_0->graph(0)->addData( time, error );
    ui->widget_plot_error_0->graph(1)->addData( time, control );

    ui->widget_plot_data_0->graph(0)->removeDataBefore( time-_graphRange0 );
    ui->widget_plot_data_0->graph(1)->removeDataBefore( time-_graphRange0 );
    ui->widget_plot_error_0->graph(0)->removeDataBefore( time-_graphRange0 );
    ui->widget_plot_error_0->graph(1)->removeDataBefore( time-_graphRange0 );

    if( error < _min_err_0 )
        _min_err_0 = error*1.2;
    if( error > _max_err_0 )
        _max_err_0 = error*1.2;

    if( motorVal < _min_val_0 )
        _min_val_0 = motorVal*1.2;
    if( motorVal > _max_val_0 )
        _max_val_0 = motorVal*1.2;

    if( control < _min_ctrl_0 )
        _min_ctrl_0 = control*1.2;
    if( control > _max_ctrl_0 )
        _max_ctrl_0 = control*1.2;

    ui->widget_plot_data_0->yAxis->setRange( _min_val_0, _max_val_0 );
    ui->widget_plot_error_0->yAxis->setRange( _min_err_0, _max_err_0 );
    ui->widget_plot_error_0->yAxis2->setRange( _min_ctrl_0, _max_ctrl_0);

    ui->widget_plot_data_0->xAxis->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange0, Qt::AlignRight);
    ui->widget_plot_error_0->xAxis->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange0, Qt::AlignRight);
    ui->widget_plot_error_0->xAxis2->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange0, Qt::AlignRight);

    ui->widget_plot_data_0->replot();
    ui->widget_plot_error_0->replot();
}

void MainWindow::updatePlots1()
{
    if( _timeVec1.isEmpty() )
    {
        ui->widget_plot_data_1->graph(0)->clearData();
        ui->widget_plot_data_1->graph(1)->clearData();
        ui->widget_plot_error_1->graph(0)->clearData();

        _graphRange1 = DEFAULT_TIME_RANGE;
        ui->widget_plot_data_1->xAxis->setRange(0, _graphRange1 );
        ui->widget_plot_error_1->xAxis->setRange(0, _graphRange1 );

        ui->widget_plot_data_1->yAxis->setRange(-DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );
        ui->widget_plot_error_1->yAxis->setRange(-DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );


        ui->widget_plot_data_1->replot();
        ui->widget_plot_error_1->replot();
        return;
    }

    qreal time = (qreal)(_timeVec1.last()-_time_bias)/1000.0;
    qreal setPoint = _setPointVec1.last();
    qreal motorVal = _currMotorValVec1.last();
    qreal error = _errorVec1.last();
    qreal control = _controlVec1.last();

    ui->widget_plot_data_1->graph(0)->addData( time, motorVal );
    ui->widget_plot_data_1->graph(1)->addData( time, setPoint);
    ui->widget_plot_error_1->graph(0)->addData( time, error );
    ui->widget_plot_error_1->graph(1)->addData( time, control);

    ui->widget_plot_data_1->graph(0)->removeDataBefore( time-_graphRange1 );
    ui->widget_plot_data_1->graph(1)->removeDataBefore( time-_graphRange1 );
    ui->widget_plot_error_1->graph(0)->removeDataBefore( time-_graphRange1 );
    ui->widget_plot_error_1->graph(1)->removeDataBefore( time-_graphRange1 );

    if( error < _min_err_1 )
        _min_err_1 = error*1.2;
    if( error > _max_err_1 )
        _max_err_1 = error*1.2;

    if( motorVal < _min_val_1 )
        _min_val_1 = motorVal*1.2;
    if( motorVal > _max_val_1 )
        _max_val_1 = motorVal*1.2;

    if( control < _min_ctrl_1 )
        _min_ctrl_1 = control*1.2;
    if( control > _max_ctrl_1 )
        _max_ctrl_1 = control*1.2;

    ui->widget_plot_data_1->yAxis->setRange( _min_val_1, _max_val_1 );
    ui->widget_plot_error_1->yAxis->setRange( _min_err_1, _max_err_1 );
    ui->widget_plot_error_1->yAxis2->setRange( _min_ctrl_1, _max_ctrl_1);

    ui->widget_plot_data_1->xAxis->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange0, Qt::AlignRight);
    ui->widget_plot_error_1->xAxis->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange0, Qt::AlignRight);
    ui->widget_plot_error_1->xAxis2->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange0, Qt::AlignRight);


    ui->widget_plot_data_1->xAxis->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange1, Qt::AlignRight);
    ui->widget_plot_error_1->xAxis->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange1, Qt::AlignRight);

    ui->widget_plot_data_1->replot();
    ui->widget_plot_error_1->replot();
}

void MainWindow::on_pushButton_reset_zoom_clicked()
{
    _min_err_0 = 0;
    _max_err_0 = 0;
    _min_val_0 = 0;
    _max_val_0 = 0;
    _min_ctrl_0 = 0;
    _max_ctrl_0 = 0;

    _min_err_1 = 0;
    _max_err_1 = 0;
    _min_val_1 = 0;
    _max_val_1 = 0;
    _min_ctrl_1 = 0;
    _max_ctrl_1 = 0;

    updatePlots0();
    updatePlots1();
}

void MainWindow::on_pushButton_reset_clicked()
{
    _timeVec0.clear();
    _setPointVec0.clear();
    _currMotorValVec0.clear();
    _errorVec0.clear();
    _controlVec0.clear();

    _timeVec1.clear();
    _setPointVec1.clear();
    _currMotorValVec1.clear();
    _errorVec1.clear();
    _controlVec1.clear();

    _min_err_0 = 0;
    _max_err_0 = 0;
    _min_val_0 = 0;
    _max_val_0 = 0;
    _min_ctrl_0 = 0;
    _max_ctrl_0 = 0;

    _min_err_1 = 0;
    _max_err_1 = 0;
    _min_val_1 = 0;
    _max_val_1 = 0;
    _min_ctrl_1 = 0;
    _max_ctrl_1 = 0;

    updatePlots0();
    updatePlots1();
}

void MainWindow::on_pushButton_stop_motors_clicked()
{
    _setPointUpdateTimer.stop();

    stopMotors();

    ui->pushButton_start_motors->setEnabled(true);
    ui->pushButton_stop_motors->setEnabled(false);

    ui->widget_plot_data_0->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_data_0->setInteraction( QCP::iRangeZoom, true );
    ui->widget_plot_error_0->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_error_0->setInteraction( QCP::iRangeZoom, true );

    ui->widget_plot_data_1->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_data_1->setInteraction( QCP::iRangeZoom, true );
    ui->widget_plot_error_1->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_error_1->setInteraction( QCP::iRangeZoom, true );
}

void MainWindow::on_pushButton_start_motors_clicked()
{
    // >>>>> Update traiectory params
    if(ui->radioButton_setpoint_dynamic->isChecked())
        on_pushButton_set_dynamic_setpoint_clicked();
    else
        on_pushButton_set_fixed_setpoint_clicked();
    // <<<<< Update traiectory params

    // >>>>> Bias calculation to avoid "Start step"
    double y0 = _setPoint_dyn_down;
    double y1 = _setPoint_dyn_up;
    _time_bias = (-y0*_t_raise_msec)/(y1-y0);
    _curr_time_msec = _time_bias;
    // <<<<< Bias calculation to avoid "Start step"


    _setPointUpdateTimer.setTimerType( Qt::PreciseTimer );



    ui->pushButton_start_motors->setEnabled(false);
    ui->pushButton_stop_motors->setEnabled(true);

    ui->widget_plot_data_0->setInteraction( QCP::iRangeDrag, false );
    ui->widget_plot_data_0->setInteraction( QCP::iRangeZoom, false );
    ui->widget_plot_error_0->setInteraction( QCP::iRangeDrag, false );
    ui->widget_plot_error_0->setInteraction( QCP::iRangeZoom, false );

    ui->widget_plot_data_1->setInteraction( QCP::iRangeDrag, false );
    ui->widget_plot_data_1->setInteraction( QCP::iRangeZoom, false );
    ui->widget_plot_error_1->setInteraction( QCP::iRangeDrag, false );
    ui->widget_plot_error_1->setInteraction( QCP::iRangeZoom, false );

    _min_err_0 = 0;
    _max_err_0 = 0;
    _min_val_0 = 0;
    _max_val_0 = 0;
    _min_ctrl_0 = 0;
    _max_ctrl_0 = 0;

    _min_err_1 = 0;
    _max_err_1 = 0;
    _min_val_1 = 0;
    _max_val_1 = 0;
    _min_ctrl_1 = 0;
    _max_ctrl_1 = 0;

    _timer.start();
    _setPointUpdateTimer.start( _updateTimeMsec );
}

void MainWindow::on_checkBox_enable_0_clicked(bool checked)
{
    sendEnable(0,checked);
}

void MainWindow::on_checkBox_enable_1_clicked(bool checked)
{
    sendEnable(1,checked);
}

void MainWindow::on_checkBox_enable_mode_clicked()
{
    sendParams( 0 );
    sendParams( 1 );
}

void MainWindow::on_checkBox_invert_mot_0_clicked()
{
    sendParams( 0 );
}

void MainWindow::on_checkBox_invert_mot_1_clicked()
{
    sendParams( 1 );
}

void MainWindow::on_pushButton_send_params_clicked()
{
    sendParams( 0 );
    sendParams( 1 );
}

void MainWindow::on_pushButton_calculate_k_params_clicked()
{
    RobotParamsCalculateDialog dlg;

    int res = dlg.exec();

    if( res == QDialog::Accepted )
    {
        double k_ang=0.0,k_vel=0.0;

        if( dlg.getParams( k_ang, k_vel) )
        {
            ui->lineEdit_k_ang->setText( tr("%1").arg(k_ang));
            ui->lineEdit_k_vel->setText( tr("%1").arg(k_vel));
        }

    }
}

