#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtSerialPort/QSerialPortInfo>
#include <QList>
#include <QDebug>
#include <QMessageBox>
#include <string>
#include <boost/asio.hpp>

using namespace std;

#define DEFAULT_TIME_RANGE 10000
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

    _connected = false;
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

    // >>>>> Time axis synchronization
    connect(ui->widget_plot_data_0->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_error_0->xAxis, SLOT(setRange(QCPRange)));
    connect(ui->widget_plot_data_1->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_error_1->xAxis, SLOT(setRange(QCPRange)));
    // <<<<< Time axis synchronization

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
    ui->widget_plot_data_0->yAxis->setLabel( tr("values (sec)") );
    ui->widget_plot_data_0->xAxis->setRange( 0, _graphRange0 );
    ui->widget_plot_data_0->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );

    ui->widget_plot_error_0->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error_0->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error_0->legend->setVisible( true );
    ui->widget_plot_error_0->legend->setFont( QFont("Arial",8) );
    ui->widget_plot_error_0->addGraph();
    ui->widget_plot_error_0->graph(0)->setName( tr("error") );
    ui->widget_plot_error_0->graph(0)->setPen( errPen );
    ui->widget_plot_error_0->xAxis->setLabel( tr("time (msec)") );
    ui->widget_plot_error_0->yAxis->setLabel( tr("error (sec)") );
    ui->widget_plot_error_0->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );
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
    ui->widget_plot_data_1->yAxis->setLabel( tr("values (sec)") );
    ui->widget_plot_data_1->xAxis->setRange( 0, _graphRange1 );
    ui->widget_plot_data_1->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );

    ui->widget_plot_error_1->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error_1->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error_1->legend->setVisible( true );
    ui->widget_plot_error_1->legend->setFont( QFont("Arial",8) );
    ui->widget_plot_error_1->addGraph();
    ui->widget_plot_error_1->graph(0)->setName( tr("error") );
    ui->widget_plot_error_1->graph(0)->setPen( errPen );
    ui->widget_plot_error_1->xAxis->setLabel( tr("time (msec)") );
    ui->widget_plot_error_1->yAxis->setLabel( tr("error (sec)") );
    ui->widget_plot_error_1->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );
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
    if( !_uNav || !_connected )
        return;

    try
    {
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

    _connected = false;
}

bool MainWindow::connectSerial()
{
    if( _uNav )
        delete _uNav;

    string serialPort = ui->comboBox_serial_port->currentText().toStdString();

    try
    {
        _uNav = new ParserPacket( serialPort, 115200 );
    }
    catch( parser_exception& e)
    {
        qDebug() << tr("Connection error: %1").arg(e.what());

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

    _connected = true;

    if( ui->checkBox_enable_0->isChecked() )
        sendEnable(0, true );
    else
        sendEnable(0, false );

    if( ui->checkBox_enable_1->isChecked() )
        sendEnable(1, true );
    else
        sendEnable(1, false );

    sendParams( 0 );
    sendParams( 1 );

    return true;
}

bool MainWindow::stopMotors()
{
    if( !_connected )
        return false;

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
    if( !_connected )
        return false;

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
    if( !_connected )
        return false;

    try
    {
        motor_control_t motor_ref = (int16_t) (setPoint*1000.0); //Convert in millirad/s
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
    if( !_connected )
        return false;

    try
    {
        motor_control_t motor_ref = (int16_t) (setPoint*1000); //Convert in millirad/s
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

bool MainWindow::sendPIDGains0(double kp, double ki, double kd )
{
    if( !_connected )
        return false;

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

bool MainWindow::sendPIDGains1(double kp, double ki, double kd )
{
    if( !_connected )
        return false;

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
    float k_vel = ui->lineEdit_k_vel->text().toFloat();
    float k_ang = ui->lineEdit_k_ang->text().toFloat();

    int8_t versus = 1;

    if(motIdx==0)
        versus = ui->checkBox_invert_mot_0->isChecked()?-1:1;
    else
        versus = ui->checkBox_invert_mot_1->isChecked()?-1:1;

    uint8_t enable_mode = ui->checkBox_enable_mode->isChecked()?1:0;

    return sendMotorParams( motIdx, k_vel, k_ang, versus, enable_mode );
}

bool MainWindow::requestStatus(int motIdx)
{
    if( !_connected )
        return false;

    try
    {
        motor_t motorStatus;

        quint8 command;
        if(motIdx==0)
            command = MOTOR_L;
        else
            command = MOTOR_R;

        information_packet_t send = _uNav->createPacket( MOTOR_L, REQUEST, HASHMAP_MOTION);
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

                    break;
                case MOTOR_R:
                    motor1 = first.packet.motor;

                    _current_value1 = ((double)motor1.measure_vel)/1000.0;
                    _current_setPoint1 = ((double)motor1.refer_vel)/1000.0;
                    _current_error1 = _current_setPoint1-_current_value1;

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
    if( !_connected )
        return false;

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
                QMessageBox::warning( this, tr("Connection error"), tr("Unknown error") );
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
    double kp = ui->doubleSpinBox_kp_0->text().toDouble();
    double ki = ui->doubleSpinBox_ki_0->text().toDouble();
    double kd = ui->doubleSpinBox_kd_0->text().toDouble();

    sendPIDGains0( kp, ki, kd );
}

void MainWindow::on_pushButton_send_gains_1_clicked()
{
    double kp = ui->doubleSpinBox_kp_1->text().toDouble();
    double ki = ui->doubleSpinBox_ki_1->text().toDouble();
    double kd = ui->doubleSpinBox_kd_1->text().toDouble();

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

            qDebug() << tr( "T_down --- cycleTime: %1 msec - setpoint: %2").arg(cycleTime).arg(setPoint);
        }
        else if( cycleTime > _t_raise_msec + _t_up_msec ) // T_fall phase
        {
            double y0 = _setPoint_dyn_up;
            double y1 = _setPoint_dyn_down;
            setPoint = y0 + (y1-y0)*((double)cycleTime-((double)(_t_raise_msec + _t_up_msec)) )/((double)_t_fall_msec);

            qDebug() << tr( "T_fall --- cycleTime: %1 msec - setpoint: %2").arg(cycleTime).arg(setPoint);
        }
        else if( cycleTime > _t_raise_msec ) // T_up phase
        {
            setPoint = _setPoint_dyn_up;

            qDebug() << tr( "T_up --- cycleTime: %1 msec - setpoint: %2").arg(cycleTime).arg(setPoint);
        }
        else // T_raise
        {
            double y0 = _setPoint_dyn_down;
            double y1 = _setPoint_dyn_up;
            setPoint = y0 + (y1-y0)*((double)cycleTime )/((double)_t_raise_msec);

            qDebug() << tr( "T_raise --- cycleTime: %1 msec - setpoint: %2").arg(cycleTime).arg(setPoint);
        }
    }

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

        ui->lcdNumber_value_0->display( _current_value0 );
        ui->lcdNumber_setpoint_0->display( _current_setPoint0 );
        ui->lcdNumber_error_0->display( _current_error0 );

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

        ui->lcdNumber_value_1->display( _current_value1 );
        ui->lcdNumber_setpoint_1->display( _current_setPoint1 );
        ui->lcdNumber_error_1->display( _current_error1 );

        updatePlots1();
    }
}

void MainWindow::on_pushButton_set_dynamic_setpoint_clicked()
{
    _setPointUpdateTimer.stop();

    _curr_time_msec = 0;
    _setPointUpdateTimer.setTimerType( Qt::PreciseTimer );

    _t_raise_msec = ui->lineEdit_t_raise->text().toDouble( )*1000.0;
    _t_up_msec = ui->lineEdit_t_up->text().toDouble( )*1000.0;
    _t_fall_msec = ui->lineEdit_t_fall->text().toDouble( )*1000.0;
    _t_down_msec = ui->lineEdit_t_down->text().toDouble( )*1000.0;

    _setPoint_dyn_up = ui->lineEdit_val_up->text().toDouble();
    _setPoint_dyn_down = ui->lineEdit_val_down->text().toDouble();

    _tot_dyn_cycle_msec = _t_raise_msec + _t_up_msec + _t_fall_msec + _t_down_msec;

    _current_value0 = 0.0;
    _current_value1 = 0.0;

    _timer.start();
    _setPointUpdateTimer.start( _updateTimeMsec );

    ui->widget_plot_data_0->yAxis->setRange( _setPoint_dyn_down*1.1, _setPoint_dyn_up*1.1 );
    ui->widget_plot_data_1->yAxis->setRange( _setPoint_dyn_down*1.1, _setPoint_dyn_up*1.1 );
}


void MainWindow::on_pushButton_set_fixed_setpoint_clicked()
{
    _setPointUpdateTimer.stop();

    _setPoint_fixed = ui->lineEdit_fixed_setpoint->text().toDouble();

    _current_value0 = 0.0;
    _current_value1 = 0.0;

    _timer.start();
    _setPointUpdateTimer.start( _updateTimeMsec );
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

    quint64 time = _timeVec0.last();
    qreal setPoint = _setPointVec0.last();
    qreal motorVal = _currMotorValVec0.last();
    qreal error = _errorVec0.last();

    ui->widget_plot_data_0->graph(0)->addData( (qreal)time, motorVal );
    ui->widget_plot_data_0->graph(1)->addData( (qreal)time, setPoint );
    ui->widget_plot_error_0->graph(0)->addData( (qreal)time, error );

    ui->widget_plot_data_0->graph(0)->removeDataBefore( (qreal)time-_graphRange0 );
    ui->widget_plot_data_0->graph(1)->removeDataBefore( (qreal)time-_graphRange0 );
    ui->widget_plot_error_0->graph(0)->removeDataBefore( (qreal)time-_graphRange0 );

    //ui->widget_plot_data_0->graph(0)->rescaleValueAxis(true);
    //ui->widget_plot_data_0->graph(1)->rescaleValueAxis(true);
    ui->widget_plot_error_0->graph(0)->rescaleValueAxis(true);

    ui->widget_plot_data_0->xAxis->setRange( time+_updateTimeMsec*2, _graphRange0, Qt::AlignRight);
    ui->widget_plot_error_0->xAxis->setRange( time+_updateTimeMsec*2, _graphRange0, Qt::AlignRight);

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

    quint64 time = _timeVec1.last();
    qreal setPoint = _setPointVec1.last();
    qreal motorVal = _currMotorValVec1.last();
    qreal error = _errorVec1.last();

    ui->widget_plot_data_1->graph(0)->addData( (qreal)time, motorVal );
    ui->widget_plot_data_1->graph(1)->addData( (qreal)time, setPoint);
    ui->widget_plot_error_1->graph(0)->addData( (qreal)time, error );

    ui->widget_plot_data_1->graph(0)->removeDataBefore( (double)time-_graphRange1 );
    ui->widget_plot_data_1->graph(1)->removeDataBefore( (double)time-_graphRange1 );
    ui->widget_plot_error_1->graph(0)->removeDataBefore( (double)time-_graphRange1 );

    //ui->widget_plot_data_1->graph(0)->rescaleValueAxis(true);
    //ui->widget_plot_data_1->graph(1)->rescaleValueAxis(true);
    ui->widget_plot_error_1->graph(0)->rescaleValueAxis(true);

    ui->widget_plot_data_1->xAxis->setRange( time+_updateTimeMsec*2, _graphRange1, Qt::AlignRight);
    ui->widget_plot_error_1->xAxis->setRange( time+_updateTimeMsec*2, _graphRange1, Qt::AlignRight);

    ui->widget_plot_data_1->replot();
    ui->widget_plot_error_1->replot();
}

void MainWindow::on_pushButton_reset_clicked()
{
    _timeVec0.clear();
    _setPointVec0.clear();
    _currMotorValVec0.clear();
    _errorVec0.clear();

    _timeVec1.clear();
    _setPointVec1.clear();
    _currMotorValVec1.clear();
    _errorVec1.clear();

    updatePlots0();
    updatePlots1();
}

void MainWindow::on_pushButton_stop_motors_clicked()
{
    _setPointUpdateTimer.stop();

    stopMotors();
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
