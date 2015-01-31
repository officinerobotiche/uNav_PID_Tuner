#include "qmotorpidparamwidget.h"
#include "ui_qmotorpidparamwidget.h"
#include "robotparamscalculatedialog.h"

QMotorPidParamWidget::QMotorPidParamWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::QMotorPidParamWidget)
{
    ui->setupUi(this);

    _updateTimeMsec = 10;

    _graphRange=DEFAULT_TIME_RANGE;

    initPlots();

    connect( &_setPointUpdateTimer, SIGNAL(timeout()),
             this, SLOT(onSetPointUpdateTimerTimeout()) );

    ui->lcdNumber_value->display( tr("%1").arg(0, 9,'f', 3) );
    ui->lcdNumber_setpoint->display( tr("%1").arg(0, 9,'f', 3) );
    ui->lcdNumber_error->display( tr("%1").arg(0, 9,'f', 3) );
    ui->lcdNumber_control->display( tr("%1").arg(0, 9,'f', 3) );

    _enablePolarity = ui->radioButton_polarity_low->isChecked()?0:1;

    connect(ui->widget_plot_data->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_error->xAxis, SLOT(setRange(QCPRange)) );
    connect(ui->widget_plot_error->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_data->xAxis, SLOT(setRange(QCPRange)) );

    connect(ui->widget_plot_error->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_error->xAxis2, SLOT(setRange(QCPRange)) );

    connect(ui->widget_plot_data->xAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_data->xAxis2, SLOT(setRange(QCPRange)) );
    connect(ui->widget_plot_data->yAxis, SIGNAL(rangeChanged(QCPRange)),
            ui->widget_plot_data->yAxis2, SLOT(setRange(QCPRange)) );
}

QMotorPidParamWidget::~QMotorPidParamWidget()
{
    delete ui;
}

void QMotorPidParamWidget::stopMotor()
{
    ui->pushButton_start_motors->setEnabled(true);

    _setPointUpdateTimer.stop();

    ui->widget_plot_data->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_data->setInteraction( QCP::iRangeZoom, true );
    ui->widget_plot_error->setInteraction( QCP::iRangeDrag, true );
    ui->widget_plot_error->setInteraction( QCP::iRangeZoom, true );
}

void QMotorPidParamWidget::enableControls(bool enable)
{
    ui->groupBox_motor->setEnabled( enable );
    ui->groupBox_controls->setEnabled( enable );
    ui->pushButton_start_motors->setEnabled( enable );
    ui->pushButton_stop_motors->setEnabled( enable );


    //_connected = false;

    ui->widget_plot_data->setInteraction( QCP::iRangeDrag, !enable );
    ui->widget_plot_data->setInteraction( QCP::iRangeZoom, !enable );
    ui->widget_plot_error->setInteraction( QCP::iRangeDrag, !enable );
    ui->widget_plot_error->setInteraction( QCP::iRangeZoom, !enable );
}

void QMotorPidParamWidget::on_pushButton_set_dynamic_setpoint_clicked()
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

    ui->widget_plot_data->yAxis->setRange( _setPoint_dyn_down*1.1, _setPoint_dyn_up*1.1 );
}

void QMotorPidParamWidget::initPlots()
{
    QPen valPen(Qt::darkGreen);
    QPen spPen(Qt::darkBlue);
    QPen errPen(Qt::darkRed);
    QPen ctrlPen(Qt::darkCyan);

    _graphRange = DEFAULT_TIME_RANGE;

    ui->widget_plot_data->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_data->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_data->legend->setVisible( true );
    ui->widget_plot_data->legend->setFont( QFont("Arial",8) );
    ui->widget_plot_data->addGraph();
    ui->widget_plot_data->addGraph();
    ui->widget_plot_data->graph(0)->setName( tr("Value") );
    ui->widget_plot_data->graph(0)->setPen( valPen );
    ui->widget_plot_data->graph(1)->setName( tr("SetPoint") );
    ui->widget_plot_data->graph(1)->setPen( spPen );
    ui->widget_plot_data->xAxis->setLabel( tr("time (msec)") );
    ui->widget_plot_data->yAxis->setLabel( tr("values (rad)") );
    ui->widget_plot_data->yAxis2->setLabel( tr("values (rad)") );
    ui->widget_plot_data->xAxis->setRange( 0, _graphRange );
    ui->widget_plot_data->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );

    ui->widget_plot_error->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
    ui->widget_plot_error->legend->setVisible( true );
    ui->widget_plot_error->legend->setFont( QFont("Arial",8) );
    ui->widget_plot_error->addGraph();
    ui->widget_plot_error->addGraph(ui->widget_plot_error->xAxis2, ui->widget_plot_error->yAxis2);
    ui->widget_plot_error->graph(0)->setName( tr("error") );
    ui->widget_plot_error->graph(0)->setPen( errPen );
    ui->widget_plot_error->graph(1)->setName( tr("control") );
    ui->widget_plot_error->graph(1)->setPen( ctrlPen );
    ui->widget_plot_error->xAxis->setLabel( tr("time (msec)") );
    ui->widget_plot_error->yAxis->setLabel( tr("error (rad)") );
    ui->widget_plot_error->yAxis2->setLabel( tr("control (V*sec/rad)") );
    ui->widget_plot_error->yAxis->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );
    ui->widget_plot_error->yAxis2->setRange( -DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );
    ui->widget_plot_error->xAxis->setRange( 0, _graphRange );

    ui->widget_plot_data->yAxis2->setVisible(true);
    ui->widget_plot_error->yAxis2->setVisible(true);
}

void QMotorPidParamWidget::sendParams( )
{
    bool ok;

    float k_vel = ui->lineEdit_k_vel->text().toFloat( &ok );

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_vel.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    float k_ang = ui->lineEdit_k_ang->text().toFloat( &ok );

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_ang.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    int8_t versus =  versus = ui->checkBox_invert_mot->isChecked()?-1:1;

    _enablePolarity = ui->radioButton_polarity_high->isChecked()?1:0;

    emit newMotorConfig( _motorIdx, k_vel, k_ang, versus, _enablePolarity );
}

void QMotorPidParamWidget::setPidParams( double Kp, double Ki, double Kd )
{
    ui->doubleSpinBox_kp->setValue( Kp);
    ui->doubleSpinBox_ki->setValue( Ki);
    ui->doubleSpinBox_kd->setValue( Kd);
}

void QMotorPidParamWidget::setMotorConfig( double Kvel, double Kang, bool inverse, bool enablePol )
{
    ui->lineEdit_k_ang->setText( tr("%1").arg(Kang) );
    ui->lineEdit_k_vel->setText( tr("%1").arg(Kvel) );
    ui->checkBox_invert_mot->setChecked( inverse );
    ui->radioButton_polarity_low->setChecked( !enablePol );
    ui->radioButton_polarity_high->setChecked( enablePol );
}

void QMotorPidParamWidget::setStatus( quint64 time, double measure, double setPoint, double error , double control)
{
    ui->lcdNumber_value->display( measure );
    ui->lcdNumber_setpoint->display( setPoint );
    ui->lcdNumber_error->display( error );
    ui->lcdNumber_control->display( control );

    _curr_time_msec = time;

    _timeVec << _curr_time_msec;
    _setPointVec << setPoint;
    _currMotorValVec << measure;
    _errorVec << error;
    _controlVec << control;

    //qDebug() << tr("Time w: %1").arg( _curr_time_msec );

    updatePlots();
}

void QMotorPidParamWidget::onSetPointUpdateTimerTimeout()
{
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

    emit newSetPoint( _motorIdx, setPoint );
    emit reqStatus( _motorIdx );
}

void QMotorPidParamWidget::on_pushButton_send_gains_clicked()
{
    bool ok;

    double kp = ui->doubleSpinBox_kp->text().toDouble(&ok);

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_p.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    double ki = ui->doubleSpinBox_ki->text().toDouble(&ok);

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for K_i.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

    double kd = ui->doubleSpinBox_kd->text().toDouble(&ok);

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

    emit newPidParams( _motorIdx, kp, ki, kd );
}

void QMotorPidParamWidget::on_pushButton_set_fixed_setpoint_clicked()
{
    bool ok;
    _setPoint_fixed = ui->doubleSpinBox_fixed_setpoint->text().toDouble( &ok );

    if( !ok )
    {
        QMessageBox::warning( this, tr("Incorrect value"),
                              tr("Please insert a correct value for SetPoint.\r\nRemember that decimal values use dot as separator.") );
        return;
    }

}

void QMotorPidParamWidget::updatePlots()
{
    if( _timeVec.isEmpty() )
    {
        ui->widget_plot_data->graph(0)->clearData();
        ui->widget_plot_data->graph(1)->clearData();
        ui->widget_plot_error->graph(0)->clearData();
        ui->widget_plot_error->graph(1)->clearData();


        _graphRange = DEFAULT_TIME_RANGE;
        ui->widget_plot_data->xAxis->setRange(0, _graphRange );
        ui->widget_plot_error->xAxis->setRange(0, _graphRange );

        ui->widget_plot_data->yAxis->setRange(-DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );
        ui->widget_plot_error->yAxis->setRange(-DEFAULT_VALS_RANGE, DEFAULT_VALS_RANGE );


        ui->widget_plot_data->replot();
        ui->widget_plot_error->replot();
        return;
    }

    qreal time = (qreal)(_timeVec.last()-_time_bias)/1000.0;
    qreal setPoint = _setPointVec.last();
    qreal motorVal = _currMotorValVec.last();
    qreal error = _errorVec.last();
    qreal control = _controlVec.last();

    ui->widget_plot_data->graph(0)->addData( time, motorVal );
    ui->widget_plot_data->graph(1)->addData( time, setPoint );
    ui->widget_plot_error->graph(0)->addData( time, error );
    ui->widget_plot_error->graph(1)->addData( time, control );

    ui->widget_plot_data->graph(0)->removeDataBefore( time-_graphRange );
    ui->widget_plot_data->graph(1)->removeDataBefore( time-_graphRange );
    ui->widget_plot_error->graph(0)->removeDataBefore( time-_graphRange );
    ui->widget_plot_error->graph(1)->removeDataBefore( time-_graphRange );

    if( error < _min_err )
        _min_err = error*1.2;
    if( error > _max_err )
        _max_err = error*1.2;

    if( motorVal < _min_val )
        _min_val = motorVal*1.2;
    if( motorVal > _max_val )
        _max_val = motorVal*1.2;

    if( control < _min_ctrl )
        _min_ctrl = control*1.2;
    if( control > _max_ctrl )
        _max_ctrl = control*1.2;

    ui->widget_plot_data->yAxis->setRange( _min_val, _max_val );
    ui->widget_plot_error->yAxis->setRange( _min_err, _max_err );
    ui->widget_plot_error->yAxis2->setRange( _min_ctrl, _max_ctrl);

    ui->widget_plot_data->xAxis->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange, Qt::AlignRight);
    ui->widget_plot_error->xAxis->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange, Qt::AlignRight);
    ui->widget_plot_error->xAxis2->setRange( time+(((qreal)(_updateTimeMsec))/1000.0)*2, _graphRange, Qt::AlignRight);

    ui->widget_plot_data->replot();
    ui->widget_plot_error->replot();
}

void QMotorPidParamWidget::resetZoom()
{
    _min_err = 0;
    _max_err = 0;
    _min_val = 0;
    _max_val = 0;
    _min_ctrl = 0;
    _max_ctrl = 0;

    updatePlots();
}

void QMotorPidParamWidget::clearPlots()
{
    _timeVec.clear();
    _setPointVec.clear();
    _currMotorValVec.clear();
    _errorVec.clear();
    _controlVec.clear();

    _min_err = 0;
    _max_err = 0;
    _min_val = 0;
    _max_val = 0;
    _min_ctrl = 0;
    _max_ctrl = 0;

    updatePlots();
}

void QMotorPidParamWidget::on_pushButton_stop_motors_clicked()
{
    stopMotor();

    emit stopMotor( _motorIdx );
}

void QMotorPidParamWidget::on_pushButton_start_motors_clicked()
{
    on_pushButton_send_params_clicked();

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

    ui->widget_plot_data->setInteraction( QCP::iRangeDrag, false );
    ui->widget_plot_data->setInteraction( QCP::iRangeZoom, false );
    ui->widget_plot_error->setInteraction( QCP::iRangeDrag, false );
    ui->widget_plot_error->setInteraction( QCP::iRangeZoom, false );

    _min_err = 0;
    _max_err = 0;
    _min_val = 0;
    _max_val = 0;
    _min_ctrl = 0;
    _max_ctrl = 0;

    _setPointUpdateTimer.start( _updateTimeMsec );

    emit startMotor( _motorIdx );
}

void QMotorPidParamWidget::on_pushButton_calculate_k_params_clicked()
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

void QMotorPidParamWidget::on_pushButton_get_gains_clicked()
{
    emit reqPidParams( _motorIdx );
}

void QMotorPidParamWidget::on_pushButton_send_params_clicked()
{
    sendParams( );
}
