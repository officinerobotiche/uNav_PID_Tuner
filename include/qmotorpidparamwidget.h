#ifndef QMOTORPIDPARAMWIDGET_H
#define QMOTORPIDPARAMWIDGET_H

#include <QWidget>
#include <QTimer>

using namespace std;

#define DEFAULT_TIME_RANGE 10.0
#define DEFAULT_VALS_RANGE 10.0

namespace Ui {
class QMotorPidParamWidget;
}

class QMotorPidParamWidget : public QWidget
{
    Q_OBJECT

public:
    explicit QMotorPidParamWidget( QWidget *parent = 0);
    ~QMotorPidParamWidget();

    void setMotorIdx( quint8 motorIdx ){_motorIdx = motorIdx;}
    void stopMotor();
    void enableControls( bool enable );

protected slots:
    void on_pushButton_set_dynamic_setpoint_clicked();
    void on_pushButton_send_gains_clicked();
    void on_pushButton_set_fixed_setpoint_clicked();

    void onSetPointUpdateTimerTimeout();


public slots:
    void setPidParams(double Kp, double Ki, double Kd );
    void setMotorConfig( double Kvel, double Kang, bool inverse, bool enablePol ); // TODO Adapt to new configuration
    void setStatus(quint64 time, double measure, double setPoint, double error/*, double control=0 */);
    void resetZoom();
    void clearPlots();

signals:
    void newPidParams( quint8 motorIdx, double Kp, double Ki, double Kd );
    void newMotorConfig( quint8 motIdx, quint16 cpr, float ratio,
                         qint8 versus, quint8 enable_mode, quint8 enc_pos, qint16 bridge_volt );  // TODO Adapt to new configuration
    void startMotor( quint8 motorIdx );
    void stopMotor( quint8 motorIdx );
    void newSetPoint( quint8 motorIdx, double setPoint );
    void reqStatus( quint8 motorIdx );
    void reqPidParams( quint8 motorIdx );

private slots:
    void on_pushButton_stop_motors_clicked();
    void on_pushButton_start_motors_clicked();
    void on_pushButton_get_gains_clicked();

    void on_pushButton_send_params_clicked();

private:
    void initPlots();
    void sendParams();
    void updatePlots();


private:
    Ui::QMotorPidParamWidget *ui;

    quint8 _motorIdx;

    QVector<quint64> _timeVec;

    QVector<qreal> _setPointVec;
    QVector<qreal> _currMotorValVec;
    QVector<qreal> _errorVec;
    // QVector<qreal> _controlVec; for the future

    qreal _graphRange;

    quint64 _updateTimeMsec;

    quint64 _curr_time_msec;
    quint64 _time_bias;
    double _t_raise_msec;
    double _t_up_msec;
    double _t_fall_msec;
    double _t_down_msec;
    double _tot_dyn_cycle_msec;

    double _setPoint_fixed;
    double _setPoint_dyn_up;
    double _setPoint_dyn_down;

    double _min_err;
    double _max_err;
    double _min_val;
    double _max_val;
    // double _min_ctrl; for the future
    // double _max_ctrl; for the future


    quint8 _enablePolarity; ///< Polarity of the Enable signal

    QTimer _setPointUpdateTimer;
};

#endif // QMOTORPIDPARAMWIDGET_H
