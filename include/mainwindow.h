#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QVector>

#include <QMainWindow>
#include <QTimer>
#include <QTime>

#include <ParserPacket.h>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void updateSerialPortList();
    bool connectSerial();
    void initPlots();
    void updatePlots0();
    void updatePlots1();

    // >>>>> Motor commands
    bool stopMotors();
    bool sendEnable(bool sendEnable=true);
    bool sendSetpoint0( double setPoint );
    bool sendSetpoint1( double setPoint );
    // <<<<< Motor commands

private slots:
    void on_pushButton_update_serial_list_clicked();
    void on_pushButton_connect_clicked(bool checked);
    void on_pushButton_send_gains_1_clicked();
    void on_pushButton_send_gains_0_clicked();
    void on_pushButton_set_dynamic_setpoint_clicked();
    void on_pushButton_set_fixed_setpoint_clicked();
    void on_pushButton_reset_clicked();
    void on_pushButton_stop_motors_clicked();

    void onSetPointUpdateTimerTimeout();

private:
    Ui::MainWindow *ui;

    ParserPacket* _uNav;


    QVector<quint64> _timeVec0;
    QVector<quint64> _timeVec1;

    QVector<qreal> _setPointVec0;
    QVector<qreal> _currMotorValVec0;
    QVector<qreal> _errorVec0;

    QVector<qreal> _setPointVec1;
    QVector<qreal> _currMotorValVec1;
    QVector<qreal> _errorVec1;

    qreal _graphRange0;
    qreal _graphRange1;

    quint64 _updateTimeMsec;

    quint64 _curr_time_msec;
    double _t_raise_msec;
    double _t_up_msec;
    double _t_fall_msec;
    double _t_down_msec;
    double _tot_dyn_cycle_msec;

    double _setPoint_fixed;
    double _setPoint_dyn_up;
    double _setPoint_dyn_down;

    double _current_value0;
    double _current_value1;

    QTimer _setPointUpdateTimer;
    QTime _timer;

    bool _connected;
};

#endif // MAINWINDOW_H
