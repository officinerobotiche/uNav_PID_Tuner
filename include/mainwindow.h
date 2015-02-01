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
    void disconnectSerial();
    void updatePlots0();
    void updatePlots1();

public slots:
    // >>>>> Motor commands
    bool startMotor(quint8 motorIdx);
    bool stopMotor(quint8 motorIdx);
    bool sendEnable(int motIdx, bool sendEnable=true);
    bool sendSetpoint( quint8 motorIdx, double setPoint );
    bool sendPIDGains( quint8 motorIdx, double kp, double ki, double kd );
    bool requestPidGains(quint8 motIdx );
    //bool sendParams( quint8 motIdx);
    bool sendMotorParams(quint8 motIdx, double k_vel, double k_ang, qint8 versus, quint8 enable_mode);
    bool requestStatus(quint8 motIdx);
    // <<<<< Motor commands

private slots:
    void on_pushButton_update_serial_list_clicked();
    void on_pushButton_connect_clicked(bool checked);
//    void on_pushButton_send_gains_1_clicked();
//    void on_pushButton_send_gains_0_clicked();
//    void on_pushButton_set_dynamic_setpoint_clicked();
//    void on_pushButton_set_fixed_setpoint_clicked();
    void on_pushButton_reset_clicked();
//    void on_pushButton_stop_motors_clicked();
//    void on_pushButton_start_motors_clicked();

//    void onSetPointUpdateTimerTimeout();

//    void on_checkBox_enable_0_clicked(bool checked);
//    void on_checkBox_enable_1_clicked(bool checked);

//    void on_checkBox_enable_mode_clicked();

//    void on_checkBox_invert_mot_0_clicked();
//    void on_checkBox_invert_mot_1_clicked();

//    void on_pushButton_send_params_clicked();

//    void on_pushButton_get_gains_0_clicked();
//    void on_pushButton_get_gains_1_clicked();

//    void on_pushButton_calculate_k_params_clicked();

    void on_pushButton_reset_zoom_clicked();

    void on_tabWidget_motors_destroyed();

private:
    Ui::MainWindow *ui;

    ParserPacket* _uNav;    

    //bool _connected;

    double _current_value_0;
    double _current_setPoint_0;
    double _current_error_0;
    double _current_control_0;

    double _current_value_1;
    double _current_setPoint_1;
    double _current_error_1;
    double _current_control_1;

    quint64 _curr_time_msec;

    QTime _timer;
};

#endif // MAINWINDOW_H
