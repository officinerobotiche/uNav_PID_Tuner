#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ParserPacket.h>

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
    bool connect();

private slots:
    void on_pushButton_update_serial_list_clicked();

    void on_pushButton_connect_clicked(bool checked);

    void on_pushButton_send_gains_1_clicked();

    void on_pushButton_send_gains_0_clicked();

    void on_pushButton_set_dynamic_setpoint_clicked();

    void on_pushButton_set_fixed_setpoint_clicked();

private:
    Ui::MainWindow *ui;

    ParserPacket* _uNav;
};

#endif // MAINWINDOW_H
