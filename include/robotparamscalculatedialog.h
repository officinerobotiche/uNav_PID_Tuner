#ifndef ROBOTPARAMSCALCULATEDIALOG_H
#define ROBOTPARAMSCALCULATEDIALOG_H

#include <QDialog>

namespace Ui {
class RobotParamsCalculateDialog;
}

class RobotParamsCalculateDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RobotParamsCalculateDialog(QWidget *parent = 0);
    ~RobotParamsCalculateDialog();

    bool getParams(double &k_ang, double &k_vel );

private slots:
    void on_pushButton_clicked();

private:
    Ui::RobotParamsCalculateDialog *ui;

    double _k_ang;
    double _k_vel;
    bool _calculated;
};

#endif // ROBOTPARAMSCALCULATEDIALOG_H
