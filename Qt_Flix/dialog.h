#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QSerialPort>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Dialog; }
QT_END_NAMESPACE

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = nullptr);
    ~Dialog();

private slots:
    void on_verticalSliderThrust_valueChanged(int value);

    void handleSerialData();
    void on_push_arm_clicked();
    void on_push_disarm_clicked();
    void on_lineEdit_Kpf_returnPressed();
    void on_lineEdit_Kpv_returnPressed();
    void on_lineEdit_Kdv_returnPressed();


    void on_checkBoxRoll_toggled(bool checked);

    void on_checkBoxPitch_toggled(bool checked);

    void on_checkBoxYaw_toggled(bool checked);

    void on_btnDisableAll_clicked();

    void on_lineEdit_RTv_returnPressed();

    void on_lineEdit_RTmu_returnPressed();

    void on_lineEdit_RTi_returnPressed();

    void on_lineEdit_RTe_returnPressed();

    void on_lineEdit_RTm_returnPressed();

    void on_lineEdit_PKPF_returnPressed();

    void on_lineEdit_PKPV_returnPressed();

    void on_lineEdit_PKDV_returnPressed();

    void on_lineEdit_PTV_returnPressed();

    void on_lineEdit_PTMU_returnPressed();

    void on_lineEdit_PTI_returnPressed();

    void on_lineEdit_PTE_returnPressed();

    void on_lineEdit_PTM_returnPressed();

    void on_save_para_clicked();

    void on_btnGetParams_clicked();

    void on_btnLogOn_clicked();

    void on_btnLogOff_clicked();

    void on_lineEdit_setRl_returnPressed();

    void on_lineEdit_setPitch_returnPressed();

    void on_lineEdit_Kal_roll_returnPressed();

    void on_lineEdit_Kal_pitch_returnPressed();

    void on_lineEdit_Kal_gyroroll_returnPressed();

    void on_lineEdit_Kal_gyropitch_returnPressed();

    void on_checkBoxPlotRoll_toggled(bool checked);
    void on_checkBoxPlotKlmRoll_toggled(bool checked);
    void on_checkBoxPlotGyro_toggled(bool checked);
    void on_checkBoxPlotKlmGyro_toggled(bool checked);
    void on_checkBoxPlotTorque_toggled(bool checked);

private:
    Ui::Dialog *ui;
    QSerialPort *serialPort;
    // ==== BIẾN CHO LOG ĐỒ THỊ ====
    QVector<double> logTime;
    QVector<double> logRoll;
    QVector<double> logKlmroll;
    QVector<double> logGyro;
    QVector<double> logKlmgyro;
    QVector<double> logTorque;
    double logIndex = 0.0;   // trục X: số thứ tự mẫu
    bool capturingParams = false;
    bool isArmed = false;
    void updateArmUi();

    void openSerial();
    void sendCommand(const QString &cmd);
    void setupPlot();                 // cấu hình QCustomPlot
    void addLogSample(int r, int g, int kr, int kg,int t);  // thêm 1 mẫu
    void handleLogLine(const QString &line); // xử lý 1 dòng log "a b c"
    void updateGraphVisibility();
};

#endif // DIALOG_H
