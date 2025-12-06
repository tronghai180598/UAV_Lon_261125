#include "dialog.h"
#include "ui_dialog.h"
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QDateTime>

#include <QDebug>

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
    , serialPort(new QSerialPort(this))
{
    ui->setupUi(this);
     currentLogMode = 0;

    // Thrust: 0..1000
    ui->verticalSliderThrust->setMinimum(0);
    ui->verticalSliderThrust->setMaximum(1000);
    ui->verticalSliderThrust->setValue(0);

    ui->progressBar->setRange(0, 1000);
    ui->progressBar->setValue(0);

    // Checkbox enable kênh điều khiển (roll/pitch/yaw)
    ui->checkBoxRoll->setChecked(false);
    ui->checkBoxPitch->setChecked(false);
    ui->checkBoxYaw->setChecked(false);

    // Trạng thái ban đầu: disarm
    sendCommand("disarm");
    isArmed = false;
    updateArmUi();

    openSerial();

    QObject::connect(serialPort, &QSerialPort::readyRead,
                     this, &Dialog::handleSerialData);

    // Cấu hình plot
    setupPlotMode1();
    setupPlotMode2();
}

Dialog::~Dialog()
{
    if (serialPort->isOpen())
        serialPort->close();
    delete ui;
}

void Dialog::openSerial()
{
    // ĐÚNG tên cổng của ESP8266/ESP32: kiểm tra bằng `ls /dev/ttyUSB*`
    serialPort->setPortName("/dev/ttyUSB0");
    serialPort->setBaudRate(QSerialPort::Baud115200);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (!serialPort->open(QIODevice::ReadWrite)) {
        qDebug() << "Không mở được" << serialPort->portName()
                 << ":" << serialPort->errorString();
    } else {
        qDebug() << "ĐÃ mở" << serialPort->portName();
    }
}

void Dialog::setupPlotMode1()
{
    ui->customPlot->clearGraphs();

    // graph 0: roll_H
    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setName("roll_H");
    ui->customPlot->graph(0)->setPen(QPen(Qt::red));        // ĐỎ

    // graph 1: Kalman_roll
    ui->customPlot->addGraph();
    ui->customPlot->graph(1)->setName("Kalman_roll");
    ui->customPlot->graph(1)->setPen(QPen(Qt::blue));       // XANH DƯƠNG

    // graph 2: gyro.x
    ui->customPlot->addGraph();
    ui->customPlot->graph(2)->setName("gyro.x");
    ui->customPlot->graph(2)->setPen(QPen(Qt::green));      // XANH LÁ

    // graph 3: Kalman_gyro
    ui->customPlot->addGraph();
    ui->customPlot->graph(3)->setName("Kalman_gyro");
    ui->customPlot->graph(3)->setPen(QPen(Qt::magenta));    // HỒNG TÍM

    // graph 4: torqueTarget.x * 1000 (uM)
    ui->customPlot->addGraph();
    ui->customPlot->graph(4)->setName("uM");
    ui->customPlot->graph(4)->setPen(QPen(Qt::darkYellow)); // VÀNG ĐẬM

    ui->customPlot->xAxis->setLabel("samples");
    ui->customPlot->yAxis->setLabel("value");

    ui->customPlot->legend->setVisible(true);

    // Cho phép kéo / zoom
    ui->customPlot->setInteraction(QCP::iRangeDrag, true);
    ui->customPlot->setInteraction(QCP::iRangeZoom, true);

    // Cập nhật hiển thị theo 5 checkbox plot
    updateGraphVisibility();
    ui->customPlot->replot();
}

void Dialog::setupPlotMode2()
{
    ui->customPlot->clearGraphs();

    // graph 0: torqueTarget.y*1000
    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setName("u_y*1000");
    ui->customPlot->graph(0)->setPen(QPen(Qt::red));

    // graph 1: Us
    ui->customPlot->addGraph();
    ui->customPlot->graph(1)->setName("Us");
    ui->customPlot->graph(1)->setPen(QPen(Qt::blue));

    ui->customPlot->xAxis->setLabel("samples");
    ui->customPlot->yAxis->setLabel("value");
    ui->customPlot->legend->setVisible(true);

    // Nếu bạn có checkbox riêng cho mode2, có thể điều khiển ở đây.
    ui->customPlot->replot();
}

void Dialog::addLogSample(int r, int g, int kr, int kg ,int t)
{
    logIndex += 1.0;  // mỗi mẫu tăng 1 đơn vị

    logTime.append(logIndex);
    logRoll.append(r);
    logKlmroll.append(kr);
    logGyro.append(g);
    logKlmgyro.append(kg);
    logTorque.append(t);

    // Giới hạn số điểm cho nhẹ (vd giữ tối đa 500 mẫu)
    const int maxPoints = 500;
    if (logTime.size() > maxPoints) {
        int extra = logTime.size() - maxPoints;
        logTime.remove(0, extra);
        logRoll.remove(0, extra);
        logKlmroll.remove(0, extra);
        logGyro.remove(0, extra);
        logKlmgyro.remove(0, extra);
        logTorque.remove(0, extra);
    }

    // Gán dữ liệu cho 5 graph
    if (ui->customPlot->graphCount() >= 5) {
        ui->customPlot->graph(0)->setData(logTime, logRoll);
        ui->customPlot->graph(1)->setData(logTime, logKlmroll);
        ui->customPlot->graph(2)->setData(logTime, logGyro);
        ui->customPlot->graph(3)->setData(logTime, logKlmgyro);
        ui->customPlot->graph(4)->setData(logTime, logTorque);
    }

    // X-axis auto scroll sang phải
    if (!logTime.isEmpty()) {
        ui->customPlot->xAxis->setRange(logTime.last(), 200, Qt::AlignRight);
    }

    // Y-axis autoscale
    ui->customPlot->yAxis->rescale(true);

    // Cập nhật visible theo checkbox plot
    updateGraphVisibility();

    ui->customPlot->replot();
}

void Dialog::addLogSample2(int uy, int us)
{
    logIndex2 += 1.0;

    logTime2.append(logIndex2);
    logUy.append(uy);
    logUs.append(us);

    const int maxPoints = 500;
    if (logTime2.size() > maxPoints) {
        int extra = logTime2.size() - maxPoints;
        logTime2.remove(0, extra);
        logUy.remove(0, extra);
        logUs.remove(0, extra);
    }

    if (ui->customPlot->graphCount() >= 2) {
        ui->customPlot->graph(0)->setData(logTime2, logUy);
        ui->customPlot->graph(1)->setData(logTime2, logUs);
    }

    if (!logTime2.isEmpty()) {
        ui->customPlot->xAxis->setRange(logTime2.last(), 200, Qt::AlignRight);
    }
    ui->customPlot->yAxis->rescale(true);
    ui->customPlot->replot();
}
void Dialog::on_btnSaveLog_clicked()
{
    QString baseDir = QDir::homePath() + "/Documents/project_quad/UAV_Lon_ESP8266";
    QDir dir;
    if (!dir.exists(baseDir)) {
        dir.mkpath(baseDir);
    }

    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString fileName = baseDir + "/log_" + timestamp + ".csv";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "Không mở được file để ghi:" << fileName << file.errorString();
        return;
    }

    QTextStream out(&file);

    if (currentLogMode == 1) {
        // HEADER 5 cột
        out << "index,roll_H,Kalman_roll,gyro_x,Kalman_gyro,torque\n";

        int n = logTime.size();
        for (int i = 0; i < n; ++i) {
            out << logTime.value(i)      << ","
                << logRoll.value(i)      << ","
                << logKlmroll.value(i)   << ","
                << logGyro.value(i)      << ","
                << logKlmgyro.value(i)   << ","
                << logTorque.value(i)    << "\n";
        }
    }
    else if (currentLogMode == 2) {
        // HEADER 2 cột (hoặc 3 nếu muốn index + 2 signal)
        out << "index,u_y*1000,Us\n";

        int n = logTime2.size();
        for (int i = 0; i < n; ++i) {
            out << logTime2.value(i) << ","
                << logUy.value(i)    << ","
                << logUs.value(i)    << "\n";
        }
    }
    else {
        // không có mode log đang chạy
        out << "# No log mode active\n";
    }

    file.close();
    qDebug() << "Đã lưu log vào:" << fileName;
}


void Dialog::updateArmUi()
{
    if (isArmed) {
        // Nút ARM đỏ, DISARM xám
        ui->push_arm->setStyleSheet("background-color: red; color: white; font-weight: bold;");
        ui->push_disarm->setStyleSheet("");

        ui->labelArmState->setText("ARMED");
        ui->labelArmState->setStyleSheet("background-color: red; color:white; font-weight:bold;");
    } else {
        ui->push_arm->setStyleSheet("");
        ui->push_disarm->setStyleSheet("background-color: green; color: white; font-weight: bold;");

        ui->labelArmState->setText("STOPPED");
        ui->labelArmState->setStyleSheet("background-color: green; color:white; font-weight:bold;");
    }
}

void Dialog::handleLogLine(const QString &line)
{
    QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);

    // MODE 1: 5 số
    if (currentLogMode == 1 && parts.size() >= 5) {
        bool ok1=false, ok2=false, ok3=false, ok4=false, ok5=false;
        int r  = parts[0].toInt(&ok1);
        int kr = parts[1].toInt(&ok2);
        int g  = parts[2].toInt(&ok3);
        int kg = parts[3].toInt(&ok4);
        int t  = parts[4].toInt(&ok5);

        if (ok1 && ok2 && ok3 && ok4 && ok5) {
            addLogSample(r, g, kr, kg, t); // như cũ
            return;
        }
    }

    // MODE 2: 2 số (u_y, Us)
    if (currentLogMode == 2 && parts.size() >= 2) {
        bool ok1=false, ok2=false;
        int uy = parts[0].toInt(&ok1);
        int us = parts[1].toInt(&ok2);
        if (ok1 && ok2) {
            addLogSample2(uy, us);
            return;
        }
    }

    // phần còn lại: params, text...
    if (capturingParams) {
        if (line.contains(" = ")) {
            ui->textEditLog->append(line);
            return;
        } else {
            capturingParams = false;
        }
    }
    // ui->textEditLog->append(line); // nếu muốn log mọi text
}

void Dialog::sendCommand(const QString &cmd)
{
    if (!serialPort->isOpen()) {
        qDebug() << "Serial chưa mở, không gửi được:" << cmd;
        return;
    }

    QString line = cmd;
    if (!line.endsWith('\n'))
        line.append('\n');

    QByteArray data = line.toUtf8();
    serialPort->write(data);

    qDebug() << ">> gửi:" << line.trimmed();
}

void Dialog::on_btnLog2_clicked()
{
    currentLogMode = 2;

    logTime2.clear();
    logUy.clear();
    logUs.clear();
    logIndex2 = 0.0;

    setupPlotMode2();
    sendCommand("log 2");
}


void Dialog::on_push_arm_clicked()
{
    if (ui->verticalSliderThrust->value() > 0) {
        qDebug() << "Không arm vì stick chưa về vị trí an toàn";
        ui->verticalSliderThrust->setValue(0);
        sendCommand("mtr 4 0");
        return;
    }
    sendCommand("arm");
    isArmed = true;
    updateArmUi();
}

void Dialog::on_push_disarm_clicked()
{
    sendCommand("disarm");
    isArmed = false;
    updateArmUi();

    // luôn kéo gas về 0 khi disarm
    ui->verticalSliderThrust->setValue(0);
    sendCommand("mtr 4 0");
    sendCommand("p SetRl 0");
    sendCommand("p SetPt 0");
    ui->checkBoxRoll->setChecked(false);
    ui->checkBoxPitch->setChecked(false);
    ui->checkBoxYaw->setChecked(false);
}

void Dialog::on_verticalSliderThrust_valueChanged(int value)
{
    ui->progressBar->setValue(value);
    QString cmd = QString("mtr 4 %1").arg(value);
    sendCommand(cmd);
}

void Dialog::handleSerialData()
{
    static QString buffer;

    buffer += QString::fromUtf8(serialPort->readAll());

    int idx;
    while ((idx = buffer.indexOf('\n')) != -1) {
        QString line = buffer.left(idx);
        buffer.remove(0, idx + 1);

        line.remove('\r');
        line = line.trimmed();
        if (line.isEmpty())
            continue;
        handleLogLine(line);
    }
}

// ===== CHECKBOX BẬT/TẮT KÊNH ĐIỀU KHIỂN (dscnl) =====

void Dialog::on_checkBoxRoll_toggled(bool checked)
{
    if (checked) {
        // bật kênh roll = dscnl 1 0
        sendCommand("dscnl 1 0");
    } else {
        // tắt kênh roll = dscnl 1 1
        sendCommand("dscnl 1 1");
    }
}

void Dialog::on_checkBoxPitch_toggled(bool checked)
{
    if (checked) {
        // bật kênh pitch = dscnl 2 0
        sendCommand("dscnl 2 0");
    } else {
        // tắt kênh pitch = dscnl 2 1
        sendCommand("dscnl 2 1");
    }
}

void Dialog::on_checkBoxYaw_toggled(bool checked)
{
    if (checked) {
        // bật kênh yaw = dscnl 3 0
        sendCommand("dscnl 3 0");
    } else {
        // tắt kênh yaw = dscnl 3 1
        sendCommand("dscnl 3 1");
    }
}

void Dialog::on_btnDisableAll_clicked()
{
    sendCommand("dscnl 4 1");
    ui->checkBoxRoll->blockSignals(true);
    ui->checkBoxPitch->blockSignals(true);
    ui->checkBoxYaw->blockSignals(true);
    ui->checkBoxRoll->setChecked(false);
    ui->checkBoxPitch->setChecked(false);
    ui->checkBoxYaw->setChecked(false);
    ui->checkBoxRoll->blockSignals(false);
    ui->checkBoxPitch->blockSignals(false);
    ui->checkBoxYaw->blockSignals(false);
}

// ===== FOR ROLL (tham số bộ điều khiển) =====

void Dialog::on_lineEdit_Kpf_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_Kpf->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p RKPF %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_Kpv_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_Kpv->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p RKPV %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_Kdv_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_Kdv->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p RKDV %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_RTv_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_RTv->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p RTV %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_RTmu_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_RTmu->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p RTmu %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_RTi_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_RTi->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p RTI %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_RTe_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_RTe->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p RTE %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_RTm_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_RTm->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p RTM %1").arg(value, 0, 'f', 6));
}

// ===== FOR PITCH =====

void Dialog::on_lineEdit_PKPF_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_PKPF->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p PKPF %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_PKPV_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_PKPV->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p PKPV %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_PKDV_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_PKDV->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p PKDV %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_PTV_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_PTV->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p PTV %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_PTMU_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_PTMU->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p PTmu %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_PTI_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_PTI->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p PTI %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_PTE_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_PTE->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p PTE %1").arg(value, 0, 'f', 6));
}

void Dialog::on_lineEdit_PTM_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_PTM->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p PTM %1").arg(value, 0, 'f', 6));
}

void Dialog::on_save_para_clicked()
{
    sendCommand("save");
}

void Dialog::on_btnGetParams_clicked()
{
    ui->textEditLog->clear();
    capturingParams = true;
    sendCommand("p");
}

// ===== LOG ON/OFF =====

void Dialog::on_btnLogOn_clicked()
{
    currentLogMode = 1;

    logTime.clear();
    logRoll.clear();
    logKlmroll.clear();
    logGyro.clear();
    logKlmgyro.clear();
    logTorque.clear();
    logIndex = 0.0;

    setupPlotMode1();
    sendCommand("log 1");
}

void Dialog::on_btnLogOff_clicked()
{
    sendCommand("log 0");
}

// ===== Setpoint roll/pitch (dùng int) =====

void Dialog::on_lineEdit_setRl_returnPressed()
{
    bool ok = false;
    int value = ui->lineEdit_setRl->text().toInt(&ok);
    if (!ok) return;
    sendCommand(QString("p SetRl %1").arg(value));
}

void Dialog::on_lineEdit_setPitch_returnPressed()
{
    bool ok = false;
    int value = ui->lineEdit_setPitch->text().toInt(&ok);
    if (!ok) return;
    sendCommand(QString("p SetPt %1").arg(value));
}

// ===== Kalman params =====

void Dialog::on_lineEdit_Kal_roll_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_Kal_roll->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p rKlmf %1").arg(value, 0, 'f', 4));
}

void Dialog::on_lineEdit_Kal_pitch_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_Kal_pitch->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p pKlmf %1").arg(value, 0, 'f', 4));
}

void Dialog::on_lineEdit_Kal_gyroroll_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_Kal_gyroroll->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p rKlmv %1").arg(value, 0, 'f', 4));
}

void Dialog::on_lineEdit_Kal_gyropitch_returnPressed()
{
    bool ok = false;
    double value = ui->lineEdit_Kal_gyropitch->text().toDouble(&ok);
    if (!ok) return;
    sendCommand(QString("p pKlmv %1").arg(value, 0, 'f', 4));
}

// ===== 5 CHECKBOX CHỌN TÍN HIỆU PLOT =====

void Dialog::updateGraphVisibility()
{
    if (ui->customPlot->graphCount() < 5)
        return;

    // Nếu trong .ui chưa tạo 5 checkbox plot thì nhớ tạo:
    // checkBoxPlotRoll, checkBoxPlotKlmRoll, checkBoxPlotGyro,
    // checkBoxPlotKlmGyro, checkBoxPlotTorque

    ui->customPlot->graph(0)->setVisible(ui->checkBoxPlotRoll->isChecked());
    ui->customPlot->graph(1)->setVisible(ui->checkBoxPlotKlmRoll->isChecked());
    ui->customPlot->graph(2)->setVisible(ui->checkBoxPlotGyro->isChecked());
    ui->customPlot->graph(3)->setVisible(ui->checkBoxPlotKlmGyro->isChecked());
    ui->customPlot->graph(4)->setVisible(ui->checkBoxPlotTorque->isChecked());
}

void Dialog::on_checkBoxPlotRoll_toggled(bool)
{
    updateGraphVisibility();
    ui->customPlot->replot();
}

void Dialog::on_checkBoxPlotKlmRoll_toggled(bool)
{
    updateGraphVisibility();
    ui->customPlot->replot();
}

void Dialog::on_checkBoxPlotGyro_toggled(bool)
{
    updateGraphVisibility();
    ui->customPlot->replot();
}

void Dialog::on_checkBoxPlotKlmGyro_toggled(bool)
{
    updateGraphVisibility();
    ui->customPlot->replot();
}

void Dialog::on_checkBoxPlotTorque_toggled(bool)
{
    updateGraphVisibility();
    ui->customPlot->replot();
}
